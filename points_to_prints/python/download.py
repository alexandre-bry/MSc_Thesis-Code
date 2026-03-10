import asyncio
import logging
import subprocess
from contextlib import suppress
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import httpx
from pyproj import Transformer
from tqdm import tqdm

BASE_URL = "https://api.stac.teledetection.fr"
COLLECTION_ID = "lidarhd"
DEFAULT_CONCURRENCY = 10
MAX_DOWNLOAD_RETRIES = 4
RETRY_BASE_DELAY_SECONDS = 1.5
STAC_HEADERS = {
    "Accept": "application/geo+json",
    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/115.0",
}


class Point2154:
    x: int
    y: int
    lon_4326: float
    lat_4326: float

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self._compute_epsg_4326()

    def _compute_epsg_4326(self):
        transformer = Transformer.from_crs("EPSG:2154", "EPSG:4326", always_xy=True)
        lon, lat = transformer.transform(self.x, self.y)
        self.lon_4326 = lon
        self.lat_4326 = lat

    def __add__(self, other):
        if not isinstance(other, Point2154):
            return NotImplemented
        return Point2154(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        if not isinstance(other, Point2154):
            return NotImplemented
        return Point2154(self.x - other.x, self.y - other.y)

    def __floordiv__(self, other):
        if not isinstance(other, int):
            return NotImplemented
        return Point2154(self.x // other, self.y // other)


class Box2154:
    p_min: Point2154
    p_max: Point2154
    p_center: Point2154

    def __init__(self, p_min: Point2154, p_max: Point2154):
        self.p_min = p_min
        self.p_max = p_max
        self.p_center = (self.p_min + self.p_max) // 2

    def get_tiles_boxes(
        self, tile_size: int = 1000, tile_base_point: Point2154 = Point2154(0, 0)
    ) -> List[Box2154]:
        start = (self.p_min - tile_base_point) // tile_size
        end = (self.p_max - tile_base_point - Point2154(1, 1)) // tile_size
        tiles_boxes = []
        for x in range(start.x, end.x + 1):
            for y in range(start.y, end.y + 1):
                tile_min = tile_base_point + Point2154(x * tile_size, y * tile_size)
                tile_max = tile_min + Point2154(tile_size, tile_size)
                tiles_boxes.append(Box2154(tile_min, tile_max))
        return tiles_boxes

    def __str__(self) -> str:
        return f"Box2154(p_min=({self.p_min.x}, {self.p_min.y}), p_max=({self.p_max.x}, {self.p_max.y}))"


async def _fetch_tiles_by_bbox(
    client: httpx.AsyncClient,
    semaphore: asyncio.Semaphore,
    tile_box: Box2154,
) -> List[Tuple[str, Optional[str]]]:
    """Fetch STAC items intersecting a bbox around the center point and return list of (tile_name, data_href|None)."""
    small_box_size = 10
    small_box_margin = Point2154(small_box_size, small_box_size)
    tile_center_small_box = Box2154(
        tile_box.p_center - small_box_margin, tile_box.p_center + small_box_margin
    )
    bbox_str = ",".join(
        map(
            str,
            [
                tile_center_small_box.p_min.lon_4326,
                tile_center_small_box.p_min.lat_4326,
                tile_center_small_box.p_max.lon_4326,
                tile_center_small_box.p_max.lat_4326,
            ],
        )
    )
    search_url = (
        f"{BASE_URL}/collections/{COLLECTION_ID}/items?bbox={bbox_str}&limit=100"
    )

    logging.debug(
        f"Searching for tiles intersecting bbox around ({tile_box.p_center.x}, {tile_box.p_center.y}): {bbox_str}"
    )
    logging.debug(f"Query: {search_url}")

    async with semaphore:
        try:
            resp = await client.get(search_url)
            if resp.status_code != 200:
                return []

            data = resp.json()
            results = []
            for feature in data.get("features", []):
                tile_name = feature.get("id")
                data_href = feature.get("assets", {}).get("data", {}).get("href")
                results.append((tile_name, data_href))
            return results
        except (httpx.HTTPError, asyncio.TimeoutError) as e:
            logging.warning(
                f"Error fetching tiles at ({tile_box.p_center.x}, {tile_box.p_center.y}): {e}"
            )
            return []


async def _download_single_tile(
    client: httpx.AsyncClient,
    tile_name: str,
    tile_url: str,
    output_path: Path,
    position: int,
    progress_lock: asyncio.Lock,
    chunk_size: int = 1024 * 1024,
) -> Tuple[str, bool, Optional[str], Optional[Path]]:
    """Download a tile file. Returns (tile_name, success, error_message)."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    last_error: Optional[Exception] = None

    for attempt in range(1, MAX_DOWNLOAD_RETRIES + 1):
        progress_bar: Optional[tqdm] = None
        try:
            if output_path.exists():
                output_path.unlink()

            async with client.stream("GET", tile_url) as resp:
                resp.raise_for_status()
                content_length_header = resp.headers.get("content-length")
                total_bytes = (
                    int(content_length_header)
                    if content_length_header and content_length_header.isdigit()
                    else None
                )
                desc = (
                    tile_name
                    if MAX_DOWNLOAD_RETRIES == 1
                    else f"{tile_name} ({attempt}/{MAX_DOWNLOAD_RETRIES})"
                )
                async with progress_lock:
                    progress_bar = tqdm(
                        total=total_bytes,
                        desc=desc,
                        unit="B",
                        unit_scale=True,
                        unit_divisor=1024,
                        position=position,
                        leave=False,
                        dynamic_ncols=True,
                    )

                bytes_written = 0
                with open(output_path, "wb") as handle:
                    async for chunk in resp.aiter_bytes(chunk_size=chunk_size):
                        handle.write(chunk)
                        bytes_written += len(chunk)
                        async with progress_lock:
                            progress_bar.update(len(chunk))

                if total_bytes is not None and bytes_written < total_bytes:
                    raise OSError(
                        f"Incomplete payload: received {bytes_written} of {total_bytes} bytes"
                    )

                async with progress_lock:
                    if attempt > 1:
                        progress_bar.set_postfix_str(f"done after retry {attempt - 1}")
                    else:
                        progress_bar.set_postfix_str("done")
                    progress_bar.close()
                    tqdm.write(f"✅ {tile_name} downloaded")
            return tile_name, True, None, output_path
        except (httpx.HTTPError, asyncio.TimeoutError, OSError) as error:
            last_error = error

            if progress_bar is not None:
                async with progress_lock:
                    if attempt < MAX_DOWNLOAD_RETRIES:
                        progress_bar.set_postfix_str("retrying")
                    else:
                        progress_bar.set_postfix_str("failed")
                    progress_bar.close()

            with suppress(OSError):
                if output_path.exists():
                    output_path.unlink()

            if attempt < MAX_DOWNLOAD_RETRIES:
                await asyncio.sleep(RETRY_BASE_DELAY_SECONDS * attempt)

    return tile_name, False, str(last_error) if last_error else "unknown error", None


async def _download_worker(
    worker_id: int,
    queue: asyncio.Queue[Optional[Tuple[str, str]]],
    client: httpx.AsyncClient,
    name_to_path: Dict[str, Path],
    progress_lock: asyncio.Lock,
    overall_bar: tqdm,
    results: List[Tuple[str, bool, Optional[str], Optional[Path]]],
) -> None:
    """Download worker bound to one tqdm line position."""
    position = worker_id + 1
    while True:
        item = await queue.get()
        try:
            if item is None:
                return

            tile_name, tile_url = item
            result = await _download_single_tile(
                client=client,
                tile_name=tile_name,
                tile_url=tile_url,
                output_path=name_to_path[tile_name],
                position=position,
                progress_lock=progress_lock,
            )
            results.append(result)
            async with progress_lock:
                overall_bar.update(1)
        finally:
            queue.task_done()


async def collect_existing_tiles(
    tiles: List[Box2154],
    concurrency: int = DEFAULT_CONCURRENCY,
) -> Dict[str, Optional[str]]:
    """Fetch all existing tiles in parallel by querying with bbox around center points. Returns tile_name -> data URL (or None if not found)."""
    semaphore = asyncio.Semaphore(concurrency)
    timeout = httpx.Timeout(timeout=60.0, connect=30.0)
    limits = httpx.Limits(
        max_connections=concurrency, max_keepalive_connections=concurrency
    )
    name_to_url: Dict[str, Optional[str]] = {}

    async with httpx.AsyncClient(
        timeout=timeout, limits=limits, headers=STAC_HEADERS
    ) as client:
        tasks: List[asyncio.Task[List[Tuple[str, Optional[str]]]]] = []
        for tile_box in tiles:
            logging.debug(f"Checking existence of tile: {tile_box}")
            tasks.append(
                asyncio.create_task(_fetch_tiles_by_bbox(client, semaphore, tile_box))
            )

        with tqdm(total=len(tasks), desc="Checking tiles", unit="tile") as pbar:
            for task in asyncio.as_completed(tasks):
                tile_results = await task
                for tile_name, tile_url in tile_results:
                    name_to_url[tile_name] = tile_url
                pbar.update(1)

    return name_to_url


async def download_tiles(
    name_to_url: Dict[str, str],
    name_to_path: Dict[str, Path],
    concurrency: int = DEFAULT_CONCURRENCY,
) -> Tuple[int, List[Tuple[str, str]], List[Path]]:
    """Download tiles in parallel. Returns (downloaded_count, failed list)."""
    timeout = httpx.Timeout(timeout=None, connect=30.0, write=60.0)
    limits = httpx.Limits(
        max_connections=concurrency, max_keepalive_connections=concurrency
    )
    failures: List[Tuple[str, str]] = []
    downloaded_count = 0
    downloaded_files: List[Path] = []
    progress_lock = asyncio.Lock()

    ordered_tiles = list(name_to_url.items())
    worker_count = min(concurrency, len(ordered_tiles)) if ordered_tiles else 0

    queue: asyncio.Queue[Optional[Tuple[str, str]]] = asyncio.Queue()
    for item in ordered_tiles:
        queue.put_nowait(item)
    for _ in range(worker_count):
        queue.put_nowait(None)

    async with httpx.AsyncClient(
        timeout=timeout, limits=limits, headers=STAC_HEADERS
    ) as client:
        results: List[Tuple[str, bool, Optional[str], Optional[Path]]] = []
        with tqdm(
            total=len(ordered_tiles),
            desc="Downloading tiles",
            unit="file",
            position=0,
            leave=True,
            dynamic_ncols=True,
        ) as overall_bar:
            workers = [
                asyncio.create_task(
                    _download_worker(
                        worker_id=worker_id,
                        queue=queue,
                        client=client,
                        name_to_path=name_to_path,
                        progress_lock=progress_lock,
                        overall_bar=overall_bar,
                        results=results,
                    )
                )
                for worker_id in range(worker_count)
            ]

            await queue.join()
            await asyncio.gather(*workers)

        for tile_name, success, error, file_path in results:
            if success:
                downloaded_count += 1
                if file_path is not None:
                    downloaded_files.append(file_path)
            else:
                failures.append((tile_name, error or "unknown error"))

    with suppress(Exception):
        tqdm.write("")

    return downloaded_count, failures, downloaded_files


def validate_downloaded_files(
    downloaded_files: List[Path],
) -> Tuple[int, List[Tuple[str, str]]]:
    """Validate produced LAZ files using PDAL."""
    valid_count = 0
    invalid_files: List[Tuple[str, str]] = []

    if not downloaded_files:
        return valid_count, invalid_files

    with tqdm(
        total=len(downloaded_files), desc="Validating files", unit="file"
    ) as pbar:
        for file_path in downloaded_files:
            is_valid = False
            error_message = "unknown validation error"

            proc = subprocess.run(
                ["pdal", "info", "--summary", str(file_path)],
                capture_output=True,
                text=True,
            )
            if proc.returncode == 0:
                is_valid = True
            else:
                stderr_text = proc.stderr.strip()
                error_message = stderr_text if stderr_text else "pdal info failed"

            if is_valid:
                valid_count += 1
            else:
                invalid_files.append((file_path.name, error_message))

            pbar.update(1)

    return valid_count, invalid_files


async def check_tile_exists_async(
    client: httpx.AsyncClient,
    semaphore: asyncio.Semaphore,
    tile_name: str,
) -> bool:
    """Check if a specific tile exists via direct item GET."""
    url = f"{BASE_URL}/collections/{COLLECTION_ID}/items/{tile_name}"
    async with semaphore:
        try:
            resp = await client.get(url)
            return resp.status_code == 200
        except httpx.HTTPError:
            return False


async def download_lidar_hd_data(
    xmin: int,
    xmax: int,
    ymin: int,
    ymax: int,
    output_path_template: Path,
    overwrite: bool,
    concurrency: int = DEFAULT_CONCURRENCY,
) -> None:
    """Main function to download LIDAR HD data for a given EPSG:2154 bounding box."""
    # Extract tile boxes covering the specified bounding box
    bbox = Box2154(Point2154(xmin, ymin), Point2154(xmax, ymax))
    tiles_boxes = bbox.get_tiles_boxes()

    logging.info(f"Generated {len(tiles_boxes)} tiles for the specified bounding box.")
    logging.debug("Tiles boxes:")
    for tile_box in tiles_boxes:
        logging.debug(
            f"  EPSG:2154({tile_box.p_min.x}, {tile_box.p_min.y}) -> EPSG:4326({tile_box.p_min.lon_4326}, {tile_box.p_min.lat_4326})"
        )

    # Find the tiles that exist by querying the STAC API with bbox around tile centers
    name_to_url_with_nones = await collect_existing_tiles(
        tiles=tiles_boxes, concurrency=concurrency
    )
    name_to_url = {
        name: url for name, url in name_to_url_with_nones.items() if url is not None
    }
    logging.info(
        f"Found {len(name_to_url)} existing tiles out of {len(name_to_url_with_nones)} candidates."
    )

    # Remove files that already exist if overwrite is disabled
    name_to_path: Dict[str, Path] = {}
    if not overwrite:
        existing_files: List[str] = []
        for (tile_name, tile_url), tile_box in zip(name_to_url.items(), tiles_boxes):
            file_name = tile_url.split("/")[-1]

            output_path_template_str = str(output_path_template)
            output_path = output_path_template_str.format(
                xmin=tile_box.p_min.x,
                ymin=tile_box.p_min.y,
                xmax=tile_box.p_max.x,
                ymax=tile_box.p_max.y,
                xmin_km=tile_box.p_min.x // 1000,
                ymin_km=tile_box.p_min.y // 1000,
                xmax_km=tile_box.p_max.x // 1000,
                ymax_km=tile_box.p_max.y // 1000,
                file_name=file_name,
            )

            if Path(output_path).exists():
                existing_files.append(file_name)
            else:
                name_to_path[tile_name] = Path(output_path)

        name_to_url = {name: name_to_url[name] for name in name_to_path.keys()}

        logging.info(f"{len(existing_files)} files already exist and will be skipped.")

    if len(name_to_url) == 0:
        logging.info("No tiles to download after filtering. Exiting.")
        return

    # Download the tiles in parallel with retries and progress bars
    downloaded_count, failures, downloaded_files = await download_tiles(
        name_to_url,
        name_to_path,
        concurrency=concurrency,
    )
    logging.info(f"Downloaded {downloaded_count}/{len(name_to_url)} files.")

    # Log any download failures
    if failures:
        logging.error(f"{len(failures)} downloads failed:")
        for tile_name, error in failures:
            logging.error(f"  - {tile_name}: {error}")

    # Validate the downloaded files using PDAL and log results
    valid_count, invalid_files = validate_downloaded_files(downloaded_files)
    logging.info(f"Valid files: {valid_count}/{len(downloaded_files)}")
    if invalid_files:
        logging.error(f"{len(invalid_files)} invalid files:")
        for file_name, error in invalid_files:
            logging.error(f"  - {file_name}: {error}")
            logging.error(f"  - {file_name}: {error}")
