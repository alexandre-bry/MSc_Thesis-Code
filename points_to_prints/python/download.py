import asyncio
from contextlib import suppress
from pathlib import Path
import subprocess
from typing import Dict, List, Optional, Tuple

import httpx
from tqdm import tqdm
import logging

BASE_URL = "https://api.stac.teledetection.fr"
COLLECTION_ID = "lidarhd"
DEFAULT_CONCURRENCY = 10
MAX_DOWNLOAD_RETRIES = 4
RETRY_BASE_DELAY_SECONDS = 1.5
STAC_HEADERS = {
    "Accept": "application/geo+json",
    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/115.0",
}


def bbox_to_tile_range(
    xmin: float, xmax: float, ymin: float, ymax: float, tile_size=1000.0
) -> Tuple[int, int, int, int]:
    """Convert EPSG:2154 bbox to tile index range (x_tile_min, x_tile_max, y_tile_min, y_tile_max)"""
    x_tile_min = int(xmin // tile_size)
    x_tile_max = int(xmax // tile_size)
    y_tile_min = int(ymin // tile_size)
    y_tile_max = int(ymax // tile_size)
    return x_tile_min, x_tile_max, y_tile_min, y_tile_max


def generate_tiles_center_points(
    x_tile_min: int, x_tile_max: int, y_tile_min: int, y_tile_max: int, tile_size=1000.0
):
    """Generate center points of all tiles in the range"""
    points = []
    for x in range(x_tile_min, x_tile_max + 1):
        for y in range(y_tile_min, y_tile_max + 1):
            x_center = (x + 0.5) * tile_size
            y_center = (y + 0.5) * tile_size
            points.append((x_center, y_center))
    return points


def convert_to_epsg_4326(x: float, y: float) -> Tuple[float, float]:
    """Convert EPSG:2154 coordinates to EPSG:4326 (lon, lat) using pyproj."""
    from pyproj import Transformer

    transformer = Transformer.from_crs("EPSG:2154", "EPSG:4326", always_xy=True)
    lon, lat = transformer.transform(x, y)
    return lon, lat


async def _fetch_tiles_by_bbox(
    client: httpx.AsyncClient,
    semaphore: asyncio.Semaphore,
    x_center: float,
    y_center: float,
) -> List[Tuple[str, Optional[str]]]:
    """Fetch STAC items intersecting a bbox around the center point and return list of (tile_name, data_href|None)."""
    bbox_size = 1e-6
    half_size = bbox_size / 2
    bbox = (
        x_center - half_size,
        y_center - half_size,
        x_center + half_size,
        y_center + half_size,
    )
    bbox_str = ",".join(map(str, bbox))
    search_url = (
        f"{BASE_URL}/collections/{COLLECTION_ID}/items?bbox={bbox_str}&limit=100"
    )

    logging.debug(
        f"Searching for tiles intersecting bbox around ({x_center}, {y_center}): {bbox}"
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
            logging.warning(f"Error fetching tiles at ({x_center}, {y_center}): {e}")
            return []


# async def _fetch_tile_data_url(
#     client: httpx.AsyncClient,
#     semaphore: asyncio.Semaphore,
#     tile_name: str,
# ) -> Tuple[str, Optional[str]]:
#     """Fetch STAC item and return (tile_name, data_href|None)."""
#     item_url = f"{BASE_URL}/collections/{COLLECTION_ID}/items/{tile_name}"
#     logging.debug(f"Fetching item for tile: '{tile_name}' at URL: {item_url}")
#     headers = {"Accept": "application/geo+json"}
#     async with semaphore:
#         try:
#             resp = await client.get(item_url, headers=headers)
#             if resp.status_code != 200:
#                     return tile_name, None
#                 item = resp.json()
#                 data_href = item.get("assets", {}).get("data", {}).get("href")
#                 return tile_name, data_href
#         except httpx.HTTPError:
#             return tile_name, None


async def _download_single_tile(
    client: httpx.AsyncClient,
    tile_name: str,
    tile_url: str,
    output_dir: Path,
    position: int,
    progress_lock: asyncio.Lock,
    chunk_size: int = 1024 * 1024,
) -> Tuple[str, bool, Optional[str], Optional[Path]]:
    """Download a tile file. Returns (tile_name, success, error_message)."""
    file_path = output_dir / f"{tile_name}.laz"
    last_error: Optional[Exception] = None

    for attempt in range(1, MAX_DOWNLOAD_RETRIES + 1):
        progress_bar: Optional[tqdm] = None
        try:
            if file_path.exists():
                file_path.unlink()

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
                with open(file_path, "wb") as handle:
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
            return tile_name, True, None, file_path
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
                if file_path.exists():
                    file_path.unlink()

            if attempt < MAX_DOWNLOAD_RETRIES:
                await asyncio.sleep(RETRY_BASE_DELAY_SECONDS * attempt)

    return tile_name, False, str(last_error) if last_error else "unknown error", None


async def _download_worker(
    worker_id: int,
    queue: asyncio.Queue[Optional[Tuple[str, str]]],
    client: httpx.AsyncClient,
    output_dir: Path,
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
                output_dir=output_dir,
                position=position,
                progress_lock=progress_lock,
            )
            results.append(result)
            async with progress_lock:
                overall_bar.update(1)
        finally:
            queue.task_done()


# async def collect_existing_tiles(
#     tile_names: List[str],
#     concurrency: int = DEFAULT_CONCURRENCY,
# ) -> Dict[str, str]:
#     """Fetch all existing tiles in parallel and return tile_name -> data URL."""
#     semaphore = asyncio.Semaphore(concurrency)
#     timeout = httpx.Timeout(timeout=120.0)
#     limits = httpx.Limits(max_connections=concurrency)
#     name_to_url: Dict[str, str] = {}

#     async with httpx.AsyncClient(timeout=timeout, limits=limits) as client:
#         tasks = [
#             asyncio.create_task(_fetch_tile_data_url(client, semaphore, tile_name))
#             for tile_name in tile_names
#         ]

#         for tile_name in tile_names:
#             logging.debug(f"Checking existence of tile: {tile_name}")

#         with tqdm(total=len(tasks), desc="Checking tiles", unit="tile") as pbar:
#             for task in asyncio.as_completed(tasks):
#                 tile_name, tile_url = await task
#                 if tile_url:
#                     name_to_url[tile_name] = tile_url
#                 pbar.update(1)

#     return name_to_url


async def collect_existing_tiles(
    tiles_centers: List[Tuple[float, float]],
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
        for x_center, y_center in tiles_centers:
            logging.debug(f"Checking existence of tile: {x_center}, {y_center}")
            tasks.append(
                asyncio.create_task(
                    _fetch_tiles_by_bbox(client, semaphore, x_center, y_center)
                )
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
    output_dir: Path,
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
                        output_dir=output_dir,
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
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    output_dir: Path,
    overwrite: bool,
    concurrency: int = DEFAULT_CONCURRENCY,
) -> None:
    """Main function to download LIDAR HD data for a given EPSG:2154 bounding box."""
    x_min, x_max, y_min, y_max = bbox_to_tile_range(xmin, xmax, ymin, ymax)
    tiles_centers = generate_tiles_center_points(x_min, x_max, y_min, y_max)
    tiles_centers_epsg4326 = [convert_to_epsg_4326(x, y) for x, y in tiles_centers]

    logging.info(
        f"Generated {len(tiles_centers)} tile centers for the specified bounding box."
    )

    logging.debug("Tiles centers:")
    for (x, y), (lon, lat) in zip(tiles_centers, tiles_centers_epsg4326):
        logging.debug(f"  EPSG:2154({x}, {y}) -> EPSG:4326({lon}, {lat})")

    name_to_url_with_nones = await collect_existing_tiles(
        tiles_centers=tiles_centers_epsg4326, concurrency=concurrency
    )
    name_to_url = {
        name: url for name, url in name_to_url_with_nones.items() if url is not None
    }
    logging.info(
        f"Found {len(name_to_url)} existing tiles out of {len(name_to_url_with_nones)} candidates."
    )

    downloaded_count, failures, downloaded_files = await download_tiles(
        name_to_url,
        output_dir,
        concurrency=concurrency,
    )

    logging.info(f"Downloaded {downloaded_count}/{len(name_to_url)} files.")
    if failures:
        logging.error(f"{len(failures)} downloads failed:")
        for tile_name, error in failures:
            logging.error(f"  - {tile_name}: {error}")

    valid_count, invalid_files = validate_downloaded_files(downloaded_files)
    logging.info(f"Valid files: {valid_count}/{len(downloaded_files)}")
    if invalid_files:
        logging.error(f"{len(invalid_files)} invalid files:")
        for file_name, error in invalid_files:
            logging.error(f"  - {file_name}: {error}")


# async def main() -> None:
#     output_dir = Path("downloaded_tiles")
#     output_dir.mkdir(exist_ok=True)

#     concurrency = DEFAULT_CONCURRENCY
#     # ---- YOUR EPSG:2154 BOUNDING BOX ----
#     # Example: around the tiles you showed (adjust these coordinates)
#     xmin, ymin = 649000, 6862000  # SW corner
#     xmax, ymax = 650000, 6863000  # NE corner

#     print("Generating tile names for EPSG:2154 bbox:")
#     print(f"  xmin: {xmin}, xmax: {xmax}")
#     print(f"  ymin: {ymin}, ymax: {ymax}")

#     # Convert to tile indices (1km tiles)
#     x_min, x_max, y_min, y_max = bbox_to_tile_range(xmin, xmax, ymin, ymax)
#     print(f"\nTile range: x={x_min:04d} to {x_max:04d}, y={y_min:04d} to {y_max:04d}")

#     # Generate all possible names
#     tile_names = generate_tile_names(x_min, x_max, y_min, y_max)
#     print(f"\nGenerated {len(tile_names)} tile names:")

#     # Show first few and last few
#     for name in tile_names[:3] + tile_names[-3:]:
#         print(f"  {name}")

#     # Optionally verify which ones actually exist (slow for large areas)
#     print(f"\nChecking existence of first 5 tiles (optional, async)...")
#     preview_tiles = tile_names[:5]
#     timeout = httpx.Timeout(timeout=60.0)
#     limits = httpx.Limits(max_connections=5)
#     semaphore = asyncio.Semaphore(5)
#     async with httpx.AsyncClient(timeout=timeout, limits=limits) as client:
#         tasks = [
#             asyncio.create_task(check_tile_exists_async(client, semaphore, name))
#             for name in preview_tiles
#         ]
#         existence_results = await asyncio.gather(*tasks)

#     for name, exists in zip(preview_tiles, existence_results):
#         print(f"  {name}: {'✅' if exists else '❌'}")

#     # Example: bulk fetch all existing tiles
#     print("\n" + "=" * 60)
#     print("Fetching all tiles in bbox...")

#     name_to_url = await collect_existing_tiles(tile_names, concurrency=concurrency)
#     print(
#         f"Found {len(name_to_url)} existing tiles out of {len(tile_names)} candidates."
#     )

#     # Download all existing tiles
#     print("\n" + "=" * 60)
#     print(
#         f"Downloading {len(name_to_url)} existing tiles in parallel (max {concurrency})..."
#     )
#     downloaded_count, failures, downloaded_files = await download_tiles(
#         name_to_url,
#         output_dir,
#         concurrency=concurrency,
#     )

#     print(f"\n✅ Downloaded {downloaded_count}/{len(name_to_url)} files.")
#     if failures:
#         print(f"❌ {len(failures)} downloads failed:")
#         for tile_name, error in failures:
#             print(f"  - {tile_name}: {error}")

#     print("\n" + "=" * 60)
#     print("Validating downloaded files...")
#     valid_count, invalid_files = validate_downloaded_files(downloaded_files)
#     print(f"✅ Valid files: {valid_count}/{len(downloaded_files)}")
#     if invalid_files:
#         print(f"❌ Invalid files: {len(invalid_files)}")
#         for file_name, error in invalid_files:
#             print(f"  - {file_name}: {error}")

#     print("\nDone!")


# if __name__ == "__main__":
#     asyncio.run(main())
