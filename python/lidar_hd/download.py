import asyncio
import logging
import subprocess
from contextlib import suppress
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import httpx
from tqdm import tqdm

from ..utils import (
    Box2154,
    InputOutput,
    LoggingContext,
    OutputAction,
    OutputActionEnum,
    Point2154,
    Verbose,
)

WFS_BASE_URL = "https://data.geopf.fr/wfs"
WFS_TYPENAME = "IGNF_LIDAR-HD_METADONNEE:metadata"
WFS_MIN_QUERY_INTERVAL_SECONDS = 1.1
DEFAULT_CONCURRENCY = 10
MAX_DOWNLOAD_RETRIES = 4
RETRY_BASE_DELAY_SECONDS = 1.5
DOWNLOAD_CHUNK_SIZE = 8 * 1024 * 1024
STAC_HEADERS = {
    "Accept": "application/geo+json",
    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64; rv:109.0) Gecko/20100101 Firefox/115.0",
}


def _tile_box_to_name(tile_box: Box2154) -> str:
    """Build a 1 km tile identifier from the tile lower-left corner.

    Parameters
    ----------
    tile_box
        Tile extent in EPSG:2154.

    Returns
    -------
    str
        Tile name in ``xxxx-yyyy`` format.
    """
    return f"{tile_box.p_min.x // 1000:04d}-{tile_box.p_min.y // 1000:04d}"


def _nw_coordinates_to_tile_box(tile_name: str) -> Optional[Box2154]:
    """Convert a LiDAR HD north-west tile code to its 1 km bounding box.

    Parameters
    ----------
    tile_name
        Tile code such as ``0676-6853``.

    Returns
    -------
    Box2154 or None
        Bounding box of the tile in EPSG:2154, or ``None`` if the tile code
        is malformed.
    """
    try:
        x_str, y_str = tile_name.split("-")
        x = int(x_str) * 1000
        north_y = int(y_str) * 1000
    except (ValueError, AttributeError):
        return None

    return Box2154(Point2154(x, north_y - 1000), Point2154(x + 1000, north_y))


def _intersection_area(tile_a: Box2154, tile_b: Box2154) -> int:
    """Compute the overlap area between two EPSG:2154 tile boxes.

    Parameters
    ----------
    tile_a
        First tile box.
    tile_b
        Second tile box.

    Returns
    -------
    int
        Overlap area in square meters.
    """
    overlap_x_min = max(tile_a.p_min.x, tile_b.p_min.x)
    overlap_y_min = max(tile_a.p_min.y, tile_b.p_min.y)
    overlap_x_max = min(tile_a.p_max.x, tile_b.p_max.x)
    overlap_y_max = min(tile_a.p_max.y, tile_b.p_max.y)

    overlap_width = max(0, overlap_x_max - overlap_x_min)
    overlap_height = max(0, overlap_y_max - overlap_y_min)
    return overlap_width * overlap_height


async def _fetch_tiles_by_bbox(
    client: httpx.AsyncClient,
    semaphore: asyncio.Semaphore,
    tile_box: Box2154,
) -> List[Tuple[str, Optional[str]]]:
    """Fetch WFS metadata intersecting a tile bbox.

    Parameters
    ----------
    client
        Shared HTTP client.
    semaphore
        Concurrency gate for the WFS endpoint.
    tile_box
        Tile bbox used to build the search request.

    Returns
    -------
    list[tuple[str, str | None]]
        Pairs of tile code and download URL from the WFS response.
    """
    # Use the full tile extent instead of a tiny box around the center to avoid
    # empty responses caused by precision/coverage edge cases.
    bbox_str = ",".join(
        map(
            str,
            [
                tile_box.p_min.lon_4326,
                tile_box.p_min.lat_4326,
                tile_box.p_max.lon_4326,
                tile_box.p_max.lat_4326,
            ],
        )
    )
    search_url = (
        f"{WFS_BASE_URL}?SERVICE=WFS&REQUEST=GetFeature&VERSION=2.0.0"
        f"&TYPENAMES={WFS_TYPENAME}&OUTPUTFORMAT=application/json"
        f"&SRSNAME=CRS:84&BBOX={bbox_str},CRS:84&COUNT=100"
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
                properties = feature.get("properties", {})
                tile_name = properties.get("coordonnees_nw") or feature.get("id")
                data_href = properties.get("url_npl")
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
    chunk_size: int = DOWNLOAD_CHUNK_SIZE,
) -> Tuple[str, bool, Optional[str], Optional[Path]]:
    """Download one LAZ tile with retries and progress reporting.

    Parameters
    ----------
    client
        Shared HTTP client.
    tile_name
        Tile identifier.
    tile_url
        Download URL for the tile.
    output_path
        Destination path for the downloaded file.
    position
        tqdm row position.
    progress_lock
        Lock used to serialize progress bar updates.
    chunk_size
        Streaming chunk size in bytes.

    Returns
    -------
    tuple[str, bool, str | None, pathlib.Path | None]
        Tile name, success flag, error message, and output path.
    """
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
                async with progress_lock:
                    progress_bar = tqdm(
                        total=total_bytes,
                        desc=tile_name,
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
    """Consume queued downloads on a dedicated tqdm line.

    Parameters
    ----------
    worker_id
        Worker index used to assign the tqdm row.
    queue
        Queue of tile name and URL pairs.
    client
        Shared HTTP client.
    name_to_path
        Mapping of tile names to output paths.
    progress_lock
        Lock guarding progress bar updates.
    overall_bar
        Aggregate download progress bar.
    results
        Collected download results.
    """
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
    """Collect the best matching WFS tile for each requested tile box.

    Parameters
    ----------
    tiles
        Requested tiles in EPSG:2154.
    concurrency
        Maximum HTTP concurrency for the client.

    Returns
    -------
    dict[str, str | None]
        Mapping from requested tile name to the matched download URL.
    """
    # The WFS endpoint enforces a strict per-second rate limit; querying
    # sequentially avoids silent partial/empty responses.
    semaphore = asyncio.Semaphore(1)
    timeout = httpx.Timeout(timeout=60.0, connect=30.0)
    limits = httpx.Limits(
        max_connections=concurrency, max_keepalive_connections=concurrency
    )
    name_to_url: Dict[str, Optional[str]] = {
        _tile_box_to_name(tile_box): None for tile_box in tiles
    }

    async with httpx.AsyncClient(
        timeout=timeout, limits=limits, headers=STAC_HEADERS
    ) as client:
        with tqdm(total=len(tiles), desc="Checking tiles", unit="tile") as pbar:
            for i, tile_box in enumerate(tiles):
                logging.debug(f"Checking existence of tile: {tile_box}")
                tile_results = await _fetch_tiles_by_bbox(client, semaphore, tile_box)
                requested_tile_name = _tile_box_to_name(tile_box)

                matched_url: Optional[str] = None
                matched_tile_name: Optional[str] = None
                best_overlap = -1
                for tile_name, tile_url in tile_results:
                    candidate_box = _nw_coordinates_to_tile_box(tile_name)
                    if candidate_box is None:
                        continue

                    overlap = _intersection_area(tile_box, candidate_box)
                    if overlap > best_overlap:
                        best_overlap = overlap
                        matched_url = tile_url
                        matched_tile_name = tile_name

                if matched_url is None and tile_results:
                    logging.debug(
                        f"No overlapping tile match for {requested_tile_name}; available tiles: {[tile_name for tile_name, _ in tile_results]}"
                    )
                elif (
                    matched_tile_name is not None
                    and matched_tile_name != requested_tile_name
                ):
                    logging.debug(
                        f"Requested {requested_tile_name} matched WFS tile {matched_tile_name} with overlap {best_overlap}"
                    )

                name_to_url[requested_tile_name] = matched_url

                pbar.update(1)
                if i < len(tiles) - 1:
                    await asyncio.sleep(WFS_MIN_QUERY_INTERVAL_SECONDS)

    return name_to_url


async def download_tiles(
    name_to_url: Dict[str, str],
    name_to_path: Dict[str, Path],
    concurrency: int = DEFAULT_CONCURRENCY,
) -> Tuple[int, List[Tuple[str, str]], List[Path]]:
    """Download the selected tiles in parallel.

    Parameters
    ----------
    name_to_url
        Mapping from tile name to download URL.
    name_to_path
        Mapping from tile name to output path.
    concurrency
        Maximum number of concurrent download workers.

    Returns
    -------
    tuple[int, list[tuple[str, str]], list[pathlib.Path]]
        Downloaded count, failures, and successfully written files.
    """
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
    """Validate downloaded LAZ files using PDAL.

    Parameters
    ----------
    downloaded_files
        Paths to files that were downloaded successfully.

    Returns
    -------
    tuple[int, list[tuple[str, str]]]
        Number of valid files and the invalid file/error pairs.
    """
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


async def download_lidar_hd_data_async(
    xmin: int,
    xmax: int,
    ymin: int,
    ymax: int,
    output_path_template: Path,
    input_output: InputOutput,
    concurrency: Optional[int],
) -> None:
    """Download LIDAR HD tiles covering a requested EPSG:2154 bounding box.

    Parameters
    ----------
    xmin : int
        Minimum X coordinate of the requested bounding box in EPSG:2154.
    xmax : int
        Maximum X coordinate of the requested bounding box in EPSG:2154.
    ymin : int
        Minimum Y coordinate of the requested bounding box in EPSG:2154.
    ymax : int
        Maximum Y coordinate of the requested bounding box in EPSG:2154.
    output_path_template : Path
        Output path template supporting the tile placeholders.
    input_output: InputOutput
        The handler for input and output file issues.
    concurrency : Optional[int]
        Maximum HTTP concurrency for tile discovery and downloading.
    """
    bbox = Box2154(Point2154(xmin, ymin), Point2154(xmax, ymax))
    tiles_boxes = bbox.get_tiles_boxes()

    logging.info(f"Generated {len(tiles_boxes)} tiles for the specified bounding box.")
    logging.debug("Tiles boxes:")
    for tile_box in tiles_boxes:
        logging.debug(
            f"  EPSG:2154({tile_box.p_min.x}, {tile_box.p_min.y}) -> EPSG:4326({tile_box.p_min.lon_4326}, {tile_box.p_min.lat_4326})"
        )

    # Discover WFS tiles intersecting the requested area.
    if concurrency is None:
        concurrency = DEFAULT_CONCURRENCY
    name_to_url_with_nones = await collect_existing_tiles(
        tiles=tiles_boxes, concurrency=concurrency
    )
    name_to_url = {
        name: url for name, url in name_to_url_with_nones.items() if url is not None
    }
    logging.info(
        f"Found {len(name_to_url)} existing tiles out of {len(name_to_url_with_nones)} candidates."
    )

    # Build the download path map, skipping files that already exist when requested.
    name_to_path: Dict[str, Path] = {}
    for tile_box in tiles_boxes:
        tile_name = _tile_box_to_name(tile_box)
        tile_url = name_to_url.get(tile_name)
        if tile_url is None:
            continue

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

        name_to_path[tile_name] = Path(output_path)

    # Handle input/output
    output_actions = input_output.get_output_actions(
        message_prefix="Downloaded LiDAR HD tiles",
        output_files=list(map(lambda path: [path], name_to_path.values())),
    )

    initial_keys = list(name_to_path.keys())
    for output_action, name in zip(output_actions, initial_keys):
        if output_action == OutputActionEnum.SKIP:
            name_to_url.pop(name)
            name_to_path.pop(name)
        elif output_action == OutputActionEnum.ERROR:
            raise Exception(f"Unexpected error when handling input/output for {name}")

    # Return if all tiles have already been downloaded
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


def download_lidar_hd_data_implementation(
    xmin: int,
    xmax: int,
    ymin: int,
    ymax: int,
    output_path_template: Path,
    input_output: InputOutput,
    concurrency: Optional[int],
):
    """Download LiDAR HD data for a specified bounding box.

    Parameters
    ----------
    xmin : int
        Minimum X coordinate of the requested bounding box in EPSG:2154.
    xmax : int
        Maximum X coordinate of the requested bounding box in EPSG:2154.
    ymin : int
        Minimum Y coordinate of the requested bounding box in EPSG:2154.
    ymax : int
        Maximum Y coordinate of the requested bounding box in EPSG:2154.
    output_path_template : Path
        Path to save the downloaded files. The path can contain the values {xmin}, {ymin}, {xmax}, {ymax}, {file_name} which will be replaced with the corresponding values. The values also have their kilometre equivalents {xmin_km}, {ymin_km}, {xmax_km}, {ymax_km}.
    input_output: InputOutput
        The handler for input and output file issues.
    concurrency: Optional[int]
        Maximum number of concurrent download workers.
    """
    asyncio.run(
        download_lidar_hd_data_async(
            xmin=xmin,
            xmax=xmax,
            ymin=ymin,
            ymax=ymax,
            output_path_template=output_path_template,
            input_output=input_output,
            concurrency=concurrency,
        )
    )


def download_lidar_hd_call(
    xmin: int,
    xmax: int,
    ymin: int,
    ymax: int,
    output_path_template: Path,
    input_output: InputOutput,
    verbose: Verbose,
    concurrency: Optional[int],
):
    """Download LiDAR HD data for a specified bounding box.

    Parameters
    ----------
    xmin : int
        Minimum X coordinate of the requested bounding box in EPSG:2154.
    xmax : int
        Maximum X coordinate of the requested bounding box in EPSG:2154.
    ymin : int
        Minimum Y coordinate of the requested bounding box in EPSG:2154.
    ymax : int
        Maximum Y coordinate of the requested bounding box in EPSG:2154.
    output_path_template : Path
        Path to save the downloaded files. The path can contain the values {xmin}, {ymin}, {xmax}, {ymax}, {file_name} which will be replaced with the corresponding values. The values also have their kilometre equivalents {xmin_km}, {ymin_km}, {xmax_km}, {ymax_km}.
    input_output: InputOutput
        The handler for input and output file issues.
    verbose: Verbose
        The verbosity level for logging.
    concurrency: Optional[int]
        Maximum number of concurrent download workers.
    """
    with LoggingContext(verbose=verbose):
        download_lidar_hd_data_implementation(
            xmin=xmin,
            xmax=xmax,
            ymin=ymin,
            ymax=ymax,
            output_path_template=output_path_template,
            input_output=input_output,
            concurrency=concurrency,
        )
