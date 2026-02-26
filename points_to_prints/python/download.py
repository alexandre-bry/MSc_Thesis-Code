import asyncio
from contextlib import suppress
from pathlib import Path
import shutil
import subprocess
from typing import Dict, List, Optional, Tuple

import aiohttp
from tqdm import tqdm

try:
    import laspy
except Exception:
    laspy = None

BASE_URL = "https://api.stac.teledetection.fr"
COLLECTION_ID = "lidarhd"
DEFAULT_CONCURRENCY = 10
MAX_DOWNLOAD_RETRIES = 4
RETRY_BASE_DELAY_SECONDS = 1.5

def parse_tile_code(tile_id: str) -> Tuple[int, int]:
    """Extract easting/1000 and northing/1000 from LHD_FXX_XXXX_YYYY_PTS_LAMB93_IGN69_HD"""
    parts = tile_id.split('_')
    x_tile = int(parts[1])  # e.g. 0490
    y_tile = int(parts[2])  # e.g. 6928
    return x_tile, y_tile

def bbox_to_tile_range(xmin: float, xmax: float, ymin: float, ymax: float, tile_size=1000.0) -> Tuple[int, int, int, int]:
    """Convert EPSG:2154 bbox to tile index range (x_tile_min, x_tile_max, y_tile_min, y_tile_max)"""
    x_tile_min = int(xmin // tile_size)
    x_tile_max = int(xmax // tile_size)
    y_tile_min = int(ymin // tile_size)
    y_tile_max = int(ymax // tile_size)
    return x_tile_min, x_tile_max, y_tile_min, y_tile_max

def generate_tile_names(x_tile_min: int, x_tile_max: int, y_tile_min: int, y_tile_max: int) -> List[str]:
    """Generate all possible tile names in the range"""
    tiles = []
    for x in range(x_tile_min, x_tile_max + 1):
        for y in range(y_tile_min, y_tile_max + 1):
            # Format as 4 digits with leading zeros
            tile_name = f"LHD_FXX_{x:04d}_{y:04d}_PTS_LAMB93_IGN69_HD"
            tiles.append(tile_name)
    return tiles

async def _fetch_tile_data_url(
    session: aiohttp.ClientSession,
    semaphore: asyncio.Semaphore,
    tile_name: str,
) -> Tuple[str, Optional[str]]:
    """Fetch STAC item and return (tile_name, data_href|None)."""
    item_url = f"{BASE_URL}/collections/{COLLECTION_ID}/items/{tile_name}"
    headers = {"Accept": "application/geo+json"}
    async with semaphore:
        try:
            async with session.get(item_url, headers=headers) as resp:
                if resp.status != 200:
                    return tile_name, None
                item = await resp.json()
                data_href = item.get("assets", {}).get("data", {}).get("href")
                return tile_name, data_href
        except aiohttp.ClientError:
            return tile_name, None


async def _download_single_tile(
    session: aiohttp.ClientSession,
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

            async with session.get(tile_url) as resp:
                resp.raise_for_status()
                total_bytes = resp.content_length
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
                    async for chunk in resp.content.iter_chunked(chunk_size):
                        handle.write(chunk)
                        bytes_written += len(chunk)
                        async with progress_lock:
                            progress_bar.update(len(chunk))

                if total_bytes is not None and bytes_written < total_bytes:
                    raise aiohttp.ClientPayloadError(
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
        except (aiohttp.ClientError, asyncio.TimeoutError, OSError) as error:
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
    session: aiohttp.ClientSession,
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
                session=session,
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


async def collect_existing_tiles(
    tile_names: List[str],
    concurrency: int = DEFAULT_CONCURRENCY,
) -> Dict[str, str]:
    """Fetch all existing tiles in parallel and return tile_name -> data URL."""
    semaphore = asyncio.Semaphore(concurrency)
    timeout = aiohttp.ClientTimeout(total=120)
    connector = aiohttp.TCPConnector(limit=concurrency)
    name_to_url: Dict[str, str] = {}

    async with aiohttp.ClientSession(timeout=timeout, connector=connector) as session:
        tasks = [
            asyncio.create_task(_fetch_tile_data_url(session, semaphore, tile_name))
            for tile_name in tile_names
        ]

        with tqdm(total=len(tasks), desc="Checking tiles", unit="tile") as pbar:
            for task in asyncio.as_completed(tasks):
                tile_name, tile_url = await task
                if tile_url:
                    name_to_url[tile_name] = tile_url
                pbar.update(1)

    return name_to_url


async def download_tiles(
    name_to_url: Dict[str, str],
    output_dir: Path,
    concurrency: int = DEFAULT_CONCURRENCY,
) -> Tuple[int, List[Tuple[str, str]], List[Path]]:
    """Download tiles in parallel. Returns (downloaded_count, failed list)."""
    timeout = aiohttp.ClientTimeout(total=0)
    connector = aiohttp.TCPConnector(limit=concurrency)
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

    async with aiohttp.ClientSession(timeout=timeout, connector=connector) as session:
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
                        session=session,
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


def validate_downloaded_files(downloaded_files: List[Path]) -> Tuple[int, List[Tuple[str, str]]]:
    """Validate produced LAZ files using PDAL if available, else laspy fallback."""
    valid_count = 0
    invalid_files: List[Tuple[str, str]] = []
    has_pdal = shutil.which("pdal") is not None

    if not downloaded_files:
        return valid_count, invalid_files

    with tqdm(total=len(downloaded_files), desc="Validating files", unit="file") as pbar:
        for file_path in downloaded_files:
            is_valid = False
            error_message = "unknown validation error"

            if has_pdal:
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

            if not is_valid and laspy is not None:
                try:
                    with laspy.open(file_path) as las_file:
                        _ = las_file.header.point_count
                    is_valid = True
                except Exception as err:
                    error_message = str(err)

            if not is_valid and not has_pdal and laspy is None:
                error_message = "neither pdal nor laspy available for validation"

            if is_valid:
                valid_count += 1
            else:
                invalid_files.append((file_path.name, error_message))

            pbar.update(1)

    return valid_count, invalid_files


async def check_tile_exists_async(
    session: aiohttp.ClientSession,
    semaphore: asyncio.Semaphore,
    tile_name: str,
) -> bool:
    """Check if a specific tile exists via direct item GET."""
    url = f"{BASE_URL}/collections/{COLLECTION_ID}/items/{tile_name}"
    headers = {"Accept": "application/geo+json"}
    async with semaphore:
        try:
            async with session.get(url, headers=headers) as resp:
                return resp.status == 200
        except aiohttp.ClientError:
            return False


async def main() -> None:
    output_dir = Path("downloaded_tiles")
    output_dir.mkdir(exist_ok=True)

    concurrency = DEFAULT_CONCURRENCY
    # ---- YOUR EPSG:2154 BOUNDING BOX ----
    # Example: around the tiles you showed (adjust these coordinates)
    xmin, ymin = 649000, 6862000   # SW corner
    xmax, ymax = 650000, 6863000   # NE corner
    
    print("Generating tile names for EPSG:2154 bbox:")
    print(f"  xmin: {xmin}, xmax: {xmax}")
    print(f"  ymin: {ymin}, ymax: {ymax}")
    
    # Convert to tile indices (1km tiles)
    x_min, x_max, y_min, y_max = bbox_to_tile_range(xmin, xmax, ymin, ymax)
    print(f"\nTile range: x={x_min:04d} to {x_max:04d}, y={y_min:04d} to {y_max:04d}")
    
    # Generate all possible names
    tile_names = generate_tile_names(x_min, x_max, y_min, y_max)
    print(f"\nGenerated {len(tile_names)} tile names:")
    
    # Show first few and last few
    for name in tile_names[:3] + tile_names[-3:]:
        print(f"  {name}")
    
    # Optionally verify which ones actually exist (slow for large areas)
    print(f"\nChecking existence of first 5 tiles (optional, async)...")
    preview_tiles = tile_names[:5]
    timeout = aiohttp.ClientTimeout(total=60)
    connector = aiohttp.TCPConnector(limit=5)
    semaphore = asyncio.Semaphore(5)
    async with aiohttp.ClientSession(timeout=timeout, connector=connector) as session:
        tasks = [
            asyncio.create_task(check_tile_exists_async(session, semaphore, name))
            for name in preview_tiles
        ]
        existence_results = await asyncio.gather(*tasks)

    for name, exists in zip(preview_tiles, existence_results):
        print(f"  {name}: {'✅' if exists else '❌'}")
        
    # Example: bulk fetch all existing tiles
    print("\n" + "="*60)
    print("Fetching all tiles in bbox...")
    
    name_to_url = await collect_existing_tiles(tile_names, concurrency=concurrency)
    print(f"Found {len(name_to_url)} existing tiles out of {len(tile_names)} candidates.")

    # Download all existing tiles
    print("\n" + "="*60)
    print(f"Downloading {len(name_to_url)} existing tiles in parallel (max {concurrency})...")
    downloaded_count, failures, downloaded_files = await download_tiles(
        name_to_url,
        output_dir,
        concurrency=concurrency,
    )

    print(f"\n✅ Downloaded {downloaded_count}/{len(name_to_url)} files.")
    if failures:
        print(f"❌ {len(failures)} downloads failed:")
        for tile_name, error in failures:
            print(f"  - {tile_name}: {error}")

    print("\n" + "=" * 60)
    print("Validating downloaded files...")
    valid_count, invalid_files = validate_downloaded_files(downloaded_files)
    print(f"✅ Valid files: {valid_count}/{len(downloaded_files)}")
    if invalid_files:
        print(f"❌ Invalid files: {len(invalid_files)}")
        for file_name, error in invalid_files:
            print(f"  - {file_name}: {error}")

    print("\nDone!")


if __name__ == "__main__":
    asyncio.run(main())
    
