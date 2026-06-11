from pathlib import Path

from ..utils.custom_logging import LoggingContext, Verbose
from ..utils.input_output import OutputAction


def prepare_bd_topo_implementation(
    bd_topo_source_file: Path,
    bd_topo_output_dir: Path,
    output_action: OutputAction,
):
    """Prepare the BD TOPO data for the pipeline.

    Parameters
    ----------
    bd_topo_source_file : Path
        Path to the source BD TOPO file (e.g., GeoPackage) to prepare.
    bd_topo_output_dir : Path
        Directory where the prepared BD TOPO files will be saved.
    output_action: OutputAction
        The output action to use for handling input and output files.
    """

    from ..bd_topo.convert import convert_bd_topo_implementation
    from ..bd_topo.intersections import intersections_implementation

    full_output_path = bd_topo_output_dir / f"bd_topo.parquet"
    edges_path = bd_topo_output_dir / f"edges.parquet"
    intersections_path = bd_topo_output_dir / f"intersections.parquet"
    groups_path = bd_topo_output_dir / f"building_groups.parquet"

    convert_bd_topo_implementation(
        input_path=bd_topo_source_file,
        output_path=full_output_path,
        overwrite=output_action.is_overwrite(),
        skip_existing=output_action.is_skip_existing(),
    )

    intersections_implementation(
        bd_topo_file=full_output_path,
        output_edges_file=edges_path,
        output_intersections_file=intersections_path,
        output_building_groups_file=groups_path,
        overwrite=output_action.is_overwrite(),
        skip_existing=output_action.is_skip_existing(),
    )


def prepare_bd_topo_call(
    bd_topo_source_file: Path,
    bd_topo_output_dir: Path,
    output_action: OutputAction,
    verbose: Verbose,
):
    """Prepare the BD TOPO data for the pipeline.

    Parameters
    ----------
    bd_topo_source_file : Path
        Path to the source BD TOPO file (e.g., GeoPackage) to prepare.
    bd_topo_output_dir : Path
        Directory where the prepared BD TOPO files will be saved.
    output_action: OutputAction
        The output action to use for handling input and output files.
    verbose: Verbose
        The verbosity level for logging.
    """

    with LoggingContext(verbose=verbose):
        prepare_bd_topo_implementation(
            bd_topo_source_file=bd_topo_source_file,
            bd_topo_output_dir=bd_topo_output_dir,
            output_action=output_action,
        )
