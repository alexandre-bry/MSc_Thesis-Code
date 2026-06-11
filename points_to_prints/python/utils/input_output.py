import logging
from enum import Enum
from pathlib import Path


class OutputActionAnalysisResult(Enum):
    SKIP = "skip"
    OVERWRITE = "overwrite"
    ERROR = "error"

    def __str__(self) -> str:
        return self.value


class OutputAction(Enum):
    OVERWRITE = "overwrite"
    SKIP_EXISTING = "skip_existing"
    NONE = "none"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_flags(cls, overwrite: bool, skip_existing: bool) -> "OutputAction":
        if overwrite and skip_existing:
            raise ValueError(
                "Cannot use both --overwrite and --skip_existing flags at the same time."
            )
        if overwrite:
            return OutputAction.OVERWRITE
        elif skip_existing:
            return OutputAction.SKIP_EXISTING
        else:
            return OutputAction.NONE

    def is_overwrite(self) -> bool:
        return self == OutputAction.OVERWRITE

    def is_skip_existing(self) -> bool:
        return self == OutputAction.SKIP_EXISTING

    def handle_input_output(
        self,
        message_prefix: str,
        input_files: list[Path] = [],
        output_files: list[Path] = [],
    ) -> OutputActionAnalysisResult:
        """Handle input and output files based on the specified output action.

        Parameters
        ----------
        message_prefix : str
            A prefix for log messages.
        input_files : list[Path], optional
            A list of paths to input files.
            By default [].
        output_files : list[Path], optional
            A list of paths to output files.
            By default [].

        Raises
        ------
        FileNotFoundError
            If any of the input files do not exist.
        ValueError
            If any of the input files are not valid files.
        ValueError
            If any of the input files are empty.
        FileNotFoundError
            If any of the output files do not exist.
        FileExistsError
            If any of the output files already exist and the output action is set to NONE.
        """
        # Validate input files
        for file in input_files:
            if not file.exists():
                raise FileNotFoundError(f"Input file {file} does not exist.")
            if not file.is_file():
                raise ValueError(f"Input file {file} is not a file.")
            if not file.stat().st_size > 0:
                raise ValueError(f"Input file {file} is empty.")

        # Check for existing output files
        existing_files = []
        for file_path in output_files:
            if file_path.exists():
                existing_files.append(file_path)
            else:
                file_path.parent.mkdir(parents=True, exist_ok=True)

        existing_files_str = ", ".join(str(f) for f in existing_files)
        not_existing_files_str = ", ".join(
            str(f) for f in output_files if f not in existing_files
        )

        match self:
            case OutputAction.SKIP_EXISTING:
                if len(existing_files) == len(output_files):
                    logging.info(
                        f"{message_prefix}: Output files already exist. Skipping processing."
                    )
                    return OutputActionAnalysisResult.SKIP
                if len(existing_files) > 0:
                    error_message = f"{message_prefix}: Cannot skip existing files because these files already exist: {existing_files_str} but these do not: {not_existing_files_str}."
                    logging.error(error_message)
                    raise FileNotFoundError(error_message)
            case OutputAction.OVERWRITE:
                if len(existing_files) > 0:
                    logging.info(
                        f"{message_prefix}: These output files already exist and will be overwritten: {existing_files_str}."
                    )
                    return OutputActionAnalysisResult.OVERWRITE
            case OutputAction.NONE:
                if len(existing_files) > 0:
                    error_message = f"{message_prefix}: Output files already exist: {existing_files_str}. Use --overwrite to overwrite them or --skip_existing to skip processing if they already exist."
                    logging.error(error_message)
                    raise FileExistsError(error_message)
            case _:
                logging.error(f"{message_prefix}: Invalid output action.")
                raise ValueError(f"{message_prefix}: Invalid output action.")

        return OutputActionAnalysisResult.ERROR
