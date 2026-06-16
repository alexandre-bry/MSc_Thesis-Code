import logging
from enum import Enum
from pathlib import Path
from typing import List, Sequence


class OutputActionEnum(Enum):
    PROCEED = "proceed"
    SKIP = "skip"
    ERROR = "error"


class OutputAction:
    def __init__(self, action: OutputActionEnum, message: str) -> None:
        self.action = action
        self.message = message

    def __str__(self) -> str:
        return f"{self.action.value}: {self.message}"

    def __repr__(self) -> str:
        return str(self)

    def is_proceed(self) -> bool:
        return self.action == OutputActionEnum.PROCEED

    def is_skip(self) -> bool:
        return self.action == OutputActionEnum.SKIP

    def is_error(self) -> bool:
        return self.action == OutputActionEnum.ERROR

    def raise_if_error(self):
        if self.is_error():
            raise RuntimeError(self.message)
        return self

    def log(self):
        if self.is_error():
            logging.error(self.message)
        elif self.is_skip():
            logging.info(self.message)
        else:
            logging.info(self.message)
        return self


class OutputBehaviour(Enum):
    ALL_OR_NOTHING = "all_or_nothing"


class InputFileError(Exception):
    pass

    def __init__(self, message: str) -> None:
        super().__init__(message)


class InputFileNotFoundError(InputFileError):
    pass


class InputFileIsNotFileError(InputFileError):
    pass


class InputFileIsEmptyError(InputFileError):
    pass


class InputOutput(Enum):
    OVERWRITE = "overwrite"
    SKIP_EXISTING = "skip_existing"
    NONE = "none"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_flags(cls, overwrite: bool, skip_existing: bool) -> "InputOutput":
        if overwrite and skip_existing:
            raise ValueError(
                "Cannot use both --overwrite and --skip_existing flags at the same time."
            )
        if overwrite:
            return InputOutput.OVERWRITE
        elif skip_existing:
            return InputOutput.SKIP_EXISTING
        else:
            return InputOutput.NONE

    def is_overwrite(self) -> bool:
        return self == InputOutput.OVERWRITE

    def is_skip_existing(self) -> bool:
        return self == InputOutput.SKIP_EXISTING

    def handle_input(
        self,
        message_prefix: str,
        input_files: List[Path],
    ):
        """
        Validate input files.

        Parameters
        ----------
        message_prefix : str
            A prefix for log messages.
        input_files : List[Path]
            A list of paths to input files.
        Raises
        ------
        InputFileNotFoundError
            If any of the input files do not exist.
        InputFileIsNotFileError
            If any of the input files are not valid files.
        InputFileIsEmptyError
            If any of the input files are empty.
        """
        # Validate input files
        for file in input_files:
            if not file.exists():
                raise InputFileNotFoundError(f"Input file {file} does not exist.")
            if not file.is_file():
                raise InputFileIsNotFileError(f"Input file {file} is not a file.")
            if not file.stat().st_size > 0:
                raise InputFileIsEmptyError(f"Input file {file} is empty.")

    def get_output_actions(
        self,
        message_prefix: str,
        output_files: Sequence[Sequence[Path]],
    ) -> List[OutputAction]:
        """Handle input and output files based on the specified output action.

        Parameters
        ----------
        message_prefix : str
            A prefix for log messages.
        output_files : Sequence[Sequence[Path]]
            A list of list of paths to output files.
            The first level of the handles all elements independently while the second level expects all or none to exist.
            For example, if output_files = [[path1, path2], [path3]], we need path1 and path2 to both exist or none to exist, and path3 can exist or not independently.

        Returns
        -------
        List[OutputAction]
            A list of output actions corresponding to the input and output files.

        Raises
        ------
        NotImplementedError
            If the current InputOutput has an unexpected value.
        """
        # Check for existing output files
        input_outputs: List[OutputAction] = []
        for linked_output_files in output_files:
            existing_files: List[Path] = []
            for file_path in linked_output_files:
                if file_path.exists():
                    existing_files.append(file_path)
                else:
                    file_path.parent.mkdir(parents=True, exist_ok=True)

            existing_files_str = ", ".join(str(f) for f in existing_files)
            not_existing_files_str = ", ".join(
                str(f) for f in linked_output_files if f not in existing_files
            )

            match self:
                case InputOutput.SKIP_EXISTING:
                    if len(existing_files) == len(linked_output_files):
                        message = f"{message_prefix}: These output files already exist and will be skipped: {existing_files_str}."
                        input_output = OutputAction(OutputActionEnum.SKIP, message)
                    elif len(existing_files) > 0:
                        error_message = f"{message_prefix}: Cannot skip existing files because these output files already exist: {existing_files_str} but these do not: {not_existing_files_str}."
                        input_output = OutputAction(
                            OutputActionEnum.ERROR, error_message
                        )
                    else:
                        message = f"{message_prefix}: These output files do not exist and will be created: {not_existing_files_str}."
                        input_output = OutputAction(OutputActionEnum.PROCEED, message)

                case InputOutput.OVERWRITE:
                    if len(existing_files) > 0:
                        message = f"{message_prefix}: These output files already exist and will be overwritten: {existing_files_str}."
                    else:
                        message = f"{message_prefix}: These output files do not exist and will be created: {not_existing_files_str}."
                    input_output = OutputAction(OutputActionEnum.PROCEED, message)
                case InputOutput.NONE:
                    if len(existing_files) > 0:
                        error_message = f"{message_prefix}: Some output files already exist: {existing_files_str}. Use --overwrite to overwrite them or --skip_existing to skip processing if they already exist."
                        input_output = OutputAction(
                            OutputActionEnum.ERROR, error_message
                        )
                    else:
                        message = f"{message_prefix}: These output files do not exist and will be created: {not_existing_files_str}."
                        input_output = OutputAction(OutputActionEnum.PROCEED, message)
                case _:
                    message = f"{message_prefix}: Invalid InputOutput value: {self}"
                    raise NotImplementedError(message)

            input_outputs.append(input_output)

        return input_outputs

    def handle_output(
        self,
        message_prefix: str,
        behaviour: OutputBehaviour,
        output_files: Sequence[Sequence[Path]],
    ) -> OutputActionEnum:
        """Handle output files based on the specified output action.

        Parameters
        ----------
        message_prefix : str
            A prefix for log messages.
        behaviour: OutputBehaviour
            The handler for input and output file issues.
        output_files : Sequence[Sequence[Path]], optional
            A list of list of paths to output files.
            The first level of the handles all elements independently while the second level expects all or none to exist.
            For example, if output_files = [[path1, path2], [path3]], we need path1 and path2 to both exist or none to exist, and path3 can exist or not independently.

        Returns
        -------
        OutputActionEnum
            The output action corresponding to the input and output files.
        """
        output_actions = self.get_output_actions(
            message_prefix=message_prefix, output_files=output_files
        )

        match behaviour:
            case OutputBehaviour.ALL_OR_NOTHING:
                for action in output_actions:
                    action.raise_if_error().log()

                all_proceed = all(action.is_proceed() for action in output_actions)
                all_skip = all(action.is_skip() for action in output_actions)
                if all_proceed:
                    return OutputActionEnum.PROCEED
                elif all_skip:
                    return OutputActionEnum.SKIP
                else:
                    raise RuntimeError(
                        f"{message_prefix}: Inconsistent output actions: {output_actions}. All output files must either be created or skipped."
                    )
            case _:
                raise NotImplementedError(f"Invalid OutputBehaviour value: {behaviour}")
