import logging
import sys
from enum import Enum

from tqdm.contrib.logging import logging_redirect_tqdm


class Verbose(Enum):
    Error = logging.ERROR
    Warning = logging.WARNING
    Info = logging.INFO
    Debug = logging.DEBUG

    @classmethod
    def from_int(cls, verbose_int: int):
        match verbose_int:
            case 0:
                return cls.Error
            case 1:
                return cls.Warning
            case 2:
                return cls.Info
            case 3:
                return cls.Debug
            case _:
                raise RuntimeError("Verbose has only 4 possible values.")


class LevelColorFormatter(logging.Formatter):
    RESET = "\033[0m"
    COLORS = {
        logging.DEBUG: "\033[36m",  # cyan
        logging.INFO: "\033[32m",  # green
        logging.WARNING: "\033[33m",  # yellow
        logging.ERROR: "\033[31m",  # red
        logging.CRITICAL: "\033[35m",  # magenta
    }

    def __init__(self, use_color: bool = True, datefmt: str | None = None):
        super().__init__(datefmt=datefmt)
        self.use_color = use_color
        self._prefix_formatter = logging.Formatter(
            fmt="%(asctime)s - %(levelname)s - ",
            datefmt=datefmt,
        )

    def format(self, record: logging.LogRecord) -> str:
        record.message = record.getMessage()
        prefix = self._prefix_formatter.format(record)
        message = f"{prefix}{record.message}"

        if record.exc_info:
            message = f"{message}\n{self.formatException(record.exc_info)}"
        if record.stack_info:
            message = f"{message}\n{self.formatStack(record.stack_info)}"

        if not self.use_color:
            return message
        color = self.COLORS.get(record.levelno)
        if color is None:
            return message
        return f"{color}{prefix}{self.RESET}{record.message}"


def setup_logging(verbose: Verbose):
    formatter = LevelColorFormatter(
        use_color=sys.stdout.isatty(),
    )

    handler = logging.StreamHandler(stream=sys.stdout)
    handler.setFormatter(formatter)

    root_logger = logging.getLogger()
    root_logger.handlers.clear()
    root_logger.addHandler(handler)
    root_logger.setLevel(verbose.value)

    if verbose.value == logging.DEBUG:
        logging.getLogger("httpx").setLevel(logging.INFO)
    else:
        logging.getLogger("httpx").setLevel(logging.WARNING)

    return verbose != Verbose.Error


class LoggingContext:
    def __init__(self, verbose: Verbose | int):
        if isinstance(verbose, int):
            verbose = Verbose.from_int(verbose)
        self.verbose = verbose

    def __enter__(self):
        setup_logging(self.verbose)
        self._redirector = logging_redirect_tqdm()
        self._redirector.__enter__()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._redirector.__exit__(exc_type, exc_val, exc_tb)
