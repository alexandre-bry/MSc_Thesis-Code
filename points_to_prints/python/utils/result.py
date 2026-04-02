from dataclasses import dataclass
from typing import Callable, Generic, TypeVar, Union

T = TypeVar("T")
E = TypeVar("E", bound=BaseException)
U = TypeVar("U")


@dataclass(frozen=True)
class Ok(Generic[T]):
    value: T


@dataclass(frozen=True)
class Err(Generic[E]):
    error: E


class Result(Generic[T, E]):
    """
    Rust-like Result type:
      - Ok(value)
      - Err(error)

    Use `unwrap()` / `expect()` to fail fast.
    """

    __slots__ = ("_inner",)

    def __init__(self, inner: Union[Ok[T], Err[E]]):
        self._inner = inner

    @staticmethod
    def ok(value: T) -> "Result[T, E]":
        return Result(Ok(value))

    @staticmethod
    def err(error: E) -> "Result[T, E]":
        return Result(Err(error))

    def is_ok(self) -> bool:
        return isinstance(self._inner, Ok)

    def is_err(self) -> bool:
        return isinstance(self._inner, Err)

    def unwrap(self) -> T:
        """Return value or raise stored error."""
        if isinstance(self._inner, Ok):
            return self._inner.value
        raise self._inner.error

    def expect(self, message: str) -> T:
        """Return value or raise RuntimeError(message) chained from stored error."""
        if isinstance(self._inner, Ok):
            return self._inner.value
        raise RuntimeError(message) from self._inner.error

    def unwrap_err(self) -> E:
        """Return error or raise if this is Ok."""
        if isinstance(self._inner, Err):
            return self._inner.error
        raise RuntimeError(f"Called unwrap_err on Ok({self._inner.value!r})")

    def map(self, fn: Callable[[T], U]) -> "Result[U, E]":
        if isinstance(self._inner, Ok):
            return Result.ok(fn(self._inner.value))
        return Result.err(self._inner.error)

    def map_err(self, fn: Callable[[E], BaseException]) -> "Result[T, BaseException]":
        if isinstance(self._inner, Err):
            return Result.err(fn(self._inner.error))
        return Result.ok(self._inner.value)

    def value_or(self, default: T) -> T:
        return self._inner.value if isinstance(self._inner, Ok) else default

    def __repr__(self) -> str:
        if isinstance(self._inner, Ok):
            return f"Result.Ok({self._inner.value!r})"
        return f"Result.Err({self._inner.error!r})"
