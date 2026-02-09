"""
Shared error-handling helpers.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from functools import wraps
from typing import Any, TypeVar

logger = logging.getLogger(__name__)

F = TypeVar("F", bound=Callable[..., Any])


def with_error_context(context: str, reraise: bool = True) -> Callable[[F], F]:
    """
    Decorate a callable to add contextual logging around exceptions.

    Args:
        context: Human-readable operation label.
        reraise: If True, re-raise the original exception after logging.
    """

    def decorator(func: F) -> F:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            try:
                return func(*args, **kwargs)
            except Exception as exc:
                logger.exception("%s failed: %s", context, exc)
                if reraise:
                    raise
                return None

        return wrapper  # type: ignore[return-value]

    return decorator
