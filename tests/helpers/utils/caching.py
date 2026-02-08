"""
Caching Utilities for Satellite Control System (test-only).

Provides caching mechanisms for expensive computations.
Supports config-based caching and LRU caching.
"""

import hashlib
import json
from collections.abc import Callable
from functools import lru_cache, wraps
from typing import Any, TypeVar

F = TypeVar("F", bound=Callable[..., Any])


def cache_key_from_config(config: Any) -> str:
    """
    Generate cache key from configuration object.

    Works with Pydantic models, dictionaries, or any object with
    a serializable representation.
    """
    if hasattr(config, "model_dump_json"):
        config_str = config.model_dump_json()
    elif hasattr(config, "model_dump"):
        config_str = json.dumps(config.model_dump(), sort_keys=True)
    elif isinstance(config, dict):
        config_str = json.dumps(config, sort_keys=True)
    else:
        try:
            config_dict = dict(config) if hasattr(config, "__dict__") else {}
            config_str = json.dumps(config_dict, sort_keys=True, default=str)
        except (TypeError, ValueError):
            config_str = str(config)

    return hashlib.md5(config_str.encode()).hexdigest()


def cache_by_config(func: F | None = None, maxsize: int = 10) -> F:
    """Decorator to cache function results based on configuration hash."""
    cache: dict[str, Any] = {}

    def decorator(f: F) -> F:
        @wraps(f)
        def wrapper(config: Any, *args: Any, **kwargs: Any) -> Any:
            config_key = cache_key_from_config(config)
            args_key = str(args) + str(sorted(kwargs.items()))
            full_key = f"{config_key}:{hashlib.md5(args_key.encode()).hexdigest()}"

            if full_key in cache:
                return cache[full_key]

            result = f(config, *args, **kwargs)

            if len(cache) >= maxsize:
                oldest_key = next(iter(cache))
                del cache[oldest_key]

            cache[full_key] = result
            return result

        wrapper.cache_clear = cache.clear  # type: ignore
        wrapper.cache_info = (
            lambda: {  # type: ignore
                "size": len(cache),
                "maxsize": maxsize,
                "keys": list(cache.keys())[:5],
            }
        )

        return wrapper  # type: ignore

    if func is None:
        return decorator  # type: ignore
    return decorator(func)  # type: ignore


def cached(maxsize: int = 128) -> Callable[[F], F]:
    """Simple LRU cache decorator with configurable size."""
    return lru_cache(maxsize=maxsize)


def cache_clear_all() -> None:
    """Clear all function caches (placeholder)."""
    pass


class CacheStats:
    """Statistics for a cache."""

    def __init__(self, name: str):
        self.name = name
        self.hits = 0
        self.misses = 0
        self.size = 0
        self.maxsize = 0

    @property
    def hit_rate(self) -> float:
        total = self.hits + self.misses
        return self.hits / total if total > 0 else 0.0

    def __str__(self) -> str:
        return (
            f"{self.name}: hits={self.hits}, misses={self.misses}, "
            f"hit_rate={self.hit_rate:.2%}, size={self.size}/{self.maxsize}"
        )


def cache_with_stats(maxsize: int = 128) -> Callable[[F], F]:
    """LRU cache decorator with statistics tracking."""
    stats = CacheStats("cache")

    def decorator(func: F) -> F:
        cached_func = lru_cache(maxsize=maxsize)(func)
        stats.maxsize = maxsize

        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            result = cached_func(*args, **kwargs)
            info = cached_func.cache_info()
            stats.hits = info.hits
            stats.misses = info.misses
            stats.size = info.currsize
            stats.maxsize = info.maxsize
            return result

        wrapper.cache_clear = cached_func.cache_clear  # type: ignore
        wrapper.cache_info = cached_func.cache_info  # type: ignore
        wrapper.cache_stats = stats  # type: ignore
        return wrapper  # type: ignore

    return decorator
