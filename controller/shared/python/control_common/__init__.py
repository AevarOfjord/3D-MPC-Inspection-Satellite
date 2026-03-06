"""Shared control primitives used by all controller profiles."""

from .base import Controller

__all__ = ["Controller", "MPCController"]


def __getattr__(name: str):
    if name == "MPCController":
        from .mpc_controller import MPCController

        return MPCController
    raise AttributeError(name)
