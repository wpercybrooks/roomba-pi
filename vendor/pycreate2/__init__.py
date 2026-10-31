# The MIT License
#
# Copyright (c) 2017 Kevin Walchko

try:
    from importlib.metadata import version # type: ignore
except ImportError:
    from importlib_metadata import version # type: ignore

from .create2api import Create2

__license__ = 'MIT'
__author__ = 'Kevin Walchko'
# PATCH: a vendored copy is not an installed distribution, so importlib.metadata finds nothing.
try:
    __version__ = version("pycreate2")
except Exception:            # PackageNotFoundError and friends
    __version__ = "0.8.0+vendored"
