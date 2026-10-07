"""pytest fixtures for the runtime test suite."""
from __future__ import annotations

import os
import pathlib

import pytest

from harness import REPO_ROOT


@pytest.fixture(scope="session")
def runtime_binary() -> str:
    env = os.environ.get("AUV_RUNTIME_BINARY")
    path = pathlib.Path(env) if env else REPO_ROOT / "build-lightweight" / "runtime" / "auv_runtime"
    if not path.exists():
        pytest.skip(f"runtime binary not built: {path} (run cmake --build build-lightweight)")
    return str(path)
