"""Bounded startup wait for native Pi integration tests (PTY only)."""
import time


def wait_for_socket(process, path, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise AssertionError(f"runtime exited during startup: {process.returncode}")
        if path.exists():
            return
        time.sleep(0.02)
    raise AssertionError("runtime control socket startup timeout")
