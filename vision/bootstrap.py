import os
import sys


LIB_GL_DISPATCH = "/lib/aarch64-linux-gnu/libGLdispatch.so.0"


def ensure_runtime_environment() -> None:
    """
    Apply runtime compatibility fixes before importing cv2 or ultralytics.
    """

    if (
        sys.platform.startswith("linux")
        and os.path.exists(LIB_GL_DISPATCH)
        and os.environ.get("LD_PRELOAD") != LIB_GL_DISPATCH
    ):
        environment = os.environ.copy()
        environment["LD_PRELOAD"] = LIB_GL_DISPATCH

        os.execve(
            sys.executable,
            [sys.executable] + sys.argv,
            environment,
        )

    import numpy as np

    if "bool" not in np.__dict__:
        np.bool = bool
