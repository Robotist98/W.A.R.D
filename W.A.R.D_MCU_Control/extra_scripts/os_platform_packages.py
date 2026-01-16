Import("env")
import sys

if sys.platform.startswith("linux"):
    env.Append(PLATFORM_PACKAGES=[
        "platformio/toolchain-gccarmnoneeabi@1.140201.0",
    ])