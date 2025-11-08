#!/usr/bin/env python3

import argparse
import platform
import shutil
import subprocess
from pathlib import Path

SYSTEM = platform.system()
IS_WINDOWS = SYSTEM == "Windows"
IS_LINUX = SYSTEM == "Linux"

assert IS_WINDOWS or IS_LINUX, "Unsupported platform"

# === Utility functions ===
def run(cmd, cwd=None):
    """Run a shell command, raising an exception if it fails."""
    print(f"→ Running: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=cwd, check=True)


def build_joltc():
    """Build JoltC"""

    print("=== Building Jolt ===")
    build_dir = Path("joltc") / "build"

    run(
        [
            "cmake",
            "-S",
            ".",
            "-B",
            "build",
            "-DJPH_SAMPLES=OFF",
            "-DJPH_BUILD_SHARED=ON",
        ],
        cwd="joltc",
    )
    run(["cmake", "--build", "build", "--config", "Distribution"], cwd="joltc")

    if IS_WINDOWS:
        shutil.copy(build_dir / "bin" / "Distribution" / "joltc.dll", Path.cwd())
        shutil.copy(build_dir / "lib" / "Distribution" / "joltc.lib", Path.cwd())
    elif IS_LINUX:
        print("To install, run 'cd joltc/build && sudo make install && sudo ldconfig'")


def build_bindgen():
    """Build odin-c-bindgen."""

    bindgen_exe = "bindgen.exe"
    if IS_LINUX:
        bindgen_exe = "bindgen.bin"

    print("=== Building odin-c-bindgen ===")
    run(["odin", "build", "src", "-out:%s" % bindgen_exe], cwd="odin-c-bindgen")


def gen_bindings():
    """Generate bindings using odin-c-bindgen."""

    bindgen_exe = "bindgen.exe"
    if IS_LINUX:
        bindgen_exe = "bindgen.bin"

    print("=== Generating bindings ===")
    run([str(Path("odin-c-bindgen") / bindgen_exe), "bindgen"])

# === Main ===
def main():
    parser = argparse.ArgumentParser(
        description="Cross-platform build script for joltc and odin-c-bindgen"
    )
    parser.add_argument(
        "--build-joltc", action="store_true", help="Download and build joltc"
    )
    parser.add_argument(
        "--build-bindgen", action="store_true", help="Download and build odin-c-bindgen"
    )
    parser.add_argument("--gen-bindings", action="store_true", help="Generate bindings")
    parser.add_argument("--all", action="store_true", help="Run all steps")
    args = parser.parse_args()

    if not any(vars(args).values()):
        parser.print_help()
        return

    if args.all or args.build_joltc:
        build_joltc()
    if args.all or args.build_bindgen:
        build_bindgen()
    if args.all or args.gen_bindings:
        gen_bindings()

    print("✅ Done.")


main()
