#!/usr/bin/env python3

import argparse
import platform
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path
from urllib.request import urlopen

SYSTEM = platform.system()
IS_WINDOWS = SYSTEM == "Windows"
IS_LINUX = SYSTEM == "Linux"

assert IS_WINDOWS or IS_LINUX, "Unsupported platform"

JOLTC_URL = "https://github.com/jrdurandt/joltc/archive/refs/heads/main.zip"
BINDGEN_URL = (
    "https://github.com/karl-zylinski/odin-c-bindgen/archive/refs/tags/2.0.zip"
)


# === Utility functions ===
def run(cmd, cwd=None):
    """Run a shell command, raising an exception if it fails."""
    print(f"→ Running: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=cwd, check=True)


def download_and_extract(url: str, dest: Path):
    """Download a GitHub zip archive and extract its contents directly into `dest`."""
    print(f"=== Downloading {dest.name} ===")
    dest.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpzip = Path(tmpdir) / "repo.zip"
        print(f"Downloading {url} ...")
        with urlopen(url) as response, open(tmpzip, "wb") as f:
            shutil.copyfileobj(response, f)

        print("Extracting...")
        with zipfile.ZipFile(tmpzip, "r") as zip_ref:
            zip_ref.extractall(tmpdir)

        # GitHub zips have a single root folder
        inner_dirs = [d for d in Path(tmpdir).iterdir() if d.is_dir()]
        if inner_dirs:
            inner = inner_dirs[0]
            for item in inner.iterdir():
                target = dest / item.name
                if item.is_dir():
                    shutil.copytree(item, target, dirs_exist_ok=True)
                else:
                    shutil.copy2(item, target)
        else:
            print("Warning: no subdirectory found inside archive.")


def build_joltc():
    """Download and build JoltC"""
    if not Path("joltc").exists():
        download_and_extract(JOLTC_URL, Path("joltc"))

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
        print("Run 'cd joltc/build && sudo make install && sudo ldconfig'")


def build_bindgen():
    """Download and build odin-c-bindgen."""
    if not Path("odin-c-bindgen").exists():
        download_and_extract(BINDGEN_URL, Path("odin-c-bindgen"))

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
