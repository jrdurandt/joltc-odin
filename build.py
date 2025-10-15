#!/usr/bin/env python3

import argparse
import os
import urllib.request
import zipfile
import shutil
import platform
import subprocess
import sys
from pathlib import Path

args_parser = argparse.ArgumentParser(
    prog="build.py",
    description="Build system for Jolt Physics bindings for Odin. Automatically downloads and compiles JoltC, builds the C binding generator, and generates Odin bindings.",
    epilog="Credits: Jolt Physics by Jorrit Rouwe, JoltC by Amer Koleci, Odin C Bindgen by Karl Zylinski",
)

args_parser.add_argument(
    "-build-lib",
    action="store_true",
    help="Build the Jolt Physics library for the current platform using CMake (produces .so/.dll/.dylib)",
)

args_parser.add_argument(
    "-gen-bindings",
    action="store_true",
    help="Generate Odin language bindings for JoltC (produces jolt.odin file). Requires compiled bindgen tool.",
)

args_parser.add_argument(
    "-update-joltc",
    action="store_true",
    help="Download the latest JoltC source code from GitHub (overwrites existing JoltC directory)",
)

args_parser.add_argument(
    "-update-bindgen",
    action="store_true",
    help="Download the latest Odin C Bindgen tool from GitHub (required for generating Odin bindings)",
)

args = args_parser.parse_args()

SYSTEM = platform.system()
IS_WINDOWS = SYSTEM == "Windows"
IS_LINUX = SYSTEM == "Linux"
IS_OSX = SYSTEM == "Darwin"

JOLTC_ZIP_URL = "https://github.com/amerkoleci/joltc/archive/refs/heads/main.zip"
BINDGEN_ZIP_URL = (
    "https://github.com/karl-zylinski/odin-c-bindgen/archive/refs/tags/1.0.zip"
)

assert IS_WINDOWS or IS_LINUX, "Unsupported platform"

BUILD_CONFIG_TYPE = "Distribution"

JOLTC_PATH = "joltc"
BINDGEN_PATH = "odin-c-bindgen"


def main():
    do_update_joltc = args.update_joltc

    if not os.path.exists(JOLTC_PATH):
        do_update_joltc = True

    if do_update_joltc:
        update_joltc()

    do_compile_joltc = args.build_lib
    do_gen_bindings = args.gen_bindings

    if not do_compile_joltc and not do_gen_bindings:
        print("Nothing to do. Either specify -build-lib or -gen-bindings")
        exit(1)

    if do_compile_joltc:
        compile_joltc()

    if do_gen_bindings:
        do_update_bindgen = args.update_bindgen

        if not os.path.exists(BINDGEN_PATH):
            do_update_bindgen = True

        if do_update_bindgen:
            update_bindgen()
            compile_bindgen()

        gen_bindings()


def cmd_execute(cmd):
    res = os.system(cmd)
    if res != 0:
        print(f"ERROR: Command failed with exit code {res}: {cmd}")
        exit(1)


def update_joltc():
    if os.path.exists(JOLTC_PATH):
        shutil.rmtree(JOLTC_PATH)

    temp_zip = "joltc-temp.zip"
    temp_folder = "joltc-temp"
    print("📥 Downloading JoltC source code from GitHub...")
    urllib.request.urlretrieve(JOLTC_ZIP_URL, temp_zip)

    with zipfile.ZipFile(temp_zip) as zip_file:
        zip_file.extractall(temp_folder)
        shutil.copytree(temp_folder + "/joltc-main", JOLTC_PATH)

    os.remove(temp_zip)
    shutil.rmtree(temp_folder)


def compile_joltc():
    owd = os.getcwd()
    os.chdir(JOLTC_PATH)

    print("🔨 Compiling JoltC library for", SYSTEM + "...")

    flags = (
        '-DJPH_SAMPLES=OFF -DJPH_BUILD_SHARED=ON -DCMAKE_INSTALL_PREFIX:String="SDK" -DCMAKE_BUILD_TYPE=%s'
        % BUILD_CONFIG_TYPE
    )

    os.chdir("build")
    if IS_LINUX:
        cmd_execute('cmake -S .. -G "Unix Makefiles" %s' % flags)
        cmd_execute("make")
        shutil.copy("lib/libjoltc.so", owd)
        print("✅ Successfully built libjoltc.so")
    elif IS_WINDOWS:
        cmd_execute('cmake -S .. -G "Visual Studio 17 2022" -A x64 %s' % flags)
        cmd_execute("cmake --build . --config %s" % BUILD_CONFIG_TYPE)
        shutil.copy("bin/%s/joltc.dll" % BUILD_CONFIG_TYPE, owd)
        shutil.copy("lib/%s/joltc.lib" % BUILD_CONFIG_TYPE, owd)
        print("✅ Successfully built joltc.dll and joltc.lib")
    elif IS_OSX:
        print("❌ ERROR: macOS JoltC build is not yet configured in this script")
        exit(1)

    os.chdir(owd)


def update_bindgen():
    if os.path.exists(BINDGEN_PATH):
        shutil.rmtree(BINDGEN_PATH)

    temp_zip = "bindgen-temp.zip"
    temp_folder = "bindgen-temp"
    print("📥 Downloading Odin C Bindgen tool from GitHub...")
    urllib.request.urlretrieve(BINDGEN_ZIP_URL, temp_zip)

    with zipfile.ZipFile(temp_zip) as zip_file:
        zip_file.extractall(temp_folder)
        shutil.copytree(temp_folder + "/odin-c-bindgen-1.0", BINDGEN_PATH)

    os.remove(temp_zip)
    shutil.rmtree(temp_folder)


def compile_bindgen():
    owd = os.getcwd()
    os.chdir(BINDGEN_PATH)

    print("🔨 Compiling Odin C Bindgen executable...")

    if IS_LINUX or IS_OSX:
        cmd_execute("odin build src -out:bindgen.bin")
        print("✅ Successfully built bindgen.bin")
    elif IS_WINDOWS:
        cmd_execute("odin build src -out:bindgen.exe")
        print("✅ Successfully built bindgen.exe")

    os.chdir(owd)


def gen_bindings():
    print("🔄 Generating Odin bindings from JoltC headers...")

    if IS_LINUX or IS_OSX:
        cmd_execute("./odin-c-bindgen/bindgen.bin bindgen")
    elif IS_WINDOWS:
        cmd_execute("odin-c-bindgen/bindgen.exe bindgen")

    joltc_file = Path("./bindgen/temp/joltc.odin")

    if not joltc_file.exists():
        print("❌ jolt.odin file not found - generate bindings first")
        exit(1)

    print("🧹 Cleaning enum prefixes in jolt.odin...")

    subprocess.run(
        [sys.executable, "./bindgen/clean_enums.py", "./bindgen/temp/joltc.odin"],
        capture_output=True,
        text=False,
        check=True,
    )

    print("✅ Enum cleaning completed successfully!")

    shutil.copy("./bindgen/temp/joltc.odin", "jolt.odin")
    print("✅ Successfully generated jolt.odin bindings file")


main()
