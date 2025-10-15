#!/usr/bin/env python3

import argparse

args_parser = argparse.ArgumentParser(
    prog="build.py",
    description="Jolt Physics bindings for Odin, using JoltC",
    epilog= "Jolt Physics by Jorrit Rouwe, JoltC by Amer Koleci, Bindings auto generated"
)

args_parser.add_argument("-update-joltc", action="store_true", help="Download latest JoltC")
args_parser.add_argument("-compile-joltc", action="store_true", help="Compile JoltC for the current platform")
args_parser.add_argument("-debug", action="store_true", help="Build Jolt Physics debug shared library")
args_parser.add_argument("-update-bindgen", action="store_true", help="Update Odin C Bindgen")
args_parser.add_argument("-compile-bindgen", action="store_true", help="Build Odin C Bindgen. Note: Requires libclang")
args_parser.add_argument("-gen-bindings", action="store_true", help="Generate Jolt odin bindings")

import os
import urllib.request
import zipfile
import shutil
import platform

args = args_parser.parse_args()

SYSTEM = platform.system()
IS_WINDOWS = SYSTEM == "Windows"
IS_LINUX = SYSTEM == "Linux"
IS_OSX = SYSTEM == "Darwin"

JOLTC_ZIP_URL = "https://github.com/amerkoleci/joltc/archive/refs/heads/main.zip"
BINDGEN_ZIP_URL = "https://github.com/karl-zylinski/odin-c-bindgen/archive/refs/tags/1.0.zip"

assert IS_WINDOWS or IS_LINUX, "Unsupported platform"

BUILD_CONFIG_TYPE = "Distribution"
if args.debug:
    BUILD_CONFIG_TYPE = "Debug"

JOLTC_PATH = "joltc"
BINDGEN_PATH = "odin-c-bindgen"

def main():
    do_update_joltc = args.update_joltc

    if not os.path.exists(JOLTC_PATH):
        do_update_joltc = True

    if do_update_joltc:
        update_joltc()

    do_compile_joltc = do_update_joltc or args.compile_joltc

    if do_compile_joltc:
        compile_joltc()

    do_gen_bindings = args.gen_bindings

    if do_gen_bindings:
        do_update_bindgen = args.update_bindgen

        if not os.path.exists(BINDGEN_PATH):
            do_update_bindgen = True

        if do_update_bindgen:
            # update_bindgen()
            pass

        do_compile_bindgen = do_update_bindgen or args.compile_bindgen

        if do_compile_bindgen:
            compile_bindgen()

        gen_bindings()


def cmd_execute(cmd):
    res = os.system(cmd)
    if res != 0:
        print("Failed running: " + cmd)
        exit(1)

def update_joltc():
    if os.path.exists(JOLTC_PATH):
        shutil.rmtree(JOLTC_PATH)

    temp_zip = "joltc-temp.zip"
    temp_folder = "joltc-temp"
    print("Downloading JoltC...")
    urllib.request.urlretrieve(JOLTC_ZIP_URL, temp_zip)

    with zipfile.ZipFile(temp_zip) as zip_file:
        zip_file.extractall(temp_folder)
        shutil.copytree(temp_folder + "/joltc-main", JOLTC_PATH)

    os.remove(temp_zip)
    shutil.rmtree(temp_folder)

def compile_joltc():
    owd = os.getcwd()
    os.chdir(JOLTC_PATH)

    print("Building JoltC libs...")

    flags = "-DJPH_SAMPLES=OFF -DJPH_BUILD_SHARED=ON -DCMAKE_INSTALL_PREFIX:String=\"SDK\" -DCMAKE_BUILD_TYPE=%s" % BUILD_CONFIG_TYPE

    os.chdir("build")
    if IS_LINUX:
        cmd_execute("cmake -S .. -G \"Unix Makefiles\" %s" % flags)
        cmd_execute("make")
        shutil.copy("lib/libjoltc.so", owd)
    elif IS_WINDOWS:
        cmd_execute("cmake -S .. -G \"Visual Studio 17 2022\" -A x64 %s" % flags)
        cmd_execute("cmake --build . --config %s" % BUILD_CONFIG_TYPE)
        shutil.copy("bin/%s/joltc.dll" % BUILD_CONFIG_TYPE, owd)
        shutil.copy("lib/%s/joltc.lib" % BUILD_CONFIG_TYPE, owd)
    elif IS_OSX:
        print("OSX JoltC build not configured")
        exit(1)

    os.chdir(owd)

def update_bindgen():
    if os.path.exists(BINDGEN_PATH):
        shutil.rmtree(BINDGEN_PATH)

    temp_zip = "bindgen-temp.zip"
    temp_folder = "bindgen-temp"
    print("Downloading Bindgen...")
    urllib.request.urlretrieve(BINDGEN_ZIP_URL, temp_zip)

    with zipfile.ZipFile(temp_zip) as zip_file:
        zip_file.extractall(temp_folder)
        shutil.copytree(temp_folder + "/odin-c-bindgen-1.0", BINDGEN_PATH)

    os.remove(temp_zip)
    shutil.rmtree(temp_folder)

def compile_bindgen():
    owd = os.getcwd()
    os.chdir(BINDGEN_PATH)

    print("Building Bindgen executable...")

    if IS_LINUX or IS_OSX:
        cmd_execute("odin build src -out:bindgen.bin")
    elif IS_WINDOWS:
        cmd_execute("odin build src -out:bindgen.exe")

    os.chdir(owd)

def gen_bindings():
    owd = os.getcwd()

    if IS_LINUX or IS_OSX:
        cmd_execute("./odin-c-bindgen/bindgen.bin bindgen")
    elif IS_WINDOWS:
        cmd_execute("odin-c-bindgen/bindgen.exe bindgen")

    shutil.copy("./bindgen/temp/joltc.odin", "jolt.odin")

main()
