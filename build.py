import os
import shutil
import platform

def execute(cmd):
    res = os.system(cmd)
    if res != 0:
        print("Failed to run cmd: " + cmd)
        exit(1)

print("Update git submodules")
execute("git submodule init")
execute("git submodule update --remote")

print("Build joltc-zig on native target")
root = os.getcwd()
os.chdir("joltc-zig")
execute("zig build -Doptimize=ReleaseFast -Dshared=true")

print("Copy shared library to root")
os.chdir(root)
SYSTEM = platform.system()
if SYSTEM == "Linux":
    shutil.copyfile("joltc-zig/zig-out/lib/linux/libjoltc.so", "libjoltc.so")
elif SYSTEM == "Window":
    shutil.copyfile("joltc-zig/zig-out/lib/windows/joltc.dll", "joltc.dll")
elif SYSTEM == "Darwin":
    ARCH = platform.architecture()
    if ARCH == "x86_64":
        shutil.copyfile("joltc-zig/zig-out/lib/macos_x86_64/libjoltc.dylib", "libjoltc.dynlib")
    elif ARCH == "aarch64":
        shutil.copyfile("joltc-zig/zig-out/lib/macos_aarch64/libjoltc.dylib", "libjoltc.dynlib")
