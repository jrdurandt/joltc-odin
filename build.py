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
    shutil.copyfile("joltc-zig/zig-out/lib/libjoltc.so", "libjoltc.so")
elif SYSTEM == "Window":
    shutil.copyfile("joltc-zig/zig-out/lib/joltc.dll", "joltc.dll")
elif SYSTEM == "Darwin":
    shutil.copyfile("joltc-zig/zig-out/lib/libjoltc.dynlib", "libjoltc.dynlib")
