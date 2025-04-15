import re
import os
import shutil
import platform
import zipfile

JOLT_FILE = "jolt/jolt.odin"
RUNE_FILE = "rune.yml"

def execute(cmd):
    res = os.system(cmd)
    if res != 0:
        print("Failed to run cmd: " + cmd)
        exit(1)

# Build
print("Building...\n")
execute("zig build")

# Generate
print("Generating...\n")
SYSTEM = platform.system()
if SYSTEM == "Linux":
    execute("runic/runic-x86_64.AppImage %s" % RUNE_FILE)
elif SYSTEM == "Window":
    with zipfile.ZipFile("runic/runic.windows-x86_64.zip") as zip_file:
        zip_file.extractall("runic")
        execute("runic/runic.windows-x86_64/runic.exe %s" % RUNE_FILE)
elif SYSTEM == "Darwin":
    ARCH = platform.architecture()
    if ARCH == "x86_64":
        execute("runic/runic.macos-x86_64 %s" % RUNE_FILE)
    elif ARCH == "aarch64":
        execute("runic/runic.macos-arm64 %s" % RUNE_FILE)

# Clean-up
print("Cleaning up...\n")

# file needs to be formatted before running cleanup
# Requires odinfmt (from odin lsp (ols))
execute("odinfmt %s > %s" % (JOLT_FILE, "temp.odin"))
shutil.move("temp.odin", JOLT_FILE)

# Clean-up some of the generated bindings
# - Removes `(...)`
# - Removes the redundant Force32 and Count from enums
# - Replaces Matrix4x4 :: [4][4]f32 with matrix[4,4]f32
# - Adds DEFAULT_CONVEX_RADIUS to procs with convexRadius
# - Replace b8 with bool
# - Remove @(link_name = "JPH_")
# - Add link_prefix= "JPH_

line_removal_pattern = re.compile(r'_JPH_\w+_(Count|Force32)\s*=\s*\d+', re.MULTILINE)
simple_cleanup_pattern = re.compile(r'`\(|f\)`?')
matrix_pattern = re.compile(r'Matrix4x4\s*::\s*\[4\]\[4\]f32')
convex_radius_pattern = re.compile(r'convexRadius:\s*f32(?!\s*=\s*DEFAULT_CONVEX_RADIUS)')
b8_pattern = re.compile(r'\bb8\b')
link_name_pattern = re.compile(r'@\s*\(link_name\s*=\s*"JPH_\w*"\)')
default_calling_convention_pattern = re.compile(r'@\(default_calling_convention\s*=\s*"c"\)')  # Matches @(default_calling_convention = "c")

with open(JOLT_FILE, "r") as file:
    lines = file.readlines()

# Filtering
filtered_lines = [line for line in lines if not line_removal_pattern.search(line) and not link_name_pattern.search(line)]

# Replacements
processed_lines = [simple_cleanup_pattern.sub('', line) for line in filtered_lines]
processed_lines = [matrix_pattern.sub('Matrix4x4 :: matrix[4,4]f32', line) for line in processed_lines]
processed_lines = [b8_pattern.sub('bool', line) for line in processed_lines]
processed_lines = [convex_radius_pattern.sub('convexRadius: f32 = DEFAULT_CONVEX_RADIUS', line) for line in processed_lines]
processed_lines = [default_calling_convention_pattern.sub('@(default_calling_convention = "c", link_prefix = "JPH_")', line) for line in processed_lines]  # Modifies calling convention

# Remove empty lines
processed_lines = [line for line in processed_lines if line.strip()]

with open(JOLT_FILE, "w") as file:
    file.writelines(processed_lines)

print("Modifications applied: Removed matching lines and cleaned up float formatting.")
