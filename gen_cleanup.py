import re

JOLT_FILE = "jolt.odin"

# Clean-up some of the generated bindings

# Removes `(...)`
# Removes the redundant Force32 and Count from enums
# Replaces Matrix4x4 :: [4][4]f32 with matrix[4,4]f32
# Adds DEFAULT_CONVEX_RADIUS to procs with convexRadius

line_removal_pattern = re.compile(r'_JPH_\w+_(Count|Force32)\s*=\s*\d+', re.MULTILINE)
simple_cleanup_pattern = re.compile(r'`\(|f\)`?')
matrix_pattern = re.compile(r'Matrix4x4\s*::\s*\[4\]\[4\]f32')
convex_radius_pattern = re.compile(r'convexRadius:\s*f32(?!\s*=\s*DEFAULT_CONVEX_RADIUS)')
b8_pattern = re.compile(r'\bb8\b')

with open(JOLT_FILE, "r") as file:
    lines = file.readlines()

filtered_lines = [line for line in lines if not line_removal_pattern.search(line)]

processed_lines = [simple_cleanup_pattern.sub('', line) for line in filtered_lines]
processed_lines = [matrix_pattern.sub('Matrix4x4 :: matrix[4,4]f32', line) for line in processed_lines]
processed_lines = [b8_pattern.sub('bool', line) for line in processed_lines]
processed_lines = [convex_radius_pattern.sub('convexRadius: f32 = DEFAULT_CONVEX_RADIUS', line) for line in processed_lines]

with open(JOLT_FILE, "w") as file:
    file.writelines(processed_lines)

print("Modifications applied: Removed matching lines and cleaned up float formatting.")
