## Requires git and zig

echo "Updating submodules"
git submodule init
git submodule update

echo "Building joltc using zig"
cd joltc-zig
zig build -Doptimize=ReleaseSafe -Dshared=true
