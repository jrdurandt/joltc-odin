#!/bin/bash

CONFIG="Distribution"

git submodule init joltc
git submodule update joltc

mkdir -p build
cmake joltc -B build \
    -DJPH_SAMPLES=OFF \
    -DJPH_TESTS=OFF \
    -DJPH_INSTALL=ON \
    -DJPH_BUILD_SHARED=ON \
    -DJPH_USE_DX12=OFF \
    -DJPH_USE_MTL=OFF \
    -DJPH_USE_VK=OFF \
    -DJPH_USE_CPU_COMPUTE=OFF
cmake --build build --config $CONFIG

echo "Run 'sudo cmake --install build'"
