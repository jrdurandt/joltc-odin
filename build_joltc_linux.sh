#!/bin/bash

CONFIG="Distribution"
SHARED="OFF"

git submodule init joltc
git submodule update joltc

mkdir -p build
cmake joltc -B build \
    -DJPH_SAMPLES=OFF
    -DJPH_TESTS=OFF \
    -DJPH_INSTALL=OFF \
    -DJPH_BUILD_SHARED=$SHARED \
    -DINTERPROCEDURAL_OPTIMIZATION=OFF
cmake --build build --config $CONFIG

mkdir -p lib
mv build/lib/* lib
