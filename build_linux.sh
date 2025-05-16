#!/bin/bash

cd joltc/build
cmake ..
make

cp lib/libjoltc.so ../../
