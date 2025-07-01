#!/bin/bash

echo "Building joltc.."
cd joltc/build
cmake ..
make

echo "Installing libjoltc.so to /usr/local/lib..."
sudo cp lib/libjoltc.so /usr/local/lib/
echo "Installation complete."
