# jolt-odin

[Odin](https://odin-lang.org/:) binding for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) using [JoltC](https://github.com/amerkoleci/joltc)

joltc build with zig

## Samples
`odin run samples ballpit.odin`

## Usage
After building, copy the shared library from `joltc-zig/zig-out/lib` (`libjoltc.so` linux/macos, `libjoltc.dll`) to the root path of your project (next the the executable).

Trying to get static libraries build with zig to work with odin to make this a easier process...
