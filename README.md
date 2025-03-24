# jolt-odin

[Odin](https://odin-lang.org/:) binding for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) using [JoltC](https://github.com/amerkoleci/joltc)

joltc build with zig

## Samples
`odin run samples ballpit.odin`

## Usage
After building, copy the shared library from `joltc-zig/zig-out/lib` (`libjoltc.so` linux/macos, `joltc.dll`) to the root path of your project (next the the executable).

Trying to get static libraries build with zig to work with odin to make this a easier process...

## Issues
Only tested on Linux (Pop!_OS 22.04). Leveraging Zig's cross-compile powers to build for Windows and macOS but NOT tested on those platforms.

## TODO
Currenty, the bindings are hand generated (with some help from ChatGPT to do the tedious work from converting C functions to Odin format). I want this to be simpler with auto-generated bindings. 
Ideas: 
  - See [jolt](https://gitlab.com/raygarner13/jolt) that uses extensive Python scripts to generate bindings
  - Use the generated Zig buldings to translate into Odin.
