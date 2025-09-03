# jolt-odin

> Archived: Moved to https://gitlab.com/jrdurandt/jolt-odin

[Odin](https://odin-lang.org/:) binding for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) using [JoltC](https://github.com/amerkoleci/joltc)

Build with zig

Bindings generated with [odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen)

## Build

Ensure the joltc dependency is pulled
`git submodule init`
`git submodule update`

Linux: `build_linux.sh`
Windows: `build_windows.bat` (TODO)

Build requirements for joltc:
Linux: CMake
Windows: CMake, VS2022

## Test
Run tests with: `odin test .`

## Using
To use within your game, make sure it points to `jolt.odin` and the shared library is linked to your executable (put it in the same directory as your exe to make it simple). You might need to adjust the paths in `jolt-odin` to the shared library.

You can copy the `jolt` directory to the root of your game along with the required shared libraries (.so for Linux, .dll for Windows and .dylib for macOS).

Reference the jolt library in your game (see `samples/ballpit.odin` for examples).

```
package my_game

import jph "jolt"

//...

assert(jph.Init())
defer jph.Shutdown()

//... Setup physics and job system as required
```

## Sample
Run sample with `odin run samples -debug`

Or build with (replace .bin with .exe on Windows) `odin build samples -out:samples.bin`

Samples is a simple application using raylib to render.
- Hold down left mouse button and use WASD to control the camera.
- Press space to toggle spawning balls.
- Left click on a ball to select/unselect it (demostrates raycasting)

This is a bit of a stress test as doing dynamic collisions of thousands of object is difficult. I get to around 3000+ balls before the pit overflows with a stable 60fps.

## Bind Gen

To generate bindings from the joltc.h

1. Download and build bindgen, see: https://github.com/karl-zylinski/odin-c-bindgen
2. Ensure that the joltc dependency is pulled (`git submodule init`, `git submodule update`)
3. Run the bindgen (note: on Windows it's will be bindgen.exe and on linux/macOS it's bindgen.bin):
`bindgen.bin bindgen`
4. Generated bindings: `bindgen/temp/joltc.odin`

Can copy the bindings as required. For this package it's copied to ./jolt.odin

TODO: Script to automate this bindgen

## Issues
Only tested on Linux (Ubuntu 24.04 and Pop!_OS 22.04).

## Changes:
- Removed Zig as build dependency. I was leveriging the cross-compile but testing the Windows generated libraries causes issues. Falling back to manual build per platform to ensure it's correct.
- Added and then removed pre-built libraries. Same issue, was using Zig to build the libraries but caused issues.
- Removed runic as bindgen for odin-c-bindgen as it's simpler to use and produces nicer bindings
