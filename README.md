# jolt-odin

> !WIP: Many breaking changes may be expected! I am constantly learning about Odin, Zig (used for building) and JoltPhysics and trying to improve best I can. Hope these bindings will be useful to others.

[Odin](https://odin-lang.org/:) binding for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) using [JoltC](https://github.com/amerkoleci/joltc)

Build with zig

Binding generated with [odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen)

## Build
Requires:
- [zig](https://ziglang.org/) (0.14.0)

`zig build` will pull the latest joltc-zig dependency and build for your current platform. It will also copy the relevant shared library to your application root.

To build for a different platform:

Linux: `zig build -Dtarget=x86_64-linux`

Windows: `zig build -Dtarget=x86_64-windows`

macOS: `zig build -Dtarget=x86_64-macos`

macOS (aarch): `zig build -Dtarget=aarch64-macos`

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

Samples is a simple application using raylib to render.
- Hold down left mouse button and use WASD to control the camera.
- Press space to toggle spawning balls.
- Left click on a ball to select/unselect it (demostrates raycasting)

This is a bit of a stress test as doing dynamic collisions of thousands of object is difficult. I get to around 3000+ balls before the pit overflows with a stable 60fps.

## Bind Gen
TODO

## Issues
Only tested on Linux (Ubuntu 24.04 and Pop!_OS 22.04).
Leveraging Zig's cross-compile powers to build for Windows and macOS but NOT tested on those platforms.
Please submit any issues to platform compatibility and I will look at it. I do have access to Windows and MacOS to test, just too lazy too, will do so in future, just trying to get a stable, unsable and easily updateable binding running first.
