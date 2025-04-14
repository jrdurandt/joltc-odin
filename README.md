# jolt-odin

> !WIP: Many breaking changes may be expected! I am constantly learning about Odin, Zig (used for building) and JoltPhysics and trying to improve best I can. Hope these bindings will be useful to others.

[Odin](https://odin-lang.org/:) binding for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) using [JoltC](https://github.com/amerkoleci/joltc)

Build with zig

Binding generated with [runic](https://github.com/Samudevv/runic)

## Usage
Requires:
- git
- python
- zig (0.14.0)
- odin (duh 🙃)

`zig build` will pull the latest joltc-zig dependency and build for your current platform. It will also copy the relevant shared library to your application root.

To build for a different platform:

Linux: `zig build -Dtarget=x86_64-linux`

Windows: `zig build -Dtarget=x86_64-windows`

macOS: `zig build -Dtarget=x86_64-macos`

macOS (aarch): `zig build -Dtarget=aarch64-macos`


To use within your game, make sure it points to `jolt.odin` and the shared library is linked to your executable (put it in the same directory as your exe to make it simple). You might need to adjust the paths in `jolt-odin` to the shared library.

## Test
Run tests with: `odin test .`

## Sample
Run sample with `odin run samples -debug`

Samples is a simple application using raylib to render.
- Hold down left mouse button and use WASD to control the camera.
- Press space to toggle spawning balls.
- Left click on a ball to select/unselect it (demostrates raycasting)

This is a bit of a stress test as doing dynamic collisions of thousands of object is difficult. I get to around 3000+ balls before the pit overflows with a stable 60fps.

## Issues
Only tested on Linux (Ubuntu 24.04).
Leveraging Zig's cross-compile powers to build for Windows and macOS but NOT tested on those platforms.
Please submit any issues to platform compatibility and I will look at it. I do have access to Windows and MacOS to test, just too lazy too, will do so in future, just trying to get a stable, unsable and easily updateable binding running first.

## Manual bindings
For the old manual bindings, please see branch `backups/manual-bind`
