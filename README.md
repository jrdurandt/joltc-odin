# jolt-odin

[Odin](https://odin-lang.org/:) binding for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) using [JoltC](https://github.com/amerkoleci/joltc)

Bindings generated with [odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen)

## Build

Requirements:
- Odin (duh)
- Python (to run build script)
- libclang (to generate bindings)

### Building JoltC
If all you want is to build a shared library (*.so linux, .dll windows):

`python build.py -compile-joltc`

This will download and compile joltc (TODO: Windows!)

### Building and generating bindings
To generate bindings from the latest JoltC changes:

`python build.py -gen-bindings`

This will download and compile "odin-c-bindgen" and generate the bindings

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

## Issues
Only tested on Linux (Ubuntu 24.04 and Pop!_OS 22.04).
