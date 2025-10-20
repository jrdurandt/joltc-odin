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
Please see [jolt-odin-samples](https://gitlab.com/jrdurandt/jolt-odin-samples)

## Issues
Only tested on Linux (Ubuntu 24.04 and Pop!_OS 22.04).
