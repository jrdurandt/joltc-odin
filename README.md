# joltc-odin

[joltc](https://github.com/amerkoleci/joltc) bindings for [Odin](https://odin-lang.org)

Bindings generated with [odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen)

## Build
Requirements:
- Odin
- Python3
- libclang (to generate bindings)

### Building JoltC
If all you want is to build a shared library (*.so linux, .dll windows):

`python build.py --build-joltc`

This will download and compile joltc

On Linux, once build has been run the shared lib (`libjoltc.so`) can be installed via:
```
cd joltc/build
sudo make install
```

### Building and generating bindings
To generate bindings from the latest JoltC changes:

`python build.py --build-bindgen --gen-bindings`

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
Please see [jolt-odin-samples](https://github.com/jrdurandt/jolt-odin-samples)

## Acknowledgements
- [Jolt Physics](https://github.com/jrouwe/JoltPhysics)
- [joltc](https://github.com/amerkoleci/joltc)
- [odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen)
