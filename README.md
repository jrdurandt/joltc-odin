# joltc-odin

[Odin](https://odin-lang.org/) bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) via the [joltc](https://github.com/amerkoleci/joltc) wrapper.

## Getting Started
Note: Only tested on Linux.

Download the "joltc" repo.

```
git submodule init
git submodule update
```

Checkout how to build joltc for your OS. See the `/build` directory for Windows build scripts.

In Linux you can do:
```
cd joltc/build
cmake ..
make
```
This will build with defaults, check the CMakeLists.txt for options to set. Default builds for release.

Once built, copy the shared library (in Linux it will be `/build/lib/libjoltc.so`, Windows will ne named `joltc.lib`) to the root (next to jolt.odin)

You can test test the sample:
`odin run samples`

When using this in you own game, you might need to update the path to the shared library in jolt.odin so it can find the shared library which should be next to your game executable.

I am still trying to see how I can make this easier without having to set things like LD_LIBRARY_PATH or rpath on Linux.

## TODO
Automatic generate the bindings. Currently it's done manually with some help from ChatGPT to do the tedious translations from C methods to Odin format.

Please see [jolt-odin](https://github.com/jrdurandt/jolt-odin) for same bindings but uses [joltc-zig](https://github.com/jrdurandt/joltc-zig) for cross-compiling of joltc using zig.

I will mostly be focusing on keeping jolt-odin with the zig build up to date so this might fall behind but should remain compatible.
