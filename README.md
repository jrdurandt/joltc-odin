# joltc-odin

[Jolt Physics](https://github.com/jrouwe/JoltPhysics) bindings for [Odin](https://odin-lang.org) via [joltc](https://github.com/amerkoleci/joltc).

## Overview

This project provides Odin language bindings for the Jolt Physics engine, enabling high-performance physics simulation in Odin applications. The bindings are automatically generated using [odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen) from the joltc C API.

## Requirements

- [Odin compiler](https://odin-lang.org)
- Python 3
- libclang (required for binding generation)

## Quick Start

### 1. Build JoltC Shared Library

To build only the shared library (`.so` on Linux, `.dll` on Windows, `.dylib` on macOS):

```bash
python build.py --build-joltc
```

This downloads and compiles joltc automatically.

**Window**
Run the above inside the VS2022 Dev Console

**Linux Installation:**
```bash
cd joltc/build
sudo make install
sudo ldconfig
```

### 2. Generate Bindings (Optional)

If you need fresh bindings from the latest JoltC changes:

```bash
python build.py --build-bindgen --gen-bindings
```

This will:
- Download and compile odin-c-bindgen
- Generate new Odin bindings from the C headers

## Usage

### In Your Project

1. Copy the `joltc-odin` directory to your project root
2. Include the appropriate shared library for your platform:
   - Linux: `libjoltc.so` (place in /usr/local/lib to make it available to the system)
   - Windows: `joltc.dll`
   - macOS: `libjoltc.dylib`
3. Import and initialize Jolt in your code:

```odin
package my_game

import jph "jolt"

main :: proc() {
    // Initialize Jolt Physics
    assert(jph.Init())
    defer jph.Shutdown()

    // Setup physics world, job system, etc.
    // See samples for complete examples
}
```

### Running Tests

```bash
odin test .
```

## Examples

For complete working examples, see the [jolt-odin-samples](https://github.com/jrdurandt/jolt-odin-samples) repository.
Run examples with `odin run examples -debug`

## Project Structure

```
joltc-odin/
├── jolt/           # Generated Odin bindings
├── build.py        # Build script
└── README.md       # This file
```

## Troubleshooting

### Linking Issues

Make sure the shared library is:
- In the same directory as your executable, or
- In your system's library path, or
- Properly referenced in your build configuration

You may need to adjust library paths in the `joltc-odin` package if your setup differs from the default.

### Build Issues

Ensure all requirements are installed and accessible in your PATH:
- Odin compiler
- Python 3
- libclang development headers

## Acknowledgements

This project builds upon the excellent work of:

- **[Jolt Physics](https://github.com/jrouwe/JoltPhysics)** - The high-performance physics engine (MIT License)
- **[joltc](https://github.com/amerkoleci/joltc)** - C bindings for Jolt Physics (MIT License)
- **[odin-c-bindgen](https://github.com/karl-zylinski/odin-c-bindgen)** - Automatic binding generator for Odin (MIT License)

## License

Please refer to the individual projects for their respective licenses.
