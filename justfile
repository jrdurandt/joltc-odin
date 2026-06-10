bindgen_exe := if os() == "windows" { "bindgen.exe" } else { "bindgen.bin" }
config := "Distribution"

[working-directory("joltc")]
build-joltc:
    @echo "Build joltc"

    git submodule init .
    git submodule update .

    # If building for windows can also look at joltc/build for batch scripts,
    # and then just copy the lib/joltc.lib to /jolt and bin/joltc.dll to root of project

    mkdir -p build
    cmake . -B build -DJPH_SAMPLES=OFF -DJPH_TESTS=OFF -DJPH_BUILD_SHARED=OFF -DINTERPROCEDURAL_OPTIMIZATION=OFF
    cmake --build build --config {{ config }}

[working-directory("odin-c-bindgen")]
build-bindgen:
    @echo "Build odin-c-bindgen"

    git submodule init .
    git submodule update .

    odin build src -out={{ bindgen_exe }}

gen-bindings: build-bindgen
    @echo "Generate joltc odin bindings"
    ./odin-c-bindgen/{{ bindgen_exe }} bindgen

run-test:
    odin test . -debug
