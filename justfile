bindgen_exe := if os() == "windows" { "bindgen.exe" } else { "bindgen.bin" }
config := "Distribution"

[working-directory("joltc")]
build-joltc shared="OFF":
    @echo "Build joltc"

    git submodule init .
    git submodule update .

    mkdir -p build
    cmake . -B build -DJPH_SAMPLES=OFF -DJPH_TESTS=OFF -DJPH_INSTALL=OFF -DJPH_BUILD_SHARED={{ shared }} -DINTERPROCEDURAL_OPTIMIZATION=OFF
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
