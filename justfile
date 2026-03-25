bindgen_exe := if os() == "windows" { "bindgen.exe" } else { "bindgen.bin" }

[working-directory("joltc")]
build-joltc:
    @echo "Build joltc"

    cmake . -B build -DJPH_SAMPLES=OFF -DJPH_BUILD_SHARED=ON
    cmake --build build --config Distribution

[working-directory("joltc/build")]
install-joltc: build-joltc
    make install
    ldconfig

[working-directory("odin-c-bindgen")]
build-bindgen:
    @echo "Build odin-c-bindgen"

    odin build src -out={{ bindgen_exe }}

gen-bindings:
    @echo "Generate joltc odin bindings"
    ./odin-c-bindgen/{{ bindgen_exe }} bindgen

[working-directory("examples/ballpit")]
run-example-ballpit:
    @echo "Run examples - Ballpit"
    odin run . -debug
