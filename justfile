bindgen_exe := if os() == "windows" { "bindgen.exe" } else { "bindgen.bin" }
config := "Release"

[working-directory("joltc")]
build-joltc:
    @echo "Build joltc"

    mkdir -p build
    cmake . -B build -DJPH_SAMPLES=OFF -DJPH_BUILD_SHARED=ON
    cmake --build build --config {{ config }}

    ## TODO: Windows
    if [ "{{ os() }}" = "linux" ]; then \
        cp build/lib/libjoltc.so ../jolt; \
    fi \

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
