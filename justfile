bindgen_exe := if os() == "windows" { "bindgen.exe" } else { "bindgen.bin" }
config := "Distribution"

[working-directory("joltc")]
build-joltc:
    @echo "Build joltc"

    # If building for windows can also look at joltc/build for batch scripts,
    # and then just copy the lib/joltc.lib to /jolt and bin/joltc.dll to root of project

    mkdir -p build
    cmake . -B build -DJPH_SAMPLES=OFF -DJPH_BUILD_SHARED=ON
    cmake --build build --config {{ config }}

    if [ "{{ os() }}" = "linux" ]; then \
        cp build/lib/libjoltc.so ../jolt; \
    elif [ "{{ os() }}" = "windows" ]; then \
        cp build/vs2022_x64/lib/{{ config }}/joltc.lib ../jolt; \
        cp build/vs2022_x64/bin/{{ config }}/joltc.dll ..; \
    elif [ "{{ os() }}" = "macos" ]; then \
        echo "TODO: macOS"; \
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
