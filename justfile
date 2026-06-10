build-script := if os() == "windows" { "build_joltc_windows.bat" } else { "./build_joltc_linux.sh" }
bindgen_exe := if os() == "windows" { "bindgen.exe" } else { "bindgen.bin" }

build-joltc:
    @echo "Build joltc"
    {{ build-script }}

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
