when (ODIN_OS == .Linux) {
    foreign import lib "system:libjoltc.so"
} else when (ODIN_OS == .Windows) {
	foreign import lib "joltc.lib"
}
