when (ODIN_OS == .Linux) {
	foreign import lib "libjoltc.so"
} else when(ODIN_OS == .Windows) {
	foreign import lib "joltc.dll"
} else when(ODIN_OS == .Darwin) {
	foreign import lib "libjoltc.dylib"
}
