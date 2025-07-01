when (ODIN_OS == .Linux) {
	foreign import lib "system:joltc"
} else when (ODIN_OS == .Windows) {
	foreign import lib "joltc.dll"
}
