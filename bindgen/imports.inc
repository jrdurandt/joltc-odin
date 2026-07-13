when ODIN_OS == .Windows {
	foreign import lib "lib/joltc.lib"
} else when ODIN_OS == .Linux {
	foreign import lib "system:joltc"
}
