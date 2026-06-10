JOLTC_SHARED :: #config(JOLTC_SHARED, false)

when ODIN_OS == .Windows {
	when JOLTC_SHARED {
		foreign import lib "joltc.lib"
	} else {
		foreign import lib {"lib/joltc.lib", "lib/Jolt.lib"}
	}
} else when ODIN_OS == .Linux {
	when JOLTC_SHARED {
		foreign import lib "libjoltc.so"
	} else {
		@(extra_linker_flags = "-lstdc++")
		foreign import lib {"lib/libjoltc.a", "lib/libJolt.a"}
	}
}
