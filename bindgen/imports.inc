JOLTC_SHARED :: #config(JOLTC_SHARED, false)

when ODIN_OS == .Windows {
	foreign import lib "joltc.lib"
} else when ODIN_OS == .Linux {
    when JOLTC_SHARED {
        foreign import lib "libjoltc.so"
    } else {
        @(extra_linker_flags = "-lstdc++")
	    foreign import lib {"joltc/build/lib/libjoltc.a", "joltc/build/lib/libJolt.a"}
	}
}
