const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});

    const joltc = b.dependency("joltc_zig", .{
        .target = target,
        .optimize = .ReleaseFast,
        .shared = true,
    });

    //Copies library to root directory
    b.getInstallStep().dependOn(&b.addInstallArtifact(joltc.artifact("joltc"), .{
        .dest_dir = .{ .override = .{ .custom = "../" } },
    }).step);

    b.installArtifact(joltc.artifact("joltc"));
}

//TODO: zig generate
//- Download runic pre-built binaries per platform
//- Run runic command to generate bindings
//- Execute clean-up script
