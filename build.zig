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
}
