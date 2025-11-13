package joltc_examples

import "core:fmt"
import "core:math/rand"

import jph ".."
import rl "vendor:raylib"

WINDOW_WIDTH :: 1920
WINDOW_HEIGHT :: 1080

main :: proc() {
	rl.SetConfigFlags({.MSAA_4X_HINT})
	rl.InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Jolt Odin Samples - Ballpit")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	camera := rl.Camera3D {
		position   = {30, 15, 30},
		target     = {0, 0, 0},
		up         = {0, 1, 0},
		fovy       = 45,
		projection = .PERSPECTIVE,
	}

	init_physics()
	defer destroy_physics()

	sphere_mesh := rl.GenMeshSphere(1, 8, 8)
	sphere := rl.LoadModelFromMesh(sphere_mesh)

	//Setup static objects (floor and walls)
	floor_id: jph.BodyID
	{
		floor_shape := jph.BoxShape_Create(&{25, 0.5, 25}, jph.DEFAULT_CONVEX_RADIUS)

		floor_settings := jph.BodyCreationSettings_Create3(
			cast(^jph.Shape)floor_shape,
			&{0, 0, 0},
			nil,
			.Static,
			OBJECT_LAYER_NON_MOVING,
		)
		defer jph.BodyCreationSettings_Destroy(floor_settings)

		jph.BodyCreationSettings_SetRestitution(floor_settings, 0.5)
		jph.BodyCreationSettings_SetFriction(floor_settings, 0.5)

		floor_id = jph.BodyInterface_CreateAndAddBody(
			physics.body_interface,
			floor_settings,
			.DontActivate,
		)
	}
	defer jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, floor_id)

	build_wall :: proc(size: [3]f32, position: [3]f32) -> jph.BodyID {
		size := size
		position := position

		wall_shape := jph.BoxShape_Create(&size, jph.DEFAULT_CONVEX_RADIUS)

		floor_settings := jph.BodyCreationSettings_Create3(
			cast(^jph.Shape)wall_shape,
			&position,
			nil,
			.Static,
			OBJECT_LAYER_NON_MOVING,
		)
		defer jph.BodyCreationSettings_Destroy(floor_settings)

		return jph.BodyInterface_CreateAndAddBody(
			physics.body_interface,
			floor_settings,
			.DontActivate,
		)
	}

	wall_n := build_wall({25, 2.5, 0.5}, {0, 2.5, -25})
	defer jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, wall_n)

	wall_s := build_wall({25, 2.5, 0.5}, {0, 2.5, 25})
	defer jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, wall_s)

	wall_e := build_wall({0.5, 2.5, 25.0}, {-25, 2.5, 0})
	defer jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, wall_e)

	wall_w := build_wall({0.5, 2.5, 25.0}, {25, 2.5, 0})
	defer jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, wall_w)

	balls: [dynamic]jph.BodyID
	defer {
		for ball_id in balls {
			jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, ball_id)
		}
		delete(balls)
	}

	sphere_shape := jph.SphereShape_Create(1)
	jph.PhysicsSystem_OptimizeBroadPhase(physics.system)

	removed_balls: [dynamic]int
	defer delete(removed_balls)

	is_spawning: bool
	for !rl.WindowShouldClose() {
		if rl.IsMouseButtonDown(.RIGHT) {
			rl.UpdateCamera(&camera, .FREE)
		} else if rl.IsKeyPressed(.SPACE) {
			is_spawning = !is_spawning
		}

		if is_spawning {
			ball_pos := [3]f32 {
				rand.float32_range(-22, 22),
				rand.float32_range(20, 30),
				rand.float32_range(-22, 22),
			}

			sphere_settings := jph.BodyCreationSettings_Create3(
				cast(^jph.Shape)sphere_shape,
				&ball_pos,
				nil,
				.Dynamic,
				OBJECT_LAYER_MOVING,
			)
			defer jph.BodyCreationSettings_Destroy(sphere_settings)

			ball_id := jph.BodyInterface_CreateAndAddBody(
				physics.body_interface,
				sphere_settings,
				.Activate,
			)

			append(&balls, ball_id)
		}

		update_physics()

		rl.BeginDrawing()
		defer rl.EndDrawing()

		rl.ClearBackground(rl.BLACK)
		rl.BeginMode3D(camera)
		{
			//Draw Floor
			rl.DrawCubeV({0, 0, 0}, {50, 1, 50}, rl.GREEN)

			//Draw Walls
			rl.DrawCubeV({0, 2.5, -25}, {50, 5, 1}, rl.BLUE)
			rl.DrawCubeV({0, 2.5, 25}, {50, 5, 1}, rl.BLUE)
			rl.DrawCubeV({-25, 2.5, 0}, {1, 5, 50}, rl.BLUE)
			rl.DrawCubeV({25, 2.5, 0}, {1, 5, 50}, rl.BLUE)

			rl.DrawCubeWiresV({0, 2.5, -25}, {50, 5, 1}, rl.DARKBLUE)
			rl.DrawCubeWiresV({0, 2.5, 25}, {50, 5, 1}, rl.DARKBLUE)
			rl.DrawCubeWiresV({-25, 2.5, 0}, {1, 5, 50}, rl.DARKBLUE)
			rl.DrawCubeWiresV({25, 2.5, 0}, {1, 5, 50}, rl.DARKBLUE)

			//Draw balls
			ball_loop: for ball_id, ball_index in balls {
				position: [3]f32
				rotation: quaternion128

				jph.BodyInterface_GetPosition(physics.body_interface, ball_id, &position)
				jph.BodyInterface_GetRotation(physics.body_interface, ball_id, &rotation)

				//Kill plane
				if position.y <= -50 {
					append(&removed_balls, ball_index)
					continue ball_loop
				}

				sphere.transform = rl.QuaternionToMatrix(rotation)

				is_active := jph.BodyInterface_IsActive(physics.body_interface, ball_id)
				rl.DrawModel(sphere, position, 1, is_active ? rl.RED : rl.GRAY)
				rl.DrawModelWires(sphere, position, 1, rl.DARKGRAY)
			}

			for ball_index in removed_balls {
				ball_id := balls[ball_index]
				jph.BodyInterface_RemoveAndDestroyBody(physics.body_interface, ball_id)
				unordered_remove(&balls, ball_index)
			}
			clear(&removed_balls)
		}
		rl.EndMode3D()

		rl.DrawFPS(2, 2)
		rl.DrawText(
			fmt.ctprintf(
				"Hold RIGHT mouse button and use WASD for camera control.\nLEFT click to select balls.\nPress space to %v spawning balls.\nBalls: %d",
				is_spawning ? "stop" : "start",
				len(balls),
			),
			2,
			42,
			20,
			rl.WHITE,
		)
	}
}
