package samples

import "core:fmt"
import "core:math/rand"
import "core:strings"

import jlt ".."
import rl "vendor:raylib"

OBJECT_LAYER_NON_MOVING: jlt.ObjectLayer = 0
OBJECT_LAYER_MOVING: jlt.ObjectLayer = 1
OBJECT_LAYER_NUM :: 2

BROAD_PHASE_LAYER_NON_MOVING: jlt.BroadPhaseLayer = 0
BROAD_PHASE_LAYER_MOVING: jlt.BroadPhaseLayer = 1
BROAD_PHASE_LAYER_NUM :: 2

@(private)
build_wall :: proc(
	body_interface: ^jlt.BodyInterface,
	size: [3]f32,
	position: [3]f32,
) -> jlt.BodyID {
	size := size
	position := position

	wall_shape := jlt.BoxShape_Create(&size)

	floor_settings := jlt.BodyCreationSettings_Create3(
		cast(^jlt.Shape)wall_shape,
		&position,
		nil,
		.Static,
		OBJECT_LAYER_NON_MOVING,
	)
	defer jlt.BodyCreationSettings_Destroy(floor_settings)

	return jlt.BodyInterface_CreateAndAddBody(body_interface, floor_settings, .DontActivate)
}

main :: proc() {
	when ODIN_DEBUG {
		track_alloc: mem.Tracking_Allocator
		mem.tracking_allocator_init(&track_alloc, context.allocator)
		defer mem.tracking_allocator_destroy(&track_alloc)

		context.allocator = mem.tracking_allocator(&track_alloc)
		defer {
			for _, leak in track_alloc.allocation_map {
				fmt.printfln("%v leaked %m\n", leak.location, leak.size)
			}

			for bad_free in track_alloc.bad_free_array {
				fmt.printfln(
					"%v allocation %p was freed badly\n",
					bad_free.location,
					bad_free.memory,
				)
			}
		}
	}

	SCREEN_WIDTH :: 800
	SCREEN_HEIGHT :: 600

	is_spawning: bool

	//Setup graphics
	rl.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "jolt-odin - samples - ballpit")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	camera := rl.Camera3D {
		position   = {30, 15, 30},
		target     = {0, 0, 0},
		up         = {0, 1, 0},
		fovy       = 45,
		projection = .PERSPECTIVE,
	}

	sphere_mesh := rl.GenMeshSphere(1, 8, 16)
	sphere := rl.LoadModelFromMesh(sphere_mesh)

	//Setup physics
	ok := jlt.Init()
	defer jlt.Shutdown()
	assert(ok, "Failed to init JoltPhysics")

	job_system := jlt.JobSystemThreadPool_Create(nil)
	defer jlt.JobSystem_Destroy(job_system)

	object_layer_pair_filter := jlt.ObjectLayerPairFilterTable_Create(OBJECT_LAYER_NUM)
	jlt.ObjectLayerPairFilterTable_EnableCollision(
		object_layer_pair_filter,
		OBJECT_LAYER_MOVING,
		OBJECT_LAYER_MOVING,
	)
	jlt.ObjectLayerPairFilterTable_EnableCollision(
		object_layer_pair_filter,
		OBJECT_LAYER_MOVING,
		OBJECT_LAYER_NON_MOVING,
	)

	broad_phase_layer_interface_table := jlt.BroadPhaseLayerInterfaceTable_Create(
		OBJECT_LAYER_NUM,
		BROAD_PHASE_LAYER_NUM,
	)
	jlt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
		broad_phase_layer_interface_table,
		OBJECT_LAYER_NON_MOVING,
		BROAD_PHASE_LAYER_NON_MOVING,
	)
	jlt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
		broad_phase_layer_interface_table,
		OBJECT_LAYER_MOVING,
		BROAD_PHASE_LAYER_MOVING,
	)

	object_vs_broad_phase_layer_filter := jlt.ObjectVsBroadPhaseLayerFilterTable_Create(
		broad_phase_layer_interface_table,
		BROAD_PHASE_LAYER_NUM,
		object_layer_pair_filter,
		OBJECT_LAYER_NUM,
	)

	physics_system_settings := jlt.PhysicsSystemSettings {
		maxBodies                     = 65535,
		numBodyMutexes                = 0,
		maxBodyPairs                  = 65535,
		maxContactConstraints         = 65535,
		broadPhaseLayerInterface      = broad_phase_layer_interface_table,
		objectLayerPairFilter         = object_layer_pair_filter,
		objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
	}
	physics_system := jlt.PhysicsSystem_Create(&physics_system_settings)
	defer jlt.PhysicsSystem_Destroy(physics_system)

	body_interface := jlt.PhysicsSystem_GetBodyInterface(physics_system)

	//Setup static objects (floor and walls)
	floor_id: jlt.BodyID
	{
		floor_shape := jlt.BoxShape_Create(&{25, 0.5, 25})

		floor_settings := jlt.BodyCreationSettings_Create3(
			cast(^jlt.Shape)floor_shape,
			&{0, 0, 0},
			nil,
			.Static,
			OBJECT_LAYER_NON_MOVING,
		)
		defer jlt.BodyCreationSettings_Destroy(floor_settings)

		jlt.BodyCreationSettings_SetRestitution(floor_settings, 0.5)
		jlt.BodyCreationSettings_SetFriction(floor_settings, 0.5)

		floor_id = jlt.BodyInterface_CreateAndAddBody(
			body_interface,
			floor_settings,
			.DontActivate,
		)
	}
	defer jlt.BodyInterface_RemoveAndDestroyBody(body_interface, floor_id)

	wall_n := build_wall(body_interface, {25, 2.5, 0.5}, {0, 2.5, -25})
	defer jlt.BodyInterface_RemoveAndDestroyBody(body_interface, wall_n)

	wall_s := build_wall(body_interface, {25, 2.5, 0.5}, {0, 2.5, 25})
	defer jlt.BodyInterface_RemoveAndDestroyBody(body_interface, wall_s)

	wall_e := build_wall(body_interface, {0.5, 2.5, 25.0}, {-25, 2.5, 0})
	defer jlt.BodyInterface_RemoveAndDestroyBody(body_interface, wall_e)

	wall_w := build_wall(body_interface, {0.5, 2.5, 25.0}, {25, 2.5, 0})
	defer jlt.BodyInterface_RemoveAndDestroyBody(body_interface, wall_w)

	balls: [dynamic]jlt.BodyID
	defer {
		for ball in balls {
			jlt.BodyInterface_RemoveAndDestroyBody(body_interface, ball)
		}
		delete(balls)
	}

	sphere_shape := jlt.SphereShape_Create(1)

	jlt.PhysicsSystem_OptimizeBroadPhase(physics_system)

	for !rl.WindowShouldClose() {
		delta_time := rl.GetFrameTime()
		jlt.PhysicsSystem_Update(physics_system, delta_time, 1, job_system)

		if rl.IsMouseButtonDown(.LEFT) {
			rl.UpdateCamera(&camera, .FREE)
		}

		if rl.IsMouseButtonPressed(.RIGHT) {
			is_spawning = !is_spawning
		}

		if is_spawning {
			ball_pos := [3]f32 {
				rand.float32_range(-22, 22),
				rand.float32_range(20, 30),
				rand.float32_range(-22, 22),
			}

			sphere_settings := jlt.BodyCreationSettings_Create3(
				cast(^jlt.Shape)sphere_shape,
				&ball_pos,
				nil,
				.Dynamic,
				OBJECT_LAYER_MOVING,
			)
			defer jlt.BodyCreationSettings_Destroy(sphere_settings)

			ball_id := jlt.BodyInterface_CreateAndAddBody(
				body_interface,
				sphere_settings,
				.Activate,
			)

			append(&balls, ball_id)
		}

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
			ball_loop: for ball_id, index in balls {
				position: [3]f32
				rotation: quaternion128

				jlt.BodyInterface_GetPosition(body_interface, ball_id, &position)
				jlt.BodyInterface_GetRotation(body_interface, ball_id, &rotation)

				//Kill plane
				if position.y <= -50 {
					jlt.BodyInterface_RemoveAndDestroyBody(body_interface, ball_id)
					unordered_remove(&balls, index)
					continue ball_loop
				}

				sphere.transform = rl.QuaternionToMatrix(rotation)

				is_active := jlt.BodyInterface_IsActive(body_interface, ball_id)

				rl.DrawModel(sphere, position, 1, is_active ? rl.RED : rl.GRAY)
				rl.DrawModelWires(sphere, position, 1, rl.BLACK)
			}


		}
		rl.EndMode3D()
		rl.DrawFPS(0, 0)

		rl.DrawText(
			fmt.ctprintf(
				"Press right mouse to %v spawning balls.\nBalls: %d",
				is_spawning ? "stop" : "start",
				len(balls),
			),
			0,
			20,
			20,
			rl.WHITE,
		)
	}
}
