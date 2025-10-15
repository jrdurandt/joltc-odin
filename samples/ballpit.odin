package samples

/*
    =========================== Ballpit Sample ================================
    Simple example to test and demo Jolt Physics.
    Uses Raylib to render simple shapes.

    # Controls
    Press and hold right mouse button to move camera and use WASD to move around.
    Press space to toggles spawning balls
    Left click in a ball to select/highlight it
*/

import "core:fmt"
import "core:log"
import "core:math"
import "core:math/rand"
import "core:mem"
import "core:strings"
import "core:thread"
import "core:time"

import jph ".."
import rl "vendor:raylib"

PHYSICS_UPDATES_PER_SECOND :: 1.0 / 50.0
PHYSICS_COLLISION_SUB_STEPS :: 1

OBJECT_LAYER_NON_MOVING: jph.ObjectLayer = 0
OBJECT_LAYER_MOVING: jph.ObjectLayer = 1
OBJECT_LAYER_NUM :: 2

BROAD_PHASE_LAYER_NON_MOVING: jph.BroadPhaseLayer = 0
BROAD_PHASE_LAYER_MOVING: jph.BroadPhaseLayer = 1
BROAD_PHASE_LAYER_NUM :: 2

Ball :: struct {
	body_id:  jph.BodyID,
	selected: bool,
}

Physics :: struct {
	job_system: ^jph.JobSystem,
	system:     ^jph.PhysicsSystem,
	is_running: bool,
	ups:        int,
}

p: ^Physics

main :: proc() {
	context.logger = log.create_console_logger(.Info)
	defer log.destroy_console_logger(context.logger)

	when ODIN_DEBUG {
		context.logger.lowest_level = .Debug

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
	log.debug("Debug enabled")

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
	p = new(Physics)
	defer free(p)

	assert(jph.Init(), "Failed to init JoltPhysics")
	defer jph.Shutdown()

	p.job_system = jph.JobSystemThreadPool_Create(nil)
	defer jph.JobSystem_Destroy(p.job_system)
	assert(p.job_system != nil)

	object_layer_pair_filter := jph.ObjectLayerPairFilterTable_Create(OBJECT_LAYER_NUM)
	jph.ObjectLayerPairFilterTable_EnableCollision(
		object_layer_pair_filter,
		OBJECT_LAYER_MOVING,
		OBJECT_LAYER_MOVING,
	)
	jph.ObjectLayerPairFilterTable_EnableCollision(
		object_layer_pair_filter,
		OBJECT_LAYER_MOVING,
		OBJECT_LAYER_NON_MOVING,
	)

	broad_phase_layer_interface_table := jph.BroadPhaseLayerInterfaceTable_Create(
		OBJECT_LAYER_NUM,
		BROAD_PHASE_LAYER_NUM,
	)
	jph.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
		broad_phase_layer_interface_table,
		OBJECT_LAYER_NON_MOVING,
		BROAD_PHASE_LAYER_NON_MOVING,
	)
	jph.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
		broad_phase_layer_interface_table,
		OBJECT_LAYER_MOVING,
		BROAD_PHASE_LAYER_MOVING,
	)

	object_vs_broad_phase_layer_filter := jph.ObjectVsBroadPhaseLayerFilterTable_Create(
		broad_phase_layer_interface_table,
		BROAD_PHASE_LAYER_NUM,
		object_layer_pair_filter,
		OBJECT_LAYER_NUM,
	)

	physics_system_settings := jph.PhysicsSystemSettings {
		maxBodies                     = 65535,
		numBodyMutexes                = 0,
		maxBodyPairs                  = 65535,
		maxContactConstraints         = 65535,
		broadPhaseLayerInterface      = broad_phase_layer_interface_table,
		objectLayerPairFilter         = object_layer_pair_filter,
		objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
	}
	p.system = jph.PhysicsSystem_Create(&physics_system_settings)
	defer jph.PhysicsSystem_Destroy(p.system)
	assert(p.system != nil)

	body_interface := jph.PhysicsSystem_GetBodyInterface(p.system)

	narrow_phase_query := jph.PhysicsSystem_GetNarrowPhaseQuery(p.system)

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
			body_interface,
			floor_settings,
			.DontActivate,
		)
	}
	defer jph.BodyInterface_RemoveAndDestroyBody(body_interface, floor_id)

	wall_n := build_wall(body_interface, {25, 2.5, 0.5}, {0, 2.5, -25})
	defer jph.BodyInterface_RemoveAndDestroyBody(body_interface, wall_n)

	wall_s := build_wall(body_interface, {25, 2.5, 0.5}, {0, 2.5, 25})
	defer jph.BodyInterface_RemoveAndDestroyBody(body_interface, wall_s)

	wall_e := build_wall(body_interface, {0.5, 2.5, 25.0}, {-25, 2.5, 0})
	defer jph.BodyInterface_RemoveAndDestroyBody(body_interface, wall_e)

	wall_w := build_wall(body_interface, {0.5, 2.5, 25.0}, {25, 2.5, 0})
	defer jph.BodyInterface_RemoveAndDestroyBody(body_interface, wall_w)

	// balls: [dynamic]jph.BodyID
	balls: map[jph.BodyID]Ball
	defer {
		for _, v in balls {
			jph.BodyInterface_RemoveAndDestroyBody(body_interface, v.body_id)
		}
		delete(balls)
	}

	sphere_shape := jph.SphereShape_Create(1)

	jph.PhysicsSystem_OptimizeBroadPhase(p.system)

	p.is_running = true
	thread.create_and_start(physics_thread, self_cleanup = true)
	defer p.is_running = false

	removed_balls: [dynamic]jph.BodyID
	defer delete(removed_balls)

	ray: rl.Ray
	for !rl.WindowShouldClose() {
		if rl.IsMouseButtonDown(.RIGHT) {
			rl.UpdateCamera(&camera, .FREE)
		} else if rl.IsKeyPressed(.SPACE) {
			is_spawning = !is_spawning
		}

		//Testing ray cast
		if rl.IsMouseButtonPressed(.LEFT) {
			ray = rl.GetScreenToWorldRay(rl.GetMousePosition(), camera)
			direction := ray.direction * 100
			result: jph.RayCastResult
			if (jph.NarrowPhaseQuery_CastRay(
					   narrow_phase_query,
					   &ray.position,
					   &direction,
					   &result,
					   nil,
					   nil,
					   nil,
				   )) {
				ball, found := &balls[result.bodyID]
				if found {
					ball.selected = !ball.selected
				}
			}
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
				body_interface,
				sphere_settings,
				.Activate,
			)

			balls[ball_id] = Ball {
				body_id = ball_id,
			}
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
			ball_loop: for key, ball in balls {
				ball_id := ball.body_id

				position: [3]f32
				rotation: quaternion128

				jph.BodyInterface_GetPosition(body_interface, ball_id, &position)
				jph.BodyInterface_GetRotation(body_interface, ball_id, &rotation)

				//Kill plane
				if position.y <= -50 {
					append(&removed_balls, key)
					continue ball_loop
				}

				sphere.transform = rl.QuaternionToMatrix(rotation)

				is_active := jph.BodyInterface_IsActive(body_interface, ball_id)
				is_selected := ball.selected

				rl.DrawModel(sphere, position, 1, is_active ? rl.RED : rl.GRAY)
				rl.DrawModelWires(sphere, position, 1, is_selected ? rl.WHITE : rl.BLACK)
			}

			for ball_id in removed_balls {
				jph.BodyInterface_RemoveAndDestroyBody(body_interface, ball_id)
				delete_key(&balls, ball_id)
			}
			clear(&removed_balls)
		}
		rl.EndMode3D()
		rl.DrawFPS(0, 0)
		rl.DrawText(fmt.ctprintf("%2d UPS", p.ups), 0, 20, 20, rl.GREEN)

		rl.DrawText(
			fmt.ctprintf(
				"Hold RIGHT mouse button and use WASD for camera control.\nLEFT click to select balls.\nPress space to %v spawning balls.\nBalls: %d",
				is_spawning ? "stop" : "start",
				len(balls),
			),
			0,
			40,
			20,
			rl.WHITE,
		)
	}
}

@(private)
build_wall :: proc(
	body_interface: ^jph.BodyInterface,
	size: [3]f32,
	position: [3]f32,
) -> jph.BodyID {
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

	return jph.BodyInterface_CreateAndAddBody(body_interface, floor_settings, .DontActivate)
}

@(private)
physics_thread :: proc() {
	ups_buff: [30]f32
	last_time := time.now()
	cnt: int
	for p.is_running {
		now_time := time.now()
		delta := f32(time.duration_seconds(time.diff(last_time, now_time)))

		if delta > PHYSICS_UPDATES_PER_SECOND {
			last_time = now_time

			err := jph.PhysicsSystem_Update(
				p.system,
				delta,
				PHYSICS_COLLISION_SUB_STEPS,
				p.job_system,
			)

			if err != .None {
				p.is_running = false
				log.errorf("Error updating physics system: %v", err)
			}

			i := cnt % 30
			ups_buff[i] = delta

			delta_total: f32 = 0
			for d in ups_buff {
				delta_total += d
			}
			avg_ups := delta_total / 30
			p.ups = int(math.ceil(1.0 / avg_ups))

			cnt += 1
		}
	}
}
