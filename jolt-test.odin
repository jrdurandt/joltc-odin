package jolt

import "base:runtime"
import "core:fmt"
import "core:testing"

@(test)
hello_world :: proc(t: ^testing.T) {
	OBJECT_LAYER_NON_MOVING: ObjectLayer = 0
	OBJECT_LAYER_MOVING: ObjectLayer = 1
	OBJECT_LAYER_NUM :: 2

	BROAD_PHASE_LAYER_NON_MOVING: BroadPhaseLayer = 0
	BROAD_PHASE_LAYER_MOVING: BroadPhaseLayer = 1
	BROAD_PHASE_LAYER_NUM :: 2

	ok := Init()
	defer Shutdown()
	assert(ok, "Failed to init JoltPhysics")

	SetTraceHandler(proc "c" (mssage: cstring) {
		context = runtime.default_context()
		fmt.printfln("Trace: %v", mssage)
	})

	job_system := JobSystemThreadPool_Create(nil)
	defer JobSystem_Destroy(job_system)

	object_layer_pair_filter := ObjectLayerPairFilterTable_Create(OBJECT_LAYER_NUM)
	ObjectLayerPairFilterTable_EnableCollision(
		object_layer_pair_filter,
		OBJECT_LAYER_MOVING,
		OBJECT_LAYER_MOVING,
	)
	ObjectLayerPairFilterTable_EnableCollision(
		object_layer_pair_filter,
		OBJECT_LAYER_MOVING,
		OBJECT_LAYER_NON_MOVING,
	)

	broad_phase_layer_interface_table := BroadPhaseLayerInterfaceTable_Create(
		OBJECT_LAYER_NUM,
		BROAD_PHASE_LAYER_NUM,
	)
	BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
		broad_phase_layer_interface_table,
		OBJECT_LAYER_NON_MOVING,
		BROAD_PHASE_LAYER_NON_MOVING,
	)
	BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
		broad_phase_layer_interface_table,
		OBJECT_LAYER_MOVING,
		BROAD_PHASE_LAYER_MOVING,
	)

	object_vs_broad_phase_layer_filter := ObjectVsBroadPhaseLayerFilterTable_Create(
		broad_phase_layer_interface_table,
		BROAD_PHASE_LAYER_NUM,
		object_layer_pair_filter,
		OBJECT_LAYER_NUM,
	)

	physics_system_settings := PhysicsSystemSettings {
		maxBodies                     = 1024,
		numBodyMutexes                = 0,
		maxBodyPairs                  = 1024,
		maxContactConstraints         = 1024,
		broadPhaseLayerInterface      = broad_phase_layer_interface_table,
		objectLayerPairFilter         = object_layer_pair_filter,
		objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
	}
	physics_system := PhysicsSystem_Create(&physics_system_settings)
	defer PhysicsSystem_Destroy(physics_system)

	my_contact_listener_procs: ContactListener_Procs
	my_contact_listener_procs.OnContactValidate =
	proc "c" (
		userData: rawptr,
		body1: ^Body,
		body2: ^Body,
		baseOffset: ^RVec3,
		collisionResult: ^CollideShapeResult,
	) -> ValidateResult {
		context = runtime.default_context()
		fmt.println("[ContactListener] Contact validate callback")
		return .AcceptAllContactsForThisBodyPair
	}
	my_contact_listener_procs.OnContactAdded =
	proc "c" (
		userData: rawptr,
		body1: ^Body,
		body2: ^Body,
		manifold: ^ContactManifold,
		settings: ^ContactSettings,
	) {
		context = runtime.default_context()
		fmt.println("[ContactListener] A contact was added")
	}
	my_contact_listener_procs.OnContactPersisted =
	proc "c" (
		userData: rawptr,
		body1: ^Body,
		body2: ^Body,
		manifold: ^ContactManifold,
		settings: ^ContactSettings,
	) {
		context = runtime.default_context()
		fmt.println("[ContactListener] A contact was persisted")
	}
	my_contact_listener_procs.OnContactRemoved =
	proc "c" (userData: rawptr, subShapePair: ^SubShapeIDPair) {
		context = runtime.default_context()
		fmt.println("[ContactListener] A contact was removed")
	}

	my_contact_listener := ContactListener_Create(&my_contact_listener_procs)
	defer ContactListener_Destroy(my_contact_listener)

	PhysicsSystem_SetContactListener(physics_system, my_contact_listener)

	my_activation_listener_proc: BodyActivationListener_Procs
	my_activation_listener_proc.OnBodyActivated =
	proc "c" (userData: rawptr, bodyID: BodyID, bodyUserData: u64) {
		context = runtime.default_context()
		fmt.println("[BodyActivationListener] A body got activated")
	}
	my_activation_listener_proc.OnBodyDeactivated =
	proc "c" (userData: rawptr, bodyID: BodyID, bodyUserData: u64) {
		context = runtime.default_context()
		fmt.println("[BodyActivationListener] A body went to sleep")
	}

	my_activation_listener := BodyActivationListener_Create(&my_activation_listener_proc)
	defer BodyActivationListener_Destroy(my_activation_listener)

	PhysicsSystem_SetBodyActivationListener(physics_system, my_activation_listener)

	body_interface := PhysicsSystem_GetBodyInterface(physics_system)

	//--------------------------------------------------------------------------------------------------
	// Hello World
	//--------------------------------------------------------------------------------------------------

	floor_id: BodyID
	{
		box_half_extents := [3]f32{100, 1, 100}
		floor_shape := BoxShape_Create(&box_half_extents, DEFAULT_CONVEX_RADIUS)

		floor_position := [3]f32{0, -1, 0}
		floor_settings := BodyCreationSettings_Create3(
			cast(^Shape)floor_shape,
			&floor_position,
			nil,
			.Static,
			OBJECT_LAYER_NON_MOVING,
		)
		defer BodyCreationSettings_Destroy(floor_settings)

		floor_id = BodyInterface_CreateAndAddBody(body_interface, floor_settings, .DontActivate)
	}
	defer BodyInterface_RemoveAndDestroyBody(body_interface, floor_id)

	sphere_id: BodyID
	{
		sphere_shape := SphereShape_Create(0.5)

		sphere_position := [3]f32{0, 2, 0}
		sphere_settings := BodyCreationSettings_Create3(
			cast(^Shape)sphere_shape,
			&sphere_position,
			nil,
			.Dynamic,
			OBJECT_LAYER_MOVING,
		)
		defer BodyCreationSettings_Destroy(sphere_settings)

		sphere_id = BodyInterface_CreateAndAddBody(body_interface, sphere_settings, .Activate)
	}
	defer BodyInterface_RemoveAndDestroyBody(body_interface, sphere_id)

	sphere_linear_velocity := [3]f32{0, -5, 0}
	BodyInterface_SetLinearVelocity(body_interface, sphere_id, &sphere_linear_velocity)

	test_vel: [3]f32
	BodyInterface_GetLinearVelocity(body_interface, sphere_id, &test_vel)
	testing.expect_value(t, test_vel, sphere_linear_velocity)

	delta_time: f32 = 1.0 / 60.0

	PhysicsSystem_OptimizeBroadPhase(physics_system)

	step := 0
	sphere_active := true
	for sphere_active {
		step += 1

		position: [3]f32
		velocity: [3]f32

		BodyInterface_GetCenterOfMassPosition(body_interface, sphere_id, &position)
		BodyInterface_GetLinearVelocity(body_interface, sphere_id, &velocity)

		fmt.printfln("Step %d: Position = (%v), Velocity = (%v)", step, position, velocity)

		PhysicsSystem_Update(physics_system, delta_time, 1, job_system)

		sphere_active = BodyInterface_IsActive(body_interface, sphere_id) == true

		if step > 100 {
			fmt.eprintf("Failed to reach stable state")
			break
		}
	}
	testing.expect_value(t, sphere_active, false)
}
