package joltc_examples

import "base:runtime"
import "core:fmt"
import "core:log"
import "core:math"
import "core:thread"
import "core:time"

import jph ".."
import rl "vendor:raylib"

PHYSICS_UPDATED_PER_SECOND :: 1.0 / 60.0
PHYSICS_COLLISION_SUB_STEPS :: 1

OBJECT_LAYER_NON_MOVING: jph.ObjectLayer = 0
OBJECT_LAYER_MOVING: jph.ObjectLayer = 1
OBJECT_LAYER_NUM :: 2

BROAD_PHASE_LAYER_NON_MOVING: jph.BroadPhaseLayer = 0
BROAD_PHASE_LAYER_MOVING: jph.BroadPhaseLayer = 1
BROAD_PHASE_LAYER_NUM :: 2

Physics_Context :: struct {
	job_system:     ^jph.JobSystem,
	system:         ^jph.PhysicsSystem,
	body_interface: ^jph.BodyInterface,
}

physics: ^Physics_Context

init_physics :: proc() {
	assert(jph.Init(), "Failed to init Jolt Physics")

	physics = new(Physics_Context)

	physics.job_system = jph.JobSystemThreadPool_Create(nil)
	assert(physics.job_system != nil, "Failed to create physics job system")

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

	physics.system = jph.PhysicsSystem_Create(&physics_system_settings)
	assert(physics.system != nil, "Failed to create Jolt physics system")

	physics.body_interface = jph.PhysicsSystem_GetBodyInterface(physics.system)
}

destroy_physics :: proc() {
	jph.PhysicsSystem_Destroy(physics.system)
	jph.JobSystem_Destroy(physics.job_system)
	free(physics)
	jph.Shutdown()
}

update_physics :: proc() {
	err := jph.PhysicsSystem_Update(
		physics.system,
		PHYSICS_UPDATED_PER_SECOND,
		PHYSICS_COLLISION_SUB_STEPS,
		physics.job_system,
	)
	fmt.assertf(err == .None, "Failed to update physics: %s", err)
}
