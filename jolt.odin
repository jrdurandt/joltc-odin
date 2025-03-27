package jolt

DEFAULT_COLLISION_TOLERANCE :: 0.0001000000000000
DEFAULT_PENETRATION_TOLERANCE :: 0.0001000000000000
DEFAULT_CONVEX_RADIUS :: 0.0500000000000000
CAPSULE_PROJECTION_SLOP :: 0.0200000000000000
MAX_PHYSICS_JOBS :: 2048
MAX_PHYSICS_BARRIERS :: 2048

BodyID :: u32
SubShapeID :: u32
ObjectLayer :: u16
BroadPhaseLayer :: u8
CollisionGroupID :: u32
CollisionSubGroupID :: u32
CharacterID :: u32
Vec3 :: [3]f32
Vec4 :: [4]f32
Quat :: quaternion128
Plane :: struct {
	normal:   Vec3,
	distance: f32,
}
Matrix4x4 :: [4][4]f32
RVec3 :: Vec3
RMatrix4x4 :: Matrix4x4
Color :: u32
AABox :: struct {
	min: Vec3,
	max: Vec3,
}
Triangle :: struct {
	v1:            Vec3,
	v2:            Vec3,
	v3:            Vec3,
	materialIndex: u32,
}
IndexedTriangleNoMaterial :: struct {
	i1: u32,
	i2: u32,
	i3: u32,
}
IndexedTriangle :: struct {
	i1:            u32,
	i2:            u32,
	i3:            u32,
	materialIndex: u32,
	userData:      u32,
}
MassProperties :: struct {
	mass:    f32,
	inertia: Matrix4x4,
}
CollideSettingsBase :: struct {
	activeEdgeMode:              ActiveEdgeMode,
	collectFacesMode:            CollectFacesMode,
	collisionTolerance:          f32,
	penetrationTolerance:        f32,
	activeEdgeMovementDirection: Vec3,
}
CollideShapeSettings :: struct {
	base:                  CollideSettingsBase,
	maxSeparationDistance: f32,
	backFaceMode:          BackFaceMode,
}
ShapeCastSettings :: struct {
	base:                            CollideSettingsBase,
	backFaceModeTriangles:           BackFaceMode,
	backFaceModeConvex:              BackFaceMode,
	useShrunkenShapeAndConvexRadius: b8,
	returnDeepestPoint:              b8,
}
RayCastSettings :: struct {
	backFaceModeTriangles: BackFaceMode,
	backFaceModeConvex:    BackFaceMode,
	treatConvexAsSolid:    b8,
}
SpringSettings :: struct {
	mode:                 SpringMode,
	frequencyOrStiffness: f32,
	damping:              f32,
}
MotorSettings :: struct {
	springSettings: SpringSettings,
	minForceLimit:  f32,
	maxForceLimit:  f32,
	minTorqueLimit: f32,
	maxTorqueLimit: f32,
}
SubShapeIDPair :: struct {
	Body1ID:     BodyID,
	subShapeID1: SubShapeID,
	Body2ID:     BodyID,
	subShapeID2: SubShapeID,
}
BroadPhaseCastResult :: struct {
	bodyID:   BodyID,
	fraction: f32,
}
RayCastResult :: struct {
	bodyID:      BodyID,
	fraction:    f32,
	subShapeID2: SubShapeID,
}
CollidePointResult :: struct {
	bodyID:      BodyID,
	subShapeID2: SubShapeID,
}
CollideShapeResult :: struct {
	contactPointOn1:  Vec3,
	contactPointOn2:  Vec3,
	penetrationAxis:  Vec3,
	penetrationDepth: f32,
	subShapeID1:      SubShapeID,
	subShapeID2:      SubShapeID,
	bodyID2:          BodyID,
	shape1FaceCount:  u32,
	shape1Faces:      [^]Vec3,
	shape2FaceCount:  u32,
	shape2Faces:      [^]Vec3,
}
ShapeCastResult :: struct {
	contactPointOn1:  Vec3,
	contactPointOn2:  Vec3,
	penetrationAxis:  Vec3,
	penetrationDepth: f32,
	subShapeID1:      SubShapeID,
	subShapeID2:      SubShapeID,
	bodyID2:          BodyID,
	fraction:         f32,
	isBackFaceHit:    b8,
}
DrawSettings :: struct {
	drawGetSupportFunction:        b8,
	drawSupportDirection:          b8,
	drawGetSupportingFace:         b8,
	drawShape:                     b8,
	drawShapeWireframe:            b8,
	drawShapeColor:                BodyManager_ShapeColor,
	drawBoundingBox:               b8,
	drawCenterOfMassTransform:     b8,
	drawWorldTransform:            b8,
	drawVelocity:                  b8,
	drawMassAndInertia:            b8,
	drawSleepStats:                b8,
	drawSoftBodyVertices:          b8,
	drawSoftBodyVertexVelocities:  b8,
	drawSoftBodyEdgeConstraints:   b8,
	drawSoftBodyBendConstraints:   b8,
	drawSoftBodyVolumeConstraints: b8,
	drawSoftBodySkinConstraints:   b8,
	drawSoftBodyLRAConstraints:    b8,
	drawSoftBodyPredictedBounds:   b8,
	drawSoftBodyConstraintColor:   SoftBodyConstraintColor,
}
SupportingFace :: struct {
	count:    u32,
	vertices: [32]Vec3,
}
CastRayResultCallback :: #type proc "c" (context_p: rawptr, result: ^RayCastResult)
RayCastBodyResultCallback :: #type proc "c" (context_p: rawptr, result: ^BroadPhaseCastResult)
CollideShapeBodyResultCallback :: #type proc "c" (context_p: rawptr, result: BodyID)
CollidePointResultCallback :: #type proc "c" (context_p: rawptr, result: ^CollidePointResult)
CollideShapeResultCallback :: #type proc "c" (context_p: rawptr, result: ^CollideShapeResult)
CastShapeResultCallback :: #type proc "c" (context_p: rawptr, result: ^ShapeCastResult)
CastRayCollectorCallback :: #type proc "c" (context_p: rawptr, result: ^RayCastResult) -> f32
RayCastBodyCollectorCallback :: #type proc "c" (
	context_p: rawptr,
	result: ^BroadPhaseCastResult,
) -> f32
CollideShapeBodyCollectorCallback :: #type proc "c" (context_p: rawptr, result: BodyID) -> f32
CollidePointCollectorCallback :: #type proc "c" (
	context_p: rawptr,
	result: ^CollidePointResult,
) -> f32
CollideShapeCollectorCallback :: #type proc "c" (
	context_p: rawptr,
	result: ^CollideShapeResult,
) -> f32
CastShapeCollectorCallback :: #type proc "c" (context_p: rawptr, result: ^ShapeCastResult) -> f32
CollisionEstimationResultImpulse :: struct {
	contactImpulse:   f32,
	frictionImpulse1: f32,
	frictionImpulse2: f32,
}
CollisionEstimationResult :: struct {
	linearVelocity1:  Vec3,
	angularVelocity1: Vec3,
	linearVelocity2:  Vec3,
	angularVelocity2: Vec3,
	tangent1:         Vec3,
	tangent2:         Vec3,
	impulseCount:     u32,
	impulses:         [^]CollisionEstimationResultImpulse,
}
ConstraintSettings :: struct {
	enabled:                  b8,
	constraintPriority:       u32,
	numVelocityStepsOverride: u32,
	numPositionStepsOverride: u32,
	drawConstraintSize:       f32,
	userData:                 u64,
}
FixedConstraintSettings :: struct {
	base:            ConstraintSettings,
	space:           ConstraintSpace,
	autoDetectPoint: b8,
	point1:          RVec3,
	axisX1:          Vec3,
	axisY1:          Vec3,
	point2:          RVec3,
	axisX2:          Vec3,
	axisY2:          Vec3,
}
DistanceConstraintSettings :: struct {
	base:                 ConstraintSettings,
	space:                ConstraintSpace,
	point1:               RVec3,
	point2:               RVec3,
	minDistance:          f32,
	maxDistance:          f32,
	limitsSpringSettings: SpringSettings,
}
PointConstraintSettings :: struct {
	base:   ConstraintSettings,
	space:  ConstraintSpace,
	point1: RVec3,
	point2: RVec3,
}
HingeConstraintSettings :: struct {
	base:                 ConstraintSettings,
	space:                ConstraintSpace,
	point1:               RVec3,
	hingeAxis1:           Vec3,
	normalAxis1:          Vec3,
	point2:               RVec3,
	hingeAxis2:           Vec3,
	normalAxis2:          Vec3,
	limitsMin:            f32,
	limitsMax:            f32,
	limitsSpringSettings: SpringSettings,
	maxFrictionTorque:    f32,
	motorSettings:        MotorSettings,
}
SliderConstraintSettings :: struct {
	base:                 ConstraintSettings,
	space:                ConstraintSpace,
	autoDetectPoint:      b8,
	point1:               RVec3,
	sliderAxis1:          Vec3,
	normalAxis1:          Vec3,
	point2:               RVec3,
	sliderAxis2:          Vec3,
	normalAxis2:          Vec3,
	limitsMin:            f32,
	limitsMax:            f32,
	limitsSpringSettings: SpringSettings,
	maxFrictionForce:     f32,
	motorSettings:        MotorSettings,
}
ConeConstraintSettings :: struct {
	base:          ConstraintSettings,
	space:         ConstraintSpace,
	point1:        RVec3,
	twistAxis1:    Vec3,
	point2:        RVec3,
	twistAxis2:    Vec3,
	halfConeAngle: f32,
}
SwingTwistConstraintSettings :: struct {
	base:                ConstraintSettings,
	space:               ConstraintSpace,
	position1:           RVec3,
	twistAxis1:          Vec3,
	planeAxis1:          Vec3,
	position2:           RVec3,
	twistAxis2:          Vec3,
	planeAxis2:          Vec3,
	swingType:           SwingType,
	normalHalfConeAngle: f32,
	planeHalfConeAngle:  f32,
	twistMinAngle:       f32,
	twistMaxAngle:       f32,
	maxFrictionTorque:   f32,
	swingMotorSettings:  MotorSettings,
	twistMotorSettings:  MotorSettings,
}
SixDOFConstraintSettings :: struct {
	base:                 ConstraintSettings,
	space:                ConstraintSpace,
	position1:            RVec3,
	axisX1:               Vec3,
	axisY1:               Vec3,
	position2:            RVec3,
	axisX2:               Vec3,
	axisY2:               Vec3,
	maxFriction:          [6]f32,
	swingType:            SwingType,
	limitMin:             [6]f32,
	limitMax:             [6]f32,
	limitsSpringSettings: [3]SpringSettings,
	motorSettings:        [6]MotorSettings,
}
GearConstraintSettings :: struct {
	base:       ConstraintSettings,
	space:      ConstraintSpace,
	hingeAxis1: Vec3,
	hingeAxis2: Vec3,
	ratio:      f32,
}
BodyLockInterface :: struct #packed {}
SharedMutex :: struct #packed {}
Body :: struct #packed {}
BodyLockRead :: struct {
	lockInterface: ^BodyLockInterface,
	mutex:         ^SharedMutex,
	body:          ^Body,
}
BodyLockWrite :: struct {
	lockInterface: ^BodyLockInterface,
	mutex:         ^SharedMutex,
	body:          ^Body,
}
ExtendedUpdateSettings :: struct {
	stickToFloorStepDown:             Vec3,
	walkStairsStepUp:                 Vec3,
	walkStairsMinStepForward:         f32,
	walkStairsStepForwardTest:        f32,
	walkStairsCosAngleForwardContact: f32,
	walkStairsStepDownExtra:          Vec3,
}
Shape :: struct #packed {}
CharacterBaseSettings :: struct {
	up:                          Vec3,
	supportingVolume:            Plane,
	maxSlopeAngle:               f32,
	enhancedInternalEdgeRemoval: b8,
	shape:                       ^Shape,
}
CharacterSettings :: struct {
	base:          CharacterBaseSettings,
	layer:         ObjectLayer,
	mass:          f32,
	friction:      f32,
	gravityFactor: f32,
	allowedDOFs:   AllowedDOFs,
}
CharacterVirtualSettings :: struct {
	base:                      CharacterBaseSettings,
	ID:                        CharacterID,
	mass:                      f32,
	maxStrength:               f32,
	shapeOffset:               Vec3,
	backFaceMode:              BackFaceMode,
	predictiveContactDistance: f32,
	maxCollisionIterations:    u32,
	maxConstraintIterations:   u32,
	minTimeRemaining:          f32,
	collisionTolerance:        f32,
	characterPadding:          f32,
	maxNumHits:                u32,
	hitReductionCosMaxAngle:   f32,
	penetrationRecoverySpeed:  f32,
	innerBodyShape:            ^Shape,
	innerBodyIDOverride:       BodyID,
	innerBodyLayer:            ObjectLayer,
}
CharacterContactSettings :: struct {
	canPushCharacter:   b8,
	canReceiveImpulses: b8,
}
CharacterVirtual :: struct #packed {}
PhysicsMaterial :: struct #packed {}
CharacterVirtualContact :: struct {
	hash:             u64,
	bodyB:            BodyID,
	characterIDB:     CharacterID,
	subShapeIDB:      SubShapeID,
	position:         RVec3,
	linearVelocity:   Vec3,
	contactNormal:    Vec3,
	surfaceNormal:    Vec3,
	distance:         f32,
	fraction:         f32,
	motionTypeB:      MotionType,
	isSensorB:        b8,
	characterB:       ^CharacterVirtual,
	userData:         u64,
	material:         ^PhysicsMaterial,
	hadCollision:     b8,
	wasDiscarded:     b8,
	canPushCharacter: b8,
}
TraceFunc :: #type proc "c" (mssage: cstring)
AssertFailureFunc :: #type proc "c" (
	expression: cstring,
	mssage: cstring,
	file: cstring,
	line: u32,
) -> b8
JobFunction :: #type proc "c" (arg: rawptr)
QueueJobCallback :: #type proc "c" (context_p: rawptr, job: ^JobFunction, arg: rawptr)
QueueJobsCallback :: #type proc "c" (
	context_p: rawptr,
	job: ^JobFunction,
	args: [^]rawptr,
	count: u32,
)
JobSystemThreadPoolConfig :: struct {
	maxJobs:     u32,
	maxBarriers: u32,
	numThreads:  i32,
}
JobSystemConfig :: struct {
	context_m:      rawptr,
	queueJob:       ^QueueJobCallback,
	queueJobs:      [^]QueueJobsCallback,
	maxConcurrency: u32,
	maxBarriers:    u32,
}
BroadPhaseLayerInterface :: struct #packed {}
ObjectLayerPairFilter :: struct #packed {}
ObjectVsBroadPhaseLayerFilter :: struct #packed {}
PhysicsSystemSettings :: struct {
	maxBodies:                     u32,
	numBodyMutexes:                u32,
	maxBodyPairs:                  u32,
	maxContactConstraints:         u32,
	_padding:                      u32,
	broadPhaseLayerInterface:      ^BroadPhaseLayerInterface,
	objectLayerPairFilter:         ^ObjectLayerPairFilter,
	objectVsBroadPhaseLayerFilter: ^ObjectVsBroadPhaseLayerFilter,
}
PhysicsSettings :: struct {
	maxInFlightBodyPairs:                 i32,
	stepListenersBatchSize:               i32,
	stepListenerBatchesPerJob:            i32,
	baumgarte:                            f32,
	speculativeContactDistance:           f32,
	penetrationSlop:                      f32,
	linearCastThreshold:                  f32,
	linearCastMaxPenetration:             f32,
	manifoldTolerance:                    f32,
	maxPenetrationDistance:               f32,
	bodyPairCacheMaxDeltaPositionSq:      f32,
	bodyPairCacheCosMaxDeltaRotationDiv2: f32,
	contactNormalCosMaxDeltaRotation:     f32,
	contactPointPreserveLambdaMaxDistSq:  f32,
	numVelocitySteps:                     u32,
	numPositionSteps:                     u32,
	minVelocityForRestitution:            f32,
	timeBeforeSleep:                      f32,
	pointVelocitySleepThreshold:          f32,
	deterministicSimulation:              b8,
	constraintWarmStart:                  b8,
	useBodyPairContactCache:              b8,
	useManifoldReduction:                 b8,
	useLargeIslandSplitter:               b8,
	allowSleeping:                        b8,
	checkActiveEdges:                     b8,
}
ShouldCollide_func_ptr_anon_0 :: #type proc "c" (userData: rawptr, layer: BroadPhaseLayer) -> b8
BroadPhaseLayerFilter_Procs :: struct {
	ShouldCollide: ShouldCollide_func_ptr_anon_0,
}
ShouldCollide_func_ptr_anon_1 :: #type proc "c" (userData: rawptr, layer: ObjectLayer) -> b8
ObjectLayerFilter_Procs :: struct {
	ShouldCollide: ShouldCollide_func_ptr_anon_1,
}
ShouldCollide_func_ptr_anon_2 :: #type proc "c" (userData: rawptr, bodyID: BodyID) -> b8
ShouldCollideLocked_func_ptr_anon_3 :: #type proc "c" (userData: rawptr, bodyID: ^Body) -> b8
BodyFilter_Procs :: struct {
	ShouldCollide:       ShouldCollide_func_ptr_anon_2,
	ShouldCollideLocked: ShouldCollideLocked_func_ptr_anon_3,
}
ShouldCollide_func_ptr_anon_4 :: #type proc "c" (
	userData: rawptr,
	shape2: ^Shape,
	subShapeIDOfShape2: ^SubShapeID,
) -> b8
ShouldCollide2_func_ptr_anon_5 :: #type proc "c" (
	userData: rawptr,
	shape1: ^Shape,
	subShapeIDOfShape1: ^SubShapeID,
	shape2: ^Shape,
	subShapeIDOfShape2: ^SubShapeID,
) -> b8
ShapeFilter_Procs :: struct {
	ShouldCollide:  ShouldCollide_func_ptr_anon_4,
	ShouldCollide2: ShouldCollide2_func_ptr_anon_5,
}
ShouldCollide_func_ptr_anon_6 :: #type proc "c" (
	userData: rawptr,
	body1: ^Body,
	shape1: ^Shape,
	subShapeIDOfShape1: ^SubShapeID,
	body2: ^Body,
	shape2: ^Shape,
	subShapeIDOfShape2: ^SubShapeID,
) -> b8
SimShapeFilter_Procs :: struct {
	ShouldCollide: ShouldCollide_func_ptr_anon_6,
}
OnContactValidate_func_ptr_anon_7 :: #type proc "c" (
	userData: rawptr,
	body1: ^Body,
	body2: ^Body,
	baseOffset: ^RVec3,
	collisionResult: ^CollideShapeResult,
) -> ValidateResult
ContactManifold :: struct #packed {}
ContactSettings :: struct #packed {}
OnContactAdded_func_ptr_anon_8 :: #type proc "c" (
	userData: rawptr,
	body1: ^Body,
	body2: ^Body,
	manifold: ^ContactManifold,
	settings: [^]ContactSettings,
)
OnContactPersisted_func_ptr_anon_9 :: #type proc "c" (
	userData: rawptr,
	body1: ^Body,
	body2: ^Body,
	manifold: ^ContactManifold,
	settings: [^]ContactSettings,
)
OnContactRemoved_func_ptr_anon_10 :: #type proc "c" (
	userData: rawptr,
	subShapePair: ^SubShapeIDPair,
)
ContactListener_Procs :: struct {
	OnContactValidate:  OnContactValidate_func_ptr_anon_7,
	OnContactAdded:     OnContactAdded_func_ptr_anon_8,
	OnContactPersisted: OnContactPersisted_func_ptr_anon_9,
	OnContactRemoved:   OnContactRemoved_func_ptr_anon_10,
}
OnBodyActivated_func_ptr_anon_11 :: #type proc "c" (
	userData: rawptr,
	bodyID: BodyID,
	bodyUserData: u64,
)
OnBodyDeactivated_func_ptr_anon_12 :: #type proc "c" (
	userData: rawptr,
	bodyID: BodyID,
	bodyUserData: u64,
)
BodyActivationListener_Procs :: struct {
	OnBodyActivated:   OnBodyActivated_func_ptr_anon_11,
	OnBodyDeactivated: OnBodyDeactivated_func_ptr_anon_12,
}
ShouldDraw_func_ptr_anon_13 :: #type proc "c" (userData: rawptr, body: ^Body) -> b8
BodyDrawFilter_Procs :: struct {
	ShouldDraw: ShouldDraw_func_ptr_anon_13,
}
OnAdjustBodyVelocity_func_ptr_anon_14 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	body2: ^Body,
	ioLinearVelocity: ^Vec3,
	ioAngularVelocity: ^Vec3,
)
OnContactValidate_func_ptr_anon_15 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	bodyID2: BodyID,
	subShapeID2: SubShapeID,
) -> b8
OnCharacterContactValidate_func_ptr_anon_16 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	otherCharacter: ^CharacterVirtual,
	subShapeID2: SubShapeID,
) -> b8
OnContactAdded_func_ptr_anon_17 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	bodyID2: BodyID,
	subShapeID2: SubShapeID,
	contactPosition: ^RVec3,
	contactNormal: ^Vec3,
	ioSettings: [^]CharacterContactSettings,
)
OnContactPersisted_func_ptr_anon_18 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	bodyID2: BodyID,
	subShapeID2: SubShapeID,
	contactPosition: ^RVec3,
	contactNormal: ^Vec3,
	ioSettings: [^]CharacterContactSettings,
)
OnContactRemoved_func_ptr_anon_19 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	bodyID2: BodyID,
	subShapeID2: SubShapeID,
)
OnCharacterContactAdded_func_ptr_anon_20 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	otherCharacter: ^CharacterVirtual,
	subShapeID2: SubShapeID,
	contactPosition: ^RVec3,
	contactNormal: ^Vec3,
	ioSettings: [^]CharacterContactSettings,
)
OnCharacterContactPersisted_func_ptr_anon_21 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	otherCharacter: ^CharacterVirtual,
	subShapeID2: SubShapeID,
	contactPosition: ^RVec3,
	contactNormal: ^Vec3,
	ioSettings: [^]CharacterContactSettings,
)
OnCharacterContactRemoved_func_ptr_anon_22 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	otherCharacterID: CharacterID,
	subShapeID2: SubShapeID,
)
OnContactSolve_func_ptr_anon_23 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	bodyID2: BodyID,
	subShapeID2: SubShapeID,
	contactPosition: ^RVec3,
	contactNormal: ^Vec3,
	contactVelocity: ^Vec3,
	contactMaterial: ^PhysicsMaterial,
	characterVelocity: ^Vec3,
	newCharacterVelocity: ^Vec3,
)
OnCharacterContactSolve_func_ptr_anon_24 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	otherCharacter: ^CharacterVirtual,
	subShapeID2: SubShapeID,
	contactPosition: ^RVec3,
	contactNormal: ^Vec3,
	contactVelocity: ^Vec3,
	contactMaterial: ^PhysicsMaterial,
	characterVelocity: ^Vec3,
	newCharacterVelocity: ^Vec3,
)
CharacterContactListener_Procs :: struct {
	OnAdjustBodyVelocity:        OnAdjustBodyVelocity_func_ptr_anon_14,
	OnContactValidate:           OnContactValidate_func_ptr_anon_15,
	OnCharacterContactValidate:  OnCharacterContactValidate_func_ptr_anon_16,
	OnContactAdded:              OnContactAdded_func_ptr_anon_17,
	OnContactPersisted:          OnContactPersisted_func_ptr_anon_18,
	OnContactRemoved:            OnContactRemoved_func_ptr_anon_19,
	OnCharacterContactAdded:     OnCharacterContactAdded_func_ptr_anon_20,
	OnCharacterContactPersisted: OnCharacterContactPersisted_func_ptr_anon_21,
	OnCharacterContactRemoved:   OnCharacterContactRemoved_func_ptr_anon_22,
	OnContactSolve:              OnContactSolve_func_ptr_anon_23,
	OnCharacterContactSolve:     OnCharacterContactSolve_func_ptr_anon_24,
}
CollideCharacter_func_ptr_anon_25 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	centerOfMassTransform: ^RMatrix4x4,
	collideShapeSettings: [^]CollideShapeSettings,
	baseOffset: ^RVec3,
)
CastCharacter_func_ptr_anon_26 :: #type proc "c" (
	userData: rawptr,
	character: ^CharacterVirtual,
	centerOfMassTransform: ^RMatrix4x4,
	direction: ^Vec3,
	shapeCastSettings: [^]ShapeCastSettings,
	baseOffset: ^RVec3,
)
CharacterVsCharacterCollision_Procs :: struct {
	CollideCharacter: CollideCharacter_func_ptr_anon_25,
	CastCharacter:    CastCharacter_func_ptr_anon_26,
}
DrawLine_func_ptr_anon_27 :: #type proc "c" (
	userData: rawptr,
	from: ^RVec3,
	to: ^RVec3,
	color: Color,
)
DrawTriangle_func_ptr_anon_28 :: #type proc "c" (
	userData: rawptr,
	v1: ^RVec3,
	v2: ^RVec3,
	v3: ^RVec3,
	color: Color,
	castShadow: DebugRenderer_CastShadow,
)
DrawText3D_func_ptr_anon_29 :: #type proc "c" (
	userData: rawptr,
	position: ^RVec3,
	str: cstring,
	color: Color,
	height: f32,
)
DebugRenderer_Procs :: struct {
	DrawLine:     DrawLine_func_ptr_anon_27,
	DrawTriangle: DrawTriangle_func_ptr_anon_28,
	DrawText3D:   DrawText3D_func_ptr_anon_29,
}
SkeletonJoint :: struct {
	name:             cstring,
	parentName:       cstring,
	parentJointIndex: i32,
}
BroadPhaseLayerFilter :: struct #packed {}
ObjectLayerFilter :: struct #packed {}
BodyFilter :: struct #packed {}
ShapeFilter :: struct #packed {}
SimShapeFilter :: struct #packed {}
PhysicsSystem :: struct #packed {}
ShapeSettings :: struct #packed {}
ConvexShapeSettings :: struct #packed {}
SphereShapeSettings :: struct #packed {}
BoxShapeSettings :: struct #packed {}
PlaneShapeSettings :: struct #packed {}
TriangleShapeSettings :: struct #packed {}
CapsuleShapeSettings :: struct #packed {}
TaperedCapsuleShapeSettings :: struct #packed {}
CylinderShapeSettings :: struct #packed {}
TaperedCylinderShapeSettings :: struct #packed {}
ConvexHullShapeSettings :: struct #packed {}
CompoundShapeSettings :: struct #packed {}
StaticCompoundShapeSettings :: struct #packed {}
MutableCompoundShapeSettings :: struct #packed {}
MeshShapeSettings :: struct #packed {}
HeightFieldShapeSettings :: struct #packed {}
RotatedTranslatedShapeSettings :: struct #packed {}
ScaledShapeSettings :: struct #packed {}
OffsetCenterOfMassShapeSettings :: struct #packed {}
EmptyShapeSettings :: struct #packed {}
ConvexShape :: struct #packed {}
SphereShape :: struct #packed {}
BoxShape :: struct #packed {}
PlaneShape :: struct #packed {}
CapsuleShape :: struct #packed {}
CylinderShape :: struct #packed {}
TaperedCylinderShape :: struct #packed {}
TriangleShape :: struct #packed {}
TaperedCapsuleShape :: struct #packed {}
ConvexHullShape :: struct #packed {}
CompoundShape :: struct #packed {}
StaticCompoundShape :: struct #packed {}
MutableCompoundShape :: struct #packed {}
MeshShape :: struct #packed {}
HeightFieldShape :: struct #packed {}
DecoratedShape :: struct #packed {}
RotatedTranslatedShape :: struct #packed {}
ScaledShape :: struct #packed {}
OffsetCenterOfMassShape :: struct #packed {}
EmptyShape :: struct #packed {}
BodyCreationSettings :: struct #packed {}
SoftBodyCreationSettings :: struct #packed {}
BodyInterface :: struct #packed {}
BroadPhaseQuery :: struct #packed {}
NarrowPhaseQuery :: struct #packed {}
MotionProperties :: struct #packed {}
ContactListener :: struct #packed {}
BodyActivationListener :: struct #packed {}
BodyDrawFilter :: struct #packed {}
DebugRenderer :: struct #packed {}
Constraint :: struct #packed {}
TwoBodyConstraint :: struct #packed {}
FixedConstraint :: struct #packed {}
DistanceConstraint :: struct #packed {}
PointConstraint :: struct #packed {}
HingeConstraint :: struct #packed {}
SliderConstraint :: struct #packed {}
ConeConstraint :: struct #packed {}
SwingTwistConstraint :: struct #packed {}
SixDOFConstraint :: struct #packed {}
GearConstraint :: struct #packed {}
CharacterBase :: struct #packed {}
Character :: struct #packed {}
CharacterContactListener :: struct #packed {}
CharacterVsCharacterCollision :: struct #packed {}
Skeleton :: struct #packed {}
RagdollSettings :: struct #packed {}
Ragdoll :: struct #packed {}
BodyLockMultiRead :: struct #packed {}
BodyLockMultiWrite :: struct #packed {}
JobSystem :: struct #packed {}

@(default_calling_convention = "c")
foreign jolt_runic {
	@(link_name = "JPH_JobSystemThreadPool_Create")
	JobSystemThreadPool_Create :: proc(config: ^JobSystemThreadPoolConfig) -> ^JobSystem ---

	@(link_name = "JPH_JobSystemCallback_Create")
	JobSystemCallback_Create :: proc(config: ^JobSystemConfig) -> ^JobSystem ---

	@(link_name = "JPH_JobSystem_Destroy")
	JobSystem_Destroy :: proc(jobSystem: ^JobSystem) ---

	@(link_name = "JPH_Init")
	Init :: proc() -> b8 ---

	@(link_name = "JPH_Shutdown")
	Shutdown :: proc() ---

	@(link_name = "JPH_SetTraceHandler")
	SetTraceHandler :: proc(handler: TraceFunc) ---

	@(link_name = "JPH_SetAssertFailureHandler")
	SetAssertFailureHandler :: proc(handler: AssertFailureFunc) ---

	@(link_name = "JPH_CollideShapeResult_FreeMembers")
	CollideShapeResult_FreeMembers :: proc(result: ^CollideShapeResult) ---

	@(link_name = "JPH_CollisionEstimationResult_FreeMembers")
	CollisionEstimationResult_FreeMembers :: proc(result: ^CollisionEstimationResult) ---

	@(link_name = "JPH_BroadPhaseLayerInterfaceMask_Create")
	BroadPhaseLayerInterfaceMask_Create :: proc(numBroadPhaseLayers: u32) -> ^BroadPhaseLayerInterface ---

	@(link_name = "JPH_BroadPhaseLayerInterfaceMask_ConfigureLayer")
	BroadPhaseLayerInterfaceMask_ConfigureLayer :: proc(bpInterface: ^BroadPhaseLayerInterface, broadPhaseLayer: BroadPhaseLayer, groupsToInclude: u32, groupsToExclude: u32) ---

	@(link_name = "JPH_BroadPhaseLayerInterfaceTable_Create")
	BroadPhaseLayerInterfaceTable_Create :: proc(numObjectLayers: u32, numBroadPhaseLayers: u32) -> ^BroadPhaseLayerInterface ---

	@(link_name = "JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer")
	BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer :: proc(bpInterface: ^BroadPhaseLayerInterface, objectLayer: ObjectLayer, broadPhaseLayer: BroadPhaseLayer) ---

	@(link_name = "JPH_ObjectLayerPairFilterMask_Create")
	ObjectLayerPairFilterMask_Create :: proc() -> ^ObjectLayerPairFilter ---

	@(link_name = "JPH_ObjectLayerPairFilterMask_GetObjectLayer")
	ObjectLayerPairFilterMask_GetObjectLayer :: proc(group: u32, mask: u32) -> ObjectLayer ---

	@(link_name = "JPH_ObjectLayerPairFilterMask_GetGroup")
	ObjectLayerPairFilterMask_GetGroup :: proc(layer: ObjectLayer) -> u32 ---

	@(link_name = "JPH_ObjectLayerPairFilterMask_GetMask")
	ObjectLayerPairFilterMask_GetMask :: proc(layer: ObjectLayer) -> u32 ---

	@(link_name = "JPH_ObjectLayerPairFilterTable_Create")
	ObjectLayerPairFilterTable_Create :: proc(numObjectLayers: u32) -> ^ObjectLayerPairFilter ---

	@(link_name = "JPH_ObjectLayerPairFilterTable_DisableCollision")
	ObjectLayerPairFilterTable_DisableCollision :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---

	@(link_name = "JPH_ObjectLayerPairFilterTable_EnableCollision")
	ObjectLayerPairFilterTable_EnableCollision :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---

	@(link_name = "JPH_ObjectLayerPairFilterTable_ShouldCollide")
	ObjectLayerPairFilterTable_ShouldCollide :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) -> b8 ---

	@(link_name = "JPH_ObjectVsBroadPhaseLayerFilterMask_Create")
	ObjectVsBroadPhaseLayerFilterMask_Create :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface) -> ^ObjectVsBroadPhaseLayerFilter ---

	@(link_name = "JPH_ObjectVsBroadPhaseLayerFilterTable_Create")
	ObjectVsBroadPhaseLayerFilterTable_Create :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface, numBroadPhaseLayers: u32, objectLayerPairFilter: ^ObjectLayerPairFilter, numObjectLayers: u32) -> ^ObjectVsBroadPhaseLayerFilter ---

	@(link_name = "JPH_DrawSettings_InitDefault")
	DrawSettings_InitDefault :: proc(settings: [^]DrawSettings) ---

	@(link_name = "JPH_PhysicsSystem_Create")
	PhysicsSystem_Create :: proc(settings: [^]PhysicsSystemSettings) -> ^PhysicsSystem ---

	@(link_name = "JPH_PhysicsSystem_Destroy")
	PhysicsSystem_Destroy :: proc(system: ^PhysicsSystem) ---

	@(link_name = "JPH_PhysicsSystem_SetPhysicsSettings")
	PhysicsSystem_SetPhysicsSettings :: proc(system: ^PhysicsSystem, settings: [^]PhysicsSettings) ---

	@(link_name = "JPH_PhysicsSystem_GetPhysicsSettings")
	PhysicsSystem_GetPhysicsSettings :: proc(system: ^PhysicsSystem, result: ^PhysicsSettings) ---

	@(link_name = "JPH_PhysicsSystem_OptimizeBroadPhase")
	PhysicsSystem_OptimizeBroadPhase :: proc(system: ^PhysicsSystem) ---

	@(link_name = "JPH_PhysicsSystem_Update")
	PhysicsSystem_Update :: proc(system: ^PhysicsSystem, deltaTime: f32, collisionSteps: i32, jobSystem: ^JobSystem) -> PhysicsUpdateError ---

	@(link_name = "JPH_PhysicsSystem_GetBodyInterface")
	PhysicsSystem_GetBodyInterface :: proc(system: ^PhysicsSystem) -> ^BodyInterface ---

	@(link_name = "JPH_PhysicsSystem_GetBodyInterfaceNoLock")
	PhysicsSystem_GetBodyInterfaceNoLock :: proc(system: ^PhysicsSystem) -> ^BodyInterface ---

	@(link_name = "JPH_PhysicsSystem_GetBodyLockInterface")
	PhysicsSystem_GetBodyLockInterface :: proc(system: ^PhysicsSystem) -> ^BodyLockInterface ---

	@(link_name = "JPH_PhysicsSystem_GetBodyLockInterfaceNoLock")
	PhysicsSystem_GetBodyLockInterfaceNoLock :: proc(system: ^PhysicsSystem) -> ^BodyLockInterface ---

	@(link_name = "JPH_PhysicsSystem_GetBroadPhaseQuery")
	PhysicsSystem_GetBroadPhaseQuery :: proc(system: ^PhysicsSystem) -> ^BroadPhaseQuery ---

	@(link_name = "JPH_PhysicsSystem_GetNarrowPhaseQuery")
	PhysicsSystem_GetNarrowPhaseQuery :: proc(system: ^PhysicsSystem) -> ^NarrowPhaseQuery ---

	@(link_name = "JPH_PhysicsSystem_GetNarrowPhaseQueryNoLock")
	PhysicsSystem_GetNarrowPhaseQueryNoLock :: proc(system: ^PhysicsSystem) -> ^NarrowPhaseQuery ---

	@(link_name = "JPH_PhysicsSystem_SetContactListener")
	PhysicsSystem_SetContactListener :: proc(system: ^PhysicsSystem, listener: ^ContactListener) ---

	@(link_name = "JPH_PhysicsSystem_SetBodyActivationListener")
	PhysicsSystem_SetBodyActivationListener :: proc(system: ^PhysicsSystem, listener: ^BodyActivationListener) ---

	@(link_name = "JPH_PhysicsSystem_SetSimShapeFilter")
	PhysicsSystem_SetSimShapeFilter :: proc(system: ^PhysicsSystem, filter: ^SimShapeFilter) ---

	@(link_name = "JPH_PhysicsSystem_WereBodiesInContact")
	PhysicsSystem_WereBodiesInContact :: proc(system: ^PhysicsSystem, body1: BodyID, body2: BodyID) -> b8 ---

	@(link_name = "JPH_PhysicsSystem_GetNumBodies")
	PhysicsSystem_GetNumBodies :: proc(system: ^PhysicsSystem) -> u32 ---

	@(link_name = "JPH_PhysicsSystem_GetNumActiveBodies")
	PhysicsSystem_GetNumActiveBodies :: proc(system: ^PhysicsSystem, type: BodyType) -> u32 ---

	@(link_name = "JPH_PhysicsSystem_GetMaxBodies")
	PhysicsSystem_GetMaxBodies :: proc(system: ^PhysicsSystem) -> u32 ---

	@(link_name = "JPH_PhysicsSystem_GetNumConstraints")
	PhysicsSystem_GetNumConstraints :: proc(system: ^PhysicsSystem) -> u32 ---

	@(link_name = "JPH_PhysicsSystem_SetGravity")
	PhysicsSystem_SetGravity :: proc(system: ^PhysicsSystem, value: ^Vec3) ---

	@(link_name = "JPH_PhysicsSystem_GetGravity")
	PhysicsSystem_GetGravity :: proc(system: ^PhysicsSystem, result: ^Vec3) ---

	@(link_name = "JPH_PhysicsSystem_AddConstraint")
	PhysicsSystem_AddConstraint :: proc(system: ^PhysicsSystem, constraint: ^Constraint) ---

	@(link_name = "JPH_PhysicsSystem_RemoveConstraint")
	PhysicsSystem_RemoveConstraint :: proc(system: ^PhysicsSystem, constraint: ^Constraint) ---

	@(link_name = "JPH_PhysicsSystem_AddConstraints")
	PhysicsSystem_AddConstraints :: proc(system: ^PhysicsSystem, constraints: [^]^Constraint, count: u32) ---

	@(link_name = "JPH_PhysicsSystem_RemoveConstraints")
	PhysicsSystem_RemoveConstraints :: proc(system: ^PhysicsSystem, constraints: [^]^Constraint, count: u32) ---

	@(link_name = "JPH_PhysicsSystem_GetBodies")
	PhysicsSystem_GetBodies :: proc(system: ^PhysicsSystem, ids: [^]BodyID, count: u32) ---

	@(link_name = "JPH_PhysicsSystem_GetConstraints")
	PhysicsSystem_GetConstraints :: proc(system: ^PhysicsSystem, constraints: [^]^Constraint, count: u32) ---

	@(link_name = "JPH_PhysicsSystem_DrawBodies")
	PhysicsSystem_DrawBodies :: proc(system: ^PhysicsSystem, settings: [^]DrawSettings, renderer: ^DebugRenderer, bodyFilter: ^BodyDrawFilter) ---

	@(link_name = "JPH_PhysicsSystem_DrawConstraints")
	PhysicsSystem_DrawConstraints :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---

	@(link_name = "JPH_PhysicsSystem_DrawConstraintLimits")
	PhysicsSystem_DrawConstraintLimits :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---

	@(link_name = "JPH_PhysicsSystem_DrawConstraintReferenceFrame")
	PhysicsSystem_DrawConstraintReferenceFrame :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---

	@(link_name = "JPH_Quaternion_FromTo")
	Quaternion_FromTo :: proc(from: ^Vec3, to: ^Vec3, quat: ^Quat) ---

	@(link_name = "JPH_Quat_GetAxisAngle")
	Quat_GetAxisAngle :: proc(quat: ^Quat, outAxis: [^]Vec3, outAngle: ^f32) ---

	@(link_name = "JPH_Quat_GetEulerAngles")
	Quat_GetEulerAngles :: proc(quat: ^Quat, result: ^Vec3) ---

	@(link_name = "JPH_Quat_RotateAxisX")
	Quat_RotateAxisX :: proc(quat: ^Quat, result: ^Vec3) ---

	@(link_name = "JPH_Quat_RotateAxisY")
	Quat_RotateAxisY :: proc(quat: ^Quat, result: ^Vec3) ---

	@(link_name = "JPH_Quat_RotateAxisZ")
	Quat_RotateAxisZ :: proc(quat: ^Quat, result: ^Vec3) ---

	@(link_name = "JPH_Quat_Inversed")
	Quat_Inversed :: proc(quat: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_GetPerpendicular")
	Quat_GetPerpendicular :: proc(quat: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_GetRotationAngle")
	Quat_GetRotationAngle :: proc(quat: ^Quat, axis: [^]Vec3) -> f32 ---

	@(link_name = "JPH_Quat_FromEulerAngles")
	Quat_FromEulerAngles :: proc(angles: [^]Vec3, result: ^Quat) ---

	@(link_name = "JPH_Quat_Add")
	Quat_Add :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_Subtract")
	Quat_Subtract :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_Multiply")
	Quat_Multiply :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_MultiplyScalar")
	Quat_MultiplyScalar :: proc(q: ^Quat, scalar: f32, result: ^Quat) ---

	@(link_name = "JPH_Quat_Divide")
	Quat_Divide :: proc(q1: ^Quat, q2: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_Dot")
	Quat_Dot :: proc(q1: ^Quat, q2: ^Quat, result: ^f32) ---

	@(link_name = "JPH_Quat_Conjugated")
	Quat_Conjugated :: proc(quat: ^Quat, result: ^Quat) ---

	@(link_name = "JPH_Quat_GetTwist")
	Quat_GetTwist :: proc(quat: ^Quat, axis: [^]Vec3, result: ^Quat) ---

	@(link_name = "JPH_Quat_GetSwingTwist")
	Quat_GetSwingTwist :: proc(quat: ^Quat, outSwing: ^Quat, outTwist: ^Quat) ---

	@(link_name = "JPH_Quat_LERP")
	Quat_LERP :: proc(from: ^Quat, to: ^Quat, fraction: f32, result: ^Quat) ---

	@(link_name = "JPH_Quat_SLERP")
	Quat_SLERP :: proc(from: ^Quat, to: ^Quat, fraction: f32, result: ^Quat) ---

	@(link_name = "JPH_Quat_Rotate")
	Quat_Rotate :: proc(quat: ^Quat, vec: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Quat_InverseRotate")
	Quat_InverseRotate :: proc(quat: ^Quat, vec: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_IsClose")
	Vec3_IsClose :: proc(v1: ^Vec3, v2: ^Vec3, maxDistSq: f32) -> b8 ---

	@(link_name = "JPH_Vec3_IsNearZero")
	Vec3_IsNearZero :: proc(v: ^Vec3, maxDistSq: f32) -> b8 ---

	@(link_name = "JPH_Vec3_IsNormalized")
	Vec3_IsNormalized :: proc(v: ^Vec3, tolerance: f32) -> b8 ---

	@(link_name = "JPH_Vec3_IsNaN")
	Vec3_IsNaN :: proc(v: ^Vec3) -> b8 ---

	@(link_name = "JPH_Vec3_Negate")
	Vec3_Negate :: proc(v: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Normalized")
	Vec3_Normalized :: proc(v: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Cross")
	Vec3_Cross :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Abs")
	Vec3_Abs :: proc(v: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Length")
	Vec3_Length :: proc(v: ^Vec3) -> f32 ---

	@(link_name = "JPH_Vec3_LengthSquared")
	Vec3_LengthSquared :: proc(v: ^Vec3) -> f32 ---

	@(link_name = "JPH_Vec3_DotProduct")
	Vec3_DotProduct :: proc(v1: ^Vec3, v2: ^Vec3, result: ^f32) ---

	@(link_name = "JPH_Vec3_Normalize")
	Vec3_Normalize :: proc(v: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Add")
	Vec3_Add :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Subtract")
	Vec3_Subtract :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Multiply")
	Vec3_Multiply :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_MultiplyScalar")
	Vec3_MultiplyScalar :: proc(v: ^Vec3, scalar: f32, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_Divide")
	Vec3_Divide :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Vec3_DivideScalar")
	Vec3_DivideScalar :: proc(v: ^Vec3, scalar: f32, result: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_Add")
	Matrix4x4_Add :: proc(m1: ^Matrix4x4, m2: ^Matrix4x4, result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_Subtract")
	Matrix4x4_Subtract :: proc(m1: ^Matrix4x4, m2: ^Matrix4x4, result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_Multiply")
	Matrix4x4_Multiply :: proc(m1: ^Matrix4x4, m2: ^Matrix4x4, result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_MultiplyScalar")
	Matrix4x4_MultiplyScalar :: proc(m: ^Matrix4x4, scalar: f32, result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_Zero")
	Matrix4x4_Zero :: proc(result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_Identity")
	Matrix4x4_Identity :: proc(result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_Rotation")
	Matrix4x4_Rotation :: proc(result: ^Matrix4x4, rotation: ^Quat) ---

	@(link_name = "JPH_Matrix4x4_Translation")
	Matrix4x4_Translation :: proc(result: ^Matrix4x4, translation: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_RotationTranslation")
	Matrix4x4_RotationTranslation :: proc(result: ^Matrix4x4, rotation: ^Quat, translation: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_InverseRotationTranslation")
	Matrix4x4_InverseRotationTranslation :: proc(result: ^Matrix4x4, rotation: ^Quat, translation: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_Scale")
	Matrix4x4_Scale :: proc(result: ^Matrix4x4, scale: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_Inversed")
	Matrix4x4_Inversed :: proc(m: ^Matrix4x4, result: ^Matrix4x4) ---

	@(link_name = "JPH_Matrix4x4_Transposed")
	Matrix4x4_Transposed :: proc(m: ^Matrix4x4, result: ^Matrix4x4) ---

	@(link_name = "JPH_RMatrix4x4_Zero")
	RMatrix4x4_Zero :: proc(result: ^RMatrix4x4) ---

	@(link_name = "JPH_RMatrix4x4_Identity")
	RMatrix4x4_Identity :: proc(result: ^RMatrix4x4) ---

	@(link_name = "JPH_RMatrix4x4_Rotation")
	RMatrix4x4_Rotation :: proc(result: ^RMatrix4x4, rotation: ^Quat) ---

	@(link_name = "JPH_RMatrix4x4_Translation")
	RMatrix4x4_Translation :: proc(result: ^RMatrix4x4, translation: ^RVec3) ---

	@(link_name = "JPH_RMatrix4x4_RotationTranslation")
	RMatrix4x4_RotationTranslation :: proc(result: ^RMatrix4x4, rotation: ^Quat, translation: ^RVec3) ---

	@(link_name = "JPH_RMatrix4x4_InverseRotationTranslation")
	RMatrix4x4_InverseRotationTranslation :: proc(result: ^RMatrix4x4, rotation: ^Quat, translation: ^RVec3) ---

	@(link_name = "JPH_RMatrix4x4_Scale")
	RMatrix4x4_Scale :: proc(result: ^RMatrix4x4, scale: ^Vec3) ---

	@(link_name = "JPH_RMatrix4x4_Inversed")
	RMatrix4x4_Inversed :: proc(m: ^RMatrix4x4, result: ^RMatrix4x4) ---

	@(link_name = "JPH_Matrix4x4_GetAxisX")
	Matrix4x4_GetAxisX :: proc(matrix_p: ^Matrix4x4, result: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_GetAxisY")
	Matrix4x4_GetAxisY :: proc(matrix_p: ^Matrix4x4, result: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_GetAxisZ")
	Matrix4x4_GetAxisZ :: proc(matrix_p: ^Matrix4x4, result: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_GetTranslation")
	Matrix4x4_GetTranslation :: proc(matrix_p: ^Matrix4x4, result: ^Vec3) ---

	@(link_name = "JPH_Matrix4x4_GetQuaternion")
	Matrix4x4_GetQuaternion :: proc(matrix_p: ^Matrix4x4, result: ^Quat) ---

	@(link_name = "JPH_PhysicsMaterial_Create")
	PhysicsMaterial_Create :: proc(name: cstring, color: u32) -> ^PhysicsMaterial ---

	@(link_name = "JPH_PhysicsMaterial_Destroy")
	PhysicsMaterial_Destroy :: proc(material: ^PhysicsMaterial) ---

	@(link_name = "JPH_PhysicsMaterial_GetDebugName")
	PhysicsMaterial_GetDebugName :: proc(material: ^PhysicsMaterial) -> cstring ---

	@(link_name = "JPH_PhysicsMaterial_GetDebugColor")
	PhysicsMaterial_GetDebugColor :: proc(material: ^PhysicsMaterial) -> u32 ---

	@(link_name = "JPH_ShapeSettings_Destroy")
	ShapeSettings_Destroy :: proc(settings: [^]ShapeSettings) ---

	@(link_name = "JPH_ShapeSettings_GetUserData")
	ShapeSettings_GetUserData :: proc(settings: [^]ShapeSettings) -> u64 ---

	@(link_name = "JPH_ShapeSettings_SetUserData")
	ShapeSettings_SetUserData :: proc(settings: [^]ShapeSettings, userData: u64) ---

	@(link_name = "JPH_Shape_Destroy")
	Shape_Destroy :: proc(shape: ^Shape) ---

	@(link_name = "JPH_Shape_GetType")
	Shape_GetType :: proc(shape: ^Shape) -> ShapeType ---

	@(link_name = "JPH_Shape_GetSubType")
	Shape_GetSubType :: proc(shape: ^Shape) -> ShapeSubType ---

	@(link_name = "JPH_Shape_GetUserData")
	Shape_GetUserData :: proc(shape: ^Shape) -> u64 ---

	@(link_name = "JPH_Shape_SetUserData")
	Shape_SetUserData :: proc(shape: ^Shape, userData: u64) ---

	@(link_name = "JPH_Shape_MustBeStatic")
	Shape_MustBeStatic :: proc(shape: ^Shape) -> b8 ---

	@(link_name = "JPH_Shape_GetCenterOfMass")
	Shape_GetCenterOfMass :: proc(shape: ^Shape, result: ^Vec3) ---

	@(link_name = "JPH_Shape_GetLocalBounds")
	Shape_GetLocalBounds :: proc(shape: ^Shape, result: ^AABox) ---

	@(link_name = "JPH_Shape_GetSubShapeIDBitsRecursive")
	Shape_GetSubShapeIDBitsRecursive :: proc(shape: ^Shape) -> u32 ---

	@(link_name = "JPH_Shape_GetWorldSpaceBounds")
	Shape_GetWorldSpaceBounds :: proc(shape: ^Shape, centerOfMassTransform: ^RMatrix4x4, scale: ^Vec3, result: ^AABox) ---

	@(link_name = "JPH_Shape_GetInnerRadius")
	Shape_GetInnerRadius :: proc(shape: ^Shape) -> f32 ---

	@(link_name = "JPH_Shape_GetMassProperties")
	Shape_GetMassProperties :: proc(shape: ^Shape, result: ^MassProperties) ---

	@(link_name = "JPH_Shape_GetLeafShape")
	Shape_GetLeafShape :: proc(shape: ^Shape, subShapeID: SubShapeID, remainder: ^SubShapeID) -> ^Shape ---

	@(link_name = "JPH_Shape_GetMaterial")
	Shape_GetMaterial :: proc(shape: ^Shape, subShapeID: SubShapeID) -> ^PhysicsMaterial ---

	@(link_name = "JPH_Shape_GetSurfaceNormal")
	Shape_GetSurfaceNormal :: proc(shape: ^Shape, subShapeID: SubShapeID, localPosition: ^Vec3, normal: ^Vec3) ---

	@(link_name = "JPH_Shape_GetSupportingFace")
	Shape_GetSupportingFace :: proc(shape: ^Shape, subShapeID: SubShapeID, direction: ^Vec3, scale: ^Vec3, centerOfMassTransform: ^Matrix4x4, outVertices: [^]SupportingFace) ---

	@(link_name = "JPH_Shape_GetVolume")
	Shape_GetVolume :: proc(shape: ^Shape) -> f32 ---

	@(link_name = "JPH_Shape_IsValidScale")
	Shape_IsValidScale :: proc(shape: ^Shape, scale: ^Vec3) -> b8 ---

	@(link_name = "JPH_Shape_MakeScaleValid")
	Shape_MakeScaleValid :: proc(shape: ^Shape, scale: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_Shape_ScaleShape")
	Shape_ScaleShape :: proc(shape: ^Shape, scale: ^Vec3) -> ^Shape ---

	@(link_name = "JPH_Shape_CastRay")
	Shape_CastRay :: proc(shape: ^Shape, origin: ^Vec3, direction: ^Vec3, hit: ^RayCastResult) -> b8 ---

	@(link_name = "JPH_Shape_CastRay2")
	Shape_CastRay2 :: proc(shape: ^Shape, origin: ^Vec3, direction: ^Vec3, rayCastSettings: [^]RayCastSettings, collectorType: CollisionCollectorType, callback: ^CastRayResultCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_Shape_CollidePoint")
	Shape_CollidePoint :: proc(shape: ^Shape, point: ^Vec3, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_Shape_CollidePoint2")
	Shape_CollidePoint2 :: proc(shape: ^Shape, point: ^Vec3, collectorType: CollisionCollectorType, callback: ^CollidePointResultCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_ConvexShapeSettings_GetDensity")
	ConvexShapeSettings_GetDensity :: proc(shape: ^ConvexShapeSettings) -> f32 ---

	@(link_name = "JPH_ConvexShapeSettings_SetDensity")
	ConvexShapeSettings_SetDensity :: proc(shape: ^ConvexShapeSettings, value: f32) ---

	@(link_name = "JPH_ConvexShape_GetDensity")
	ConvexShape_GetDensity :: proc(shape: ^ConvexShape) -> f32 ---

	@(link_name = "JPH_ConvexShape_SetDensity")
	ConvexShape_SetDensity :: proc(shape: ^ConvexShape, inDensity: f32) ---

	@(link_name = "JPH_BoxShapeSettings_Create")
	BoxShapeSettings_Create :: proc(halfExtent: ^Vec3, convexRadius: f32) -> ^BoxShapeSettings ---

	@(link_name = "JPH_BoxShapeSettings_CreateShape")
	BoxShapeSettings_CreateShape :: proc(settings: [^]BoxShapeSettings) -> ^BoxShape ---

	@(link_name = "JPH_BoxShape_Create")
	BoxShape_Create :: proc(halfExtent: ^Vec3, convexRadius: f32) -> ^BoxShape ---

	@(link_name = "JPH_BoxShape_GetHalfExtent")
	BoxShape_GetHalfExtent :: proc(shape: ^BoxShape, halfExtent: ^Vec3) ---

	@(link_name = "JPH_BoxShape_GetConvexRadius")
	BoxShape_GetConvexRadius :: proc(shape: ^BoxShape) -> f32 ---

	@(link_name = "JPH_SphereShapeSettings_Create")
	SphereShapeSettings_Create :: proc(radius: f32) -> ^SphereShapeSettings ---

	@(link_name = "JPH_SphereShapeSettings_CreateShape")
	SphereShapeSettings_CreateShape :: proc(settings: [^]SphereShapeSettings) -> ^SphereShape ---

	@(link_name = "JPH_SphereShapeSettings_GetRadius")
	SphereShapeSettings_GetRadius :: proc(settings: [^]SphereShapeSettings) -> f32 ---

	@(link_name = "JPH_SphereShapeSettings_SetRadius")
	SphereShapeSettings_SetRadius :: proc(settings: [^]SphereShapeSettings, radius: f32) ---

	@(link_name = "JPH_SphereShape_Create")
	SphereShape_Create :: proc(radius: f32) -> ^SphereShape ---

	@(link_name = "JPH_SphereShape_GetRadius")
	SphereShape_GetRadius :: proc(shape: ^SphereShape) -> f32 ---

	@(link_name = "JPH_PlaneShapeSettings_Create")
	PlaneShapeSettings_Create :: proc(plane: ^Plane, material: ^PhysicsMaterial, halfExtent: f32) -> ^PlaneShapeSettings ---

	@(link_name = "JPH_PlaneShapeSettings_CreateShape")
	PlaneShapeSettings_CreateShape :: proc(settings: [^]PlaneShapeSettings) -> ^PlaneShape ---

	@(link_name = "JPH_PlaneShape_Create")
	PlaneShape_Create :: proc(plane: ^Plane, material: ^PhysicsMaterial, halfExtent: f32) -> ^PlaneShape ---

	@(link_name = "JPH_PlaneShape_GetPlane")
	PlaneShape_GetPlane :: proc(shape: ^PlaneShape, result: ^Plane) ---

	@(link_name = "JPH_PlaneShape_GetHalfExtent")
	PlaneShape_GetHalfExtent :: proc(shape: ^PlaneShape) -> f32 ---

	@(link_name = "JPH_TriangleShapeSettings_Create")
	TriangleShapeSettings_Create :: proc(v1: ^Vec3, v2: ^Vec3, v3: ^Vec3, convexRadius: f32) -> ^TriangleShapeSettings ---

	@(link_name = "JPH_TriangleShapeSettings_CreateShape")
	TriangleShapeSettings_CreateShape :: proc(settings: [^]TriangleShapeSettings) -> ^TriangleShape ---

	@(link_name = "JPH_TriangleShape_Create")
	TriangleShape_Create :: proc(v1: ^Vec3, v2: ^Vec3, v3: ^Vec3, convexRadius: f32) -> ^TriangleShape ---

	@(link_name = "JPH_TriangleShape_GetConvexRadius")
	TriangleShape_GetConvexRadius :: proc(shape: ^TriangleShape) -> f32 ---

	@(link_name = "JPH_TriangleShape_GetVertex1")
	TriangleShape_GetVertex1 :: proc(shape: ^TriangleShape, result: ^Vec3) ---

	@(link_name = "JPH_TriangleShape_GetVertex2")
	TriangleShape_GetVertex2 :: proc(shape: ^TriangleShape, result: ^Vec3) ---

	@(link_name = "JPH_TriangleShape_GetVertex3")
	TriangleShape_GetVertex3 :: proc(shape: ^TriangleShape, result: ^Vec3) ---

	@(link_name = "JPH_CapsuleShapeSettings_Create")
	CapsuleShapeSettings_Create :: proc(halfHeightOfCylinder: f32, radius: f32) -> ^CapsuleShapeSettings ---

	@(link_name = "JPH_CapsuleShapeSettings_CreateShape")
	CapsuleShapeSettings_CreateShape :: proc(settings: [^]CapsuleShapeSettings) -> ^CapsuleShape ---

	@(link_name = "JPH_CapsuleShape_Create")
	CapsuleShape_Create :: proc(halfHeightOfCylinder: f32, radius: f32) -> ^CapsuleShape ---

	@(link_name = "JPH_CapsuleShape_GetRadius")
	CapsuleShape_GetRadius :: proc(shape: ^CapsuleShape) -> f32 ---

	@(link_name = "JPH_CapsuleShape_GetHalfHeightOfCylinder")
	CapsuleShape_GetHalfHeightOfCylinder :: proc(shape: ^CapsuleShape) -> f32 ---

	@(link_name = "JPH_CylinderShapeSettings_Create")
	CylinderShapeSettings_Create :: proc(halfHeight: f32, radius: f32, convexRadius: f32) -> ^CylinderShapeSettings ---

	@(link_name = "JPH_CylinderShapeSettings_CreateShape")
	CylinderShapeSettings_CreateShape :: proc(settings: [^]CylinderShapeSettings) -> ^CylinderShape ---

	@(link_name = "JPH_CylinderShape_Create")
	CylinderShape_Create :: proc(halfHeight: f32, radius: f32) -> ^CylinderShape ---

	@(link_name = "JPH_CylinderShape_GetRadius")
	CylinderShape_GetRadius :: proc(shape: ^CylinderShape) -> f32 ---

	@(link_name = "JPH_CylinderShape_GetHalfHeight")
	CylinderShape_GetHalfHeight :: proc(shape: ^CylinderShape) -> f32 ---

	@(link_name = "JPH_TaperedCylinderShapeSettings_Create")
	TaperedCylinderShapeSettings_Create :: proc(halfHeightOfTaperedCylinder: f32, topRadius: f32, bottomRadius: f32, convexRadius: f32, material: ^PhysicsMaterial) -> ^TaperedCylinderShapeSettings ---

	@(link_name = "JPH_TaperedCylinderShapeSettings_CreateShape")
	TaperedCylinderShapeSettings_CreateShape :: proc(settings: [^]TaperedCylinderShapeSettings) -> ^TaperedCylinderShape ---

	@(link_name = "JPH_TaperedCylinderShape_GetTopRadius")
	TaperedCylinderShape_GetTopRadius :: proc(shape: ^TaperedCylinderShape) -> f32 ---

	@(link_name = "JPH_TaperedCylinderShape_GetBottomRadius")
	TaperedCylinderShape_GetBottomRadius :: proc(shape: ^TaperedCylinderShape) -> f32 ---

	@(link_name = "JPH_TaperedCylinderShape_GetConvexRadius")
	TaperedCylinderShape_GetConvexRadius :: proc(shape: ^TaperedCylinderShape) -> f32 ---

	@(link_name = "JPH_TaperedCylinderShape_GetHalfHeight")
	TaperedCylinderShape_GetHalfHeight :: proc(shape: ^TaperedCylinderShape) -> f32 ---

	@(link_name = "JPH_ConvexHullShapeSettings_Create")
	ConvexHullShapeSettings_Create :: proc(points: [^]Vec3, pointsCount: u32, maxConvexRadius: f32) -> ^ConvexHullShapeSettings ---

	@(link_name = "JPH_ConvexHullShapeSettings_CreateShape")
	ConvexHullShapeSettings_CreateShape :: proc(settings: [^]ConvexHullShapeSettings) -> ^ConvexHullShape ---

	@(link_name = "JPH_ConvexHullShape_GetNumPoints")
	ConvexHullShape_GetNumPoints :: proc(shape: ^ConvexHullShape) -> u32 ---

	@(link_name = "JPH_ConvexHullShape_GetPoint")
	ConvexHullShape_GetPoint :: proc(shape: ^ConvexHullShape, index: u32, result: ^Vec3) ---

	@(link_name = "JPH_ConvexHullShape_GetNumFaces")
	ConvexHullShape_GetNumFaces :: proc(shape: ^ConvexHullShape) -> u32 ---

	@(link_name = "JPH_ConvexHullShape_GetNumVerticesInFace")
	ConvexHullShape_GetNumVerticesInFace :: proc(shape: ^ConvexHullShape, faceIndex: u32) -> u32 ---

	@(link_name = "JPH_ConvexHullShape_GetFaceVertices")
	ConvexHullShape_GetFaceVertices :: proc(shape: ^ConvexHullShape, faceIndex: u32, maxVertices: u32, vertices: [^]u32) -> u32 ---

	@(link_name = "JPH_MeshShapeSettings_Create")
	MeshShapeSettings_Create :: proc(triangles: [^]Triangle, triangleCount: u32) -> ^MeshShapeSettings ---

	@(link_name = "JPH_MeshShapeSettings_Create2")
	MeshShapeSettings_Create2 :: proc(vertices: [^]Vec3, verticesCount: u32, triangles: [^]IndexedTriangle, triangleCount: u32) -> ^MeshShapeSettings ---

	@(link_name = "JPH_MeshShapeSettings_GetMaxTrianglesPerLeaf")
	MeshShapeSettings_GetMaxTrianglesPerLeaf :: proc(settings: [^]MeshShapeSettings) -> u32 ---

	@(link_name = "JPH_MeshShapeSettings_SetMaxTrianglesPerLeaf")
	MeshShapeSettings_SetMaxTrianglesPerLeaf :: proc(settings: [^]MeshShapeSettings, value: u32) ---

	@(link_name = "JPH_MeshShapeSettings_GetActiveEdgeCosThresholdAngle")
	MeshShapeSettings_GetActiveEdgeCosThresholdAngle :: proc(settings: [^]MeshShapeSettings) -> f32 ---

	@(link_name = "JPH_MeshShapeSettings_SetActiveEdgeCosThresholdAngle")
	MeshShapeSettings_SetActiveEdgeCosThresholdAngle :: proc(settings: [^]MeshShapeSettings, value: f32) ---

	@(link_name = "JPH_MeshShapeSettings_GetPerTriangleUserData")
	MeshShapeSettings_GetPerTriangleUserData :: proc(settings: [^]MeshShapeSettings) -> b8 ---

	@(link_name = "JPH_MeshShapeSettings_SetPerTriangleUserData")
	MeshShapeSettings_SetPerTriangleUserData :: proc(settings: [^]MeshShapeSettings, value: b8) ---

	@(link_name = "JPH_MeshShapeSettings_GetBuildQuality")
	MeshShapeSettings_GetBuildQuality :: proc(settings: [^]MeshShapeSettings) -> Mesh_Shape_BuildQuality ---

	@(link_name = "JPH_MeshShapeSettings_SetBuildQuality")
	MeshShapeSettings_SetBuildQuality :: proc(settings: [^]MeshShapeSettings, value: Mesh_Shape_BuildQuality) ---

	@(link_name = "JPH_MeshShapeSettings_Sanitize")
	MeshShapeSettings_Sanitize :: proc(settings: [^]MeshShapeSettings) ---

	@(link_name = "JPH_MeshShapeSettings_CreateShape")
	MeshShapeSettings_CreateShape :: proc(settings: [^]MeshShapeSettings) -> ^MeshShape ---

	@(link_name = "JPH_MeshShape_GetTriangleUserData")
	MeshShape_GetTriangleUserData :: proc(shape: ^MeshShape, id: SubShapeID) -> u32 ---

	@(link_name = "JPH_HeightFieldShapeSettings_Create")
	HeightFieldShapeSettings_Create :: proc(samples: [^]f32, offset: ^Vec3, scale: ^Vec3, sampleCount: u32) -> ^HeightFieldShapeSettings ---

	@(link_name = "JPH_HeightFieldShapeSettings_CreateShape")
	HeightFieldShapeSettings_CreateShape :: proc(settings: [^]HeightFieldShapeSettings) -> ^HeightFieldShape ---

	@(link_name = "JPH_HeightFieldShapeSettings_DetermineMinAndMaxSample")
	HeightFieldShapeSettings_DetermineMinAndMaxSample :: proc(settings: [^]HeightFieldShapeSettings, pOutMinValue: ^f32, pOutMaxValue: ^f32, pOutQuantizationScale: ^f32) ---

	@(link_name = "JPH_HeightFieldShapeSettings_CalculateBitsPerSampleForError")
	HeightFieldShapeSettings_CalculateBitsPerSampleForError :: proc(settings: [^]HeightFieldShapeSettings, maxError: f32) -> u32 ---

	@(link_name = "JPH_HeightFieldShape_GetSampleCount")
	HeightFieldShape_GetSampleCount :: proc(shape: ^HeightFieldShape) -> u32 ---

	@(link_name = "JPH_HeightFieldShape_GetBlockSize")
	HeightFieldShape_GetBlockSize :: proc(shape: ^HeightFieldShape) -> u32 ---

	@(link_name = "JPH_HeightFieldShape_GetMaterial")
	HeightFieldShape_GetMaterial :: proc(shape: ^HeightFieldShape, x: u32, y: u32) -> ^PhysicsMaterial ---

	@(link_name = "JPH_HeightFieldShape_GetPosition")
	HeightFieldShape_GetPosition :: proc(shape: ^HeightFieldShape, x: u32, y: u32, result: ^Vec3) ---

	@(link_name = "JPH_HeightFieldShape_IsNoCollision")
	HeightFieldShape_IsNoCollision :: proc(shape: ^HeightFieldShape, x: u32, y: u32) -> b8 ---

	@(link_name = "JPH_HeightFieldShape_ProjectOntoSurface")
	HeightFieldShape_ProjectOntoSurface :: proc(shape: ^HeightFieldShape, localPosition: ^Vec3, outSurfacePosition: ^Vec3, outSubShapeID: ^SubShapeID) -> b8 ---

	@(link_name = "JPH_HeightFieldShape_GetMinHeightValue")
	HeightFieldShape_GetMinHeightValue :: proc(shape: ^HeightFieldShape) -> f32 ---

	@(link_name = "JPH_HeightFieldShape_GetMaxHeightValue")
	HeightFieldShape_GetMaxHeightValue :: proc(shape: ^HeightFieldShape) -> f32 ---

	@(link_name = "JPH_TaperedCapsuleShapeSettings_Create")
	TaperedCapsuleShapeSettings_Create :: proc(halfHeightOfTaperedCylinder: f32, topRadius: f32, bottomRadius: f32) -> ^TaperedCapsuleShapeSettings ---

	@(link_name = "JPH_TaperedCapsuleShapeSettings_CreateShape")
	TaperedCapsuleShapeSettings_CreateShape :: proc(settings: [^]TaperedCapsuleShapeSettings) -> ^TaperedCapsuleShape ---

	@(link_name = "JPH_TaperedCapsuleShape_GetTopRadius")
	TaperedCapsuleShape_GetTopRadius :: proc(shape: ^TaperedCapsuleShape) -> f32 ---

	@(link_name = "JPH_TaperedCapsuleShape_GetBottomRadius")
	TaperedCapsuleShape_GetBottomRadius :: proc(shape: ^TaperedCapsuleShape) -> f32 ---

	@(link_name = "JPH_TaperedCapsuleShape_GetHalfHeight")
	TaperedCapsuleShape_GetHalfHeight :: proc(shape: ^TaperedCapsuleShape) -> f32 ---

	@(link_name = "JPH_CompoundShapeSettings_AddShape")
	CompoundShapeSettings_AddShape :: proc(settings: [^]CompoundShapeSettings, position: ^Vec3, rotation: ^Quat, shapeSettings: [^]ShapeSettings, userData: u32) ---

	@(link_name = "JPH_CompoundShapeSettings_AddShape2")
	CompoundShapeSettings_AddShape2 :: proc(settings: [^]CompoundShapeSettings, position: ^Vec3, rotation: ^Quat, shape: ^Shape, userData: u32) ---

	@(link_name = "JPH_CompoundShape_GetNumSubShapes")
	CompoundShape_GetNumSubShapes :: proc(shape: ^CompoundShape) -> u32 ---

	@(link_name = "JPH_CompoundShape_GetSubShape")
	CompoundShape_GetSubShape :: proc(shape: ^CompoundShape, index: u32, subShape: ^^Shape, positionCOM: ^Vec3, rotation: ^Quat, userData: ^u32) ---

	@(link_name = "JPH_CompoundShape_GetSubShapeIndexFromID")
	CompoundShape_GetSubShapeIndexFromID :: proc(shape: ^CompoundShape, id: SubShapeID, remainder: ^SubShapeID) -> u32 ---

	@(link_name = "JPH_StaticCompoundShapeSettings_Create")
	StaticCompoundShapeSettings_Create :: proc() -> ^StaticCompoundShapeSettings ---

	@(link_name = "JPH_StaticCompoundShape_Create")
	StaticCompoundShape_Create :: proc(settings: [^]StaticCompoundShapeSettings) -> ^StaticCompoundShape ---

	@(link_name = "JPH_MutableCompoundShapeSettings_Create")
	MutableCompoundShapeSettings_Create :: proc() -> ^MutableCompoundShapeSettings ---

	@(link_name = "JPH_MutableCompoundShape_Create")
	MutableCompoundShape_Create :: proc(settings: [^]MutableCompoundShapeSettings) -> ^MutableCompoundShape ---

	@(link_name = "JPH_MutableCompoundShape_AddShape")
	MutableCompoundShape_AddShape :: proc(shape: ^MutableCompoundShape, position: ^Vec3, rotation: ^Quat, child: ^Shape, userData: u32, index: u32) -> u32 ---

	@(link_name = "JPH_MutableCompoundShape_RemoveShape")
	MutableCompoundShape_RemoveShape :: proc(shape: ^MutableCompoundShape, index: u32) ---

	@(link_name = "JPH_MutableCompoundShape_ModifyShape")
	MutableCompoundShape_ModifyShape :: proc(shape: ^MutableCompoundShape, index: u32, position: ^Vec3, rotation: ^Quat) ---

	@(link_name = "JPH_MutableCompoundShape_ModifyShape2")
	MutableCompoundShape_ModifyShape2 :: proc(shape: ^MutableCompoundShape, index: u32, position: ^Vec3, rotation: ^Quat, newShape: ^Shape) ---

	@(link_name = "JPH_MutableCompoundShape_AdjustCenterOfMass")
	MutableCompoundShape_AdjustCenterOfMass :: proc(shape: ^MutableCompoundShape) ---

	@(link_name = "JPH_DecoratedShape_GetInnerShape")
	DecoratedShape_GetInnerShape :: proc(shape: ^DecoratedShape) -> ^Shape ---

	@(link_name = "JPH_RotatedTranslatedShapeSettings_Create")
	RotatedTranslatedShapeSettings_Create :: proc(position: ^Vec3, rotation: ^Quat, shapeSettings: [^]ShapeSettings) -> ^RotatedTranslatedShapeSettings ---

	@(link_name = "JPH_RotatedTranslatedShapeSettings_Create2")
	RotatedTranslatedShapeSettings_Create2 :: proc(position: ^Vec3, rotation: ^Quat, shape: ^Shape) -> ^RotatedTranslatedShapeSettings ---

	@(link_name = "JPH_RotatedTranslatedShapeSettings_CreateShape")
	RotatedTranslatedShapeSettings_CreateShape :: proc(settings: [^]RotatedTranslatedShapeSettings) -> ^RotatedTranslatedShape ---

	@(link_name = "JPH_RotatedTranslatedShape_Create")
	RotatedTranslatedShape_Create :: proc(position: ^Vec3, rotation: ^Quat, shape: ^Shape) -> ^RotatedTranslatedShape ---

	@(link_name = "JPH_RotatedTranslatedShape_GetPosition")
	RotatedTranslatedShape_GetPosition :: proc(shape: ^RotatedTranslatedShape, position: ^Vec3) ---

	@(link_name = "JPH_RotatedTranslatedShape_GetRotation")
	RotatedTranslatedShape_GetRotation :: proc(shape: ^RotatedTranslatedShape, rotation: ^Quat) ---

	@(link_name = "JPH_ScaledShapeSettings_Create")
	ScaledShapeSettings_Create :: proc(shapeSettings: [^]ShapeSettings, scale: ^Vec3) -> ^ScaledShapeSettings ---

	@(link_name = "JPH_ScaledShapeSettings_Create2")
	ScaledShapeSettings_Create2 :: proc(shape: ^Shape, scale: ^Vec3) -> ^ScaledShapeSettings ---

	@(link_name = "JPH_ScaledShapeSettings_CreateShape")
	ScaledShapeSettings_CreateShape :: proc(settings: [^]ScaledShapeSettings) -> ^ScaledShape ---

	@(link_name = "JPH_ScaledShape_Create")
	ScaledShape_Create :: proc(shape: ^Shape, scale: ^Vec3) -> ^ScaledShape ---

	@(link_name = "JPH_ScaledShape_GetScale")
	ScaledShape_GetScale :: proc(shape: ^ScaledShape, result: ^Vec3) ---

	@(link_name = "JPH_OffsetCenterOfMassShapeSettings_Create")
	OffsetCenterOfMassShapeSettings_Create :: proc(offset: ^Vec3, shapeSettings: [^]ShapeSettings) -> ^OffsetCenterOfMassShapeSettings ---

	@(link_name = "JPH_OffsetCenterOfMassShapeSettings_Create2")
	OffsetCenterOfMassShapeSettings_Create2 :: proc(offset: ^Vec3, shape: ^Shape) -> ^OffsetCenterOfMassShapeSettings ---

	@(link_name = "JPH_OffsetCenterOfMassShapeSettings_CreateShape")
	OffsetCenterOfMassShapeSettings_CreateShape :: proc(settings: [^]OffsetCenterOfMassShapeSettings) -> ^OffsetCenterOfMassShape ---

	@(link_name = "JPH_OffsetCenterOfMassShape_Create")
	OffsetCenterOfMassShape_Create :: proc(offset: ^Vec3, shape: ^Shape) -> ^OffsetCenterOfMassShape ---

	@(link_name = "JPH_OffsetCenterOfMassShape_GetOffset")
	OffsetCenterOfMassShape_GetOffset :: proc(shape: ^OffsetCenterOfMassShape, result: ^Vec3) ---

	@(link_name = "JPH_EmptyShapeSettings_Create")
	EmptyShapeSettings_Create :: proc(centerOfMass: [^]Vec3) -> ^EmptyShapeSettings ---

	@(link_name = "JPH_EmptyShapeSettings_CreateShape")
	EmptyShapeSettings_CreateShape :: proc(settings: [^]EmptyShapeSettings) -> ^EmptyShape ---

	@(link_name = "JPH_BodyCreationSettings_Create")
	BodyCreationSettings_Create :: proc() -> ^BodyCreationSettings ---

	@(link_name = "JPH_BodyCreationSettings_Create2")
	BodyCreationSettings_Create2 :: proc(settings: [^]ShapeSettings, position: ^RVec3, rotation: ^Quat, motionType: MotionType, objectLayer: ObjectLayer) -> ^BodyCreationSettings ---

	@(link_name = "JPH_BodyCreationSettings_Create3")
	BodyCreationSettings_Create3 :: proc(shape: ^Shape, position: ^RVec3, rotation: ^Quat, motionType: MotionType, objectLayer: ObjectLayer) -> ^BodyCreationSettings ---

	@(link_name = "JPH_BodyCreationSettings_Destroy")
	BodyCreationSettings_Destroy :: proc(settings: [^]BodyCreationSettings) ---

	@(link_name = "JPH_BodyCreationSettings_GetPosition")
	BodyCreationSettings_GetPosition :: proc(settings: [^]BodyCreationSettings, result: ^RVec3) ---

	@(link_name = "JPH_BodyCreationSettings_SetPosition")
	BodyCreationSettings_SetPosition :: proc(settings: [^]BodyCreationSettings, value: ^RVec3) ---

	@(link_name = "JPH_BodyCreationSettings_GetRotation")
	BodyCreationSettings_GetRotation :: proc(settings: [^]BodyCreationSettings, result: ^Quat) ---

	@(link_name = "JPH_BodyCreationSettings_SetRotation")
	BodyCreationSettings_SetRotation :: proc(settings: [^]BodyCreationSettings, value: ^Quat) ---

	@(link_name = "JPH_BodyCreationSettings_GetLinearVelocity")
	BodyCreationSettings_GetLinearVelocity :: proc(settings: [^]BodyCreationSettings, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyCreationSettings_SetLinearVelocity")
	BodyCreationSettings_SetLinearVelocity :: proc(settings: [^]BodyCreationSettings, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyCreationSettings_GetAngularVelocity")
	BodyCreationSettings_GetAngularVelocity :: proc(settings: [^]BodyCreationSettings, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyCreationSettings_SetAngularVelocity")
	BodyCreationSettings_SetAngularVelocity :: proc(settings: [^]BodyCreationSettings, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyCreationSettings_GetUserData")
	BodyCreationSettings_GetUserData :: proc(settings: [^]BodyCreationSettings) -> u64 ---

	@(link_name = "JPH_BodyCreationSettings_SetUserData")
	BodyCreationSettings_SetUserData :: proc(settings: [^]BodyCreationSettings, value: u64) ---

	@(link_name = "JPH_BodyCreationSettings_GetObjectLayer")
	BodyCreationSettings_GetObjectLayer :: proc(settings: [^]BodyCreationSettings) -> ObjectLayer ---

	@(link_name = "JPH_BodyCreationSettings_SetObjectLayer")
	BodyCreationSettings_SetObjectLayer :: proc(settings: [^]BodyCreationSettings, value: ObjectLayer) ---

	@(link_name = "JPH_BodyCreationSettings_GetMotionType")
	BodyCreationSettings_GetMotionType :: proc(settings: [^]BodyCreationSettings) -> MotionType ---

	@(link_name = "JPH_BodyCreationSettings_SetMotionType")
	BodyCreationSettings_SetMotionType :: proc(settings: [^]BodyCreationSettings, value: MotionType) ---

	@(link_name = "JPH_BodyCreationSettings_GetAllowedDOFs")
	BodyCreationSettings_GetAllowedDOFs :: proc(settings: [^]BodyCreationSettings) -> AllowedDOFs ---

	@(link_name = "JPH_BodyCreationSettings_SetAllowedDOFs")
	BodyCreationSettings_SetAllowedDOFs :: proc(settings: [^]BodyCreationSettings, value: AllowedDOFs) ---

	@(link_name = "JPH_BodyCreationSettings_GetAllowDynamicOrKinematic")
	BodyCreationSettings_GetAllowDynamicOrKinematic :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetAllowDynamicOrKinematic")
	BodyCreationSettings_SetAllowDynamicOrKinematic :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetIsSensor")
	BodyCreationSettings_GetIsSensor :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetIsSensor")
	BodyCreationSettings_SetIsSensor :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetCollideKinematicVsNonDynamic")
	BodyCreationSettings_GetCollideKinematicVsNonDynamic :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetCollideKinematicVsNonDynamic")
	BodyCreationSettings_SetCollideKinematicVsNonDynamic :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetUseManifoldReduction")
	BodyCreationSettings_GetUseManifoldReduction :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetUseManifoldReduction")
	BodyCreationSettings_SetUseManifoldReduction :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetApplyGyroscopicForce")
	BodyCreationSettings_GetApplyGyroscopicForce :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetApplyGyroscopicForce")
	BodyCreationSettings_SetApplyGyroscopicForce :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetMotionQuality")
	BodyCreationSettings_GetMotionQuality :: proc(settings: [^]BodyCreationSettings) -> MotionQuality ---

	@(link_name = "JPH_BodyCreationSettings_SetMotionQuality")
	BodyCreationSettings_SetMotionQuality :: proc(settings: [^]BodyCreationSettings, value: MotionQuality) ---

	@(link_name = "JPH_BodyCreationSettings_GetEnhancedInternalEdgeRemoval")
	BodyCreationSettings_GetEnhancedInternalEdgeRemoval :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetEnhancedInternalEdgeRemoval")
	BodyCreationSettings_SetEnhancedInternalEdgeRemoval :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetAllowSleeping")
	BodyCreationSettings_GetAllowSleeping :: proc(settings: [^]BodyCreationSettings) -> b8 ---

	@(link_name = "JPH_BodyCreationSettings_SetAllowSleeping")
	BodyCreationSettings_SetAllowSleeping :: proc(settings: [^]BodyCreationSettings, value: b8) ---

	@(link_name = "JPH_BodyCreationSettings_GetFriction")
	BodyCreationSettings_GetFriction :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetFriction")
	BodyCreationSettings_SetFriction :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetRestitution")
	BodyCreationSettings_GetRestitution :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetRestitution")
	BodyCreationSettings_SetRestitution :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetLinearDamping")
	BodyCreationSettings_GetLinearDamping :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetLinearDamping")
	BodyCreationSettings_SetLinearDamping :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetAngularDamping")
	BodyCreationSettings_GetAngularDamping :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetAngularDamping")
	BodyCreationSettings_SetAngularDamping :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetMaxLinearVelocity")
	BodyCreationSettings_GetMaxLinearVelocity :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetMaxLinearVelocity")
	BodyCreationSettings_SetMaxLinearVelocity :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetMaxAngularVelocity")
	BodyCreationSettings_GetMaxAngularVelocity :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetMaxAngularVelocity")
	BodyCreationSettings_SetMaxAngularVelocity :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetGravityFactor")
	BodyCreationSettings_GetGravityFactor :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetGravityFactor")
	BodyCreationSettings_SetGravityFactor :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetNumVelocityStepsOverride")
	BodyCreationSettings_GetNumVelocityStepsOverride :: proc(settings: [^]BodyCreationSettings) -> u32 ---

	@(link_name = "JPH_BodyCreationSettings_SetNumVelocityStepsOverride")
	BodyCreationSettings_SetNumVelocityStepsOverride :: proc(settings: [^]BodyCreationSettings, value: u32) ---

	@(link_name = "JPH_BodyCreationSettings_GetNumPositionStepsOverride")
	BodyCreationSettings_GetNumPositionStepsOverride :: proc(settings: [^]BodyCreationSettings) -> u32 ---

	@(link_name = "JPH_BodyCreationSettings_SetNumPositionStepsOverride")
	BodyCreationSettings_SetNumPositionStepsOverride :: proc(settings: [^]BodyCreationSettings, value: u32) ---

	@(link_name = "JPH_BodyCreationSettings_GetOverrideMassProperties")
	BodyCreationSettings_GetOverrideMassProperties :: proc(settings: [^]BodyCreationSettings) -> OverrideMassProperties ---

	@(link_name = "JPH_BodyCreationSettings_SetOverrideMassProperties")
	BodyCreationSettings_SetOverrideMassProperties :: proc(settings: [^]BodyCreationSettings, value: OverrideMassProperties) ---

	@(link_name = "JPH_BodyCreationSettings_GetInertiaMultiplier")
	BodyCreationSettings_GetInertiaMultiplier :: proc(settings: [^]BodyCreationSettings) -> f32 ---

	@(link_name = "JPH_BodyCreationSettings_SetInertiaMultiplier")
	BodyCreationSettings_SetInertiaMultiplier :: proc(settings: [^]BodyCreationSettings, value: f32) ---

	@(link_name = "JPH_BodyCreationSettings_GetMassPropertiesOverride")
	BodyCreationSettings_GetMassPropertiesOverride :: proc(settings: [^]BodyCreationSettings, result: ^MassProperties) ---

	@(link_name = "JPH_BodyCreationSettings_SetMassPropertiesOverride")
	BodyCreationSettings_SetMassPropertiesOverride :: proc(settings: [^]BodyCreationSettings, massProperties: [^]MassProperties) ---

	@(link_name = "JPH_SoftBodyCreationSettings_Create")
	SoftBodyCreationSettings_Create :: proc() -> ^SoftBodyCreationSettings ---

	@(link_name = "JPH_SoftBodyCreationSettings_Destroy")
	SoftBodyCreationSettings_Destroy :: proc(settings: [^]SoftBodyCreationSettings) ---

	@(link_name = "JPH_Constraint_Destroy")
	Constraint_Destroy :: proc(constraint: ^Constraint) ---

	@(link_name = "JPH_Constraint_GetType")
	Constraint_GetType :: proc(constraint: ^Constraint) -> ConstraintType ---

	@(link_name = "JPH_Constraint_GetSubType")
	Constraint_GetSubType :: proc(constraint: ^Constraint) -> ConstraintSubType ---

	@(link_name = "JPH_Constraint_GetConstraintPriority")
	Constraint_GetConstraintPriority :: proc(constraint: ^Constraint) -> u32 ---

	@(link_name = "JPH_Constraint_SetConstraintPriority")
	Constraint_SetConstraintPriority :: proc(constraint: ^Constraint, priority: u32) ---

	@(link_name = "JPH_Constraint_GetNumVelocityStepsOverride")
	Constraint_GetNumVelocityStepsOverride :: proc(constraint: ^Constraint) -> u32 ---

	@(link_name = "JPH_Constraint_SetNumVelocityStepsOverride")
	Constraint_SetNumVelocityStepsOverride :: proc(constraint: ^Constraint, value: u32) ---

	@(link_name = "JPH_Constraint_GetNumPositionStepsOverride")
	Constraint_GetNumPositionStepsOverride :: proc(constraint: ^Constraint) -> u32 ---

	@(link_name = "JPH_Constraint_SetNumPositionStepsOverride")
	Constraint_SetNumPositionStepsOverride :: proc(constraint: ^Constraint, value: u32) ---

	@(link_name = "JPH_Constraint_GetEnabled")
	Constraint_GetEnabled :: proc(constraint: ^Constraint) -> b8 ---

	@(link_name = "JPH_Constraint_SetEnabled")
	Constraint_SetEnabled :: proc(constraint: ^Constraint, enabled: b8) ---

	@(link_name = "JPH_Constraint_GetUserData")
	Constraint_GetUserData :: proc(constraint: ^Constraint) -> u64 ---

	@(link_name = "JPH_Constraint_SetUserData")
	Constraint_SetUserData :: proc(constraint: ^Constraint, userData: u64) ---

	@(link_name = "JPH_Constraint_NotifyShapeChanged")
	Constraint_NotifyShapeChanged :: proc(constraint: ^Constraint, bodyID: BodyID, deltaCOM: ^Vec3) ---

	@(link_name = "JPH_Constraint_ResetWarmStart")
	Constraint_ResetWarmStart :: proc(constraint: ^Constraint) ---

	@(link_name = "JPH_Constraint_IsActive")
	Constraint_IsActive :: proc(constraint: ^Constraint) -> b8 ---

	@(link_name = "JPH_Constraint_SetupVelocityConstraint")
	Constraint_SetupVelocityConstraint :: proc(constraint: ^Constraint, deltaTime: f32) ---

	@(link_name = "JPH_Constraint_WarmStartVelocityConstraint")
	Constraint_WarmStartVelocityConstraint :: proc(constraint: ^Constraint, warmStartImpulseRatio: f32) ---

	@(link_name = "JPH_Constraint_SolveVelocityConstraint")
	Constraint_SolveVelocityConstraint :: proc(constraint: ^Constraint, deltaTime: f32) -> b8 ---

	@(link_name = "JPH_Constraint_SolvePositionConstraint")
	Constraint_SolvePositionConstraint :: proc(constraint: ^Constraint, deltaTime: f32, baumgarte: f32) -> b8 ---

	@(link_name = "JPH_TwoBodyConstraint_GetBody1")
	TwoBodyConstraint_GetBody1 :: proc(constraint: ^TwoBodyConstraint) -> ^Body ---

	@(link_name = "JPH_TwoBodyConstraint_GetBody2")
	TwoBodyConstraint_GetBody2 :: proc(constraint: ^TwoBodyConstraint) -> ^Body ---

	@(link_name = "JPH_TwoBodyConstraint_GetConstraintToBody1Matrix")
	TwoBodyConstraint_GetConstraintToBody1Matrix :: proc(constraint: ^TwoBodyConstraint, result: ^Matrix4x4) ---

	@(link_name = "JPH_TwoBodyConstraint_GetConstraintToBody2Matrix")
	TwoBodyConstraint_GetConstraintToBody2Matrix :: proc(constraint: ^TwoBodyConstraint, result: ^Matrix4x4) ---

	@(link_name = "JPH_FixedConstraintSettings_Init")
	FixedConstraintSettings_Init :: proc(settings: [^]FixedConstraintSettings) ---

	@(link_name = "JPH_FixedConstraint_Create")
	FixedConstraint_Create :: proc(settings: [^]FixedConstraintSettings, body1: ^Body, body2: ^Body) -> ^FixedConstraint ---

	@(link_name = "JPH_FixedConstraint_GetSettings")
	FixedConstraint_GetSettings :: proc(constraint: ^FixedConstraint, settings: [^]FixedConstraintSettings) ---

	@(link_name = "JPH_FixedConstraint_GetTotalLambdaPosition")
	FixedConstraint_GetTotalLambdaPosition :: proc(constraint: ^FixedConstraint, result: ^Vec3) ---

	@(link_name = "JPH_FixedConstraint_GetTotalLambdaRotation")
	FixedConstraint_GetTotalLambdaRotation :: proc(constraint: ^FixedConstraint, result: ^Vec3) ---

	@(link_name = "JPH_DistanceConstraintSettings_Init")
	DistanceConstraintSettings_Init :: proc(settings: [^]DistanceConstraintSettings) ---

	@(link_name = "JPH_DistanceConstraint_Create")
	DistanceConstraint_Create :: proc(settings: [^]DistanceConstraintSettings, body1: ^Body, body2: ^Body) -> ^DistanceConstraint ---

	@(link_name = "JPH_DistanceConstraint_GetSettings")
	DistanceConstraint_GetSettings :: proc(constraint: ^DistanceConstraint, settings: [^]DistanceConstraintSettings) ---

	@(link_name = "JPH_DistanceConstraint_SetDistance")
	DistanceConstraint_SetDistance :: proc(constraint: ^DistanceConstraint, minDistance: f32, maxDistance: f32) ---

	@(link_name = "JPH_DistanceConstraint_GetMinDistance")
	DistanceConstraint_GetMinDistance :: proc(constraint: ^DistanceConstraint) -> f32 ---

	@(link_name = "JPH_DistanceConstraint_GetMaxDistance")
	DistanceConstraint_GetMaxDistance :: proc(constraint: ^DistanceConstraint) -> f32 ---

	@(link_name = "JPH_DistanceConstraint_GetLimitsSpringSettings")
	DistanceConstraint_GetLimitsSpringSettings :: proc(constraint: ^DistanceConstraint, result: ^SpringSettings) ---

	@(link_name = "JPH_DistanceConstraint_SetLimitsSpringSettings")
	DistanceConstraint_SetLimitsSpringSettings :: proc(constraint: ^DistanceConstraint, settings: [^]SpringSettings) ---

	@(link_name = "JPH_DistanceConstraint_GetTotalLambdaPosition")
	DistanceConstraint_GetTotalLambdaPosition :: proc(constraint: ^DistanceConstraint) -> f32 ---

	@(link_name = "JPH_PointConstraintSettings_Init")
	PointConstraintSettings_Init :: proc(settings: [^]PointConstraintSettings) ---

	@(link_name = "JPH_PointConstraint_Create")
	PointConstraint_Create :: proc(settings: [^]PointConstraintSettings, body1: ^Body, body2: ^Body) -> ^PointConstraint ---

	@(link_name = "JPH_PointConstraint_GetSettings")
	PointConstraint_GetSettings :: proc(constraint: ^PointConstraint, settings: [^]PointConstraintSettings) ---

	@(link_name = "JPH_PointConstraint_SetPoint1")
	PointConstraint_SetPoint1 :: proc(constraint: ^PointConstraint, space: ConstraintSpace, value: ^RVec3) ---

	@(link_name = "JPH_PointConstraint_SetPoint2")
	PointConstraint_SetPoint2 :: proc(constraint: ^PointConstraint, space: ConstraintSpace, value: ^RVec3) ---

	@(link_name = "JPH_PointConstraint_GetLocalSpacePoint1")
	PointConstraint_GetLocalSpacePoint1 :: proc(constraint: ^PointConstraint, result: ^Vec3) ---

	@(link_name = "JPH_PointConstraint_GetLocalSpacePoint2")
	PointConstraint_GetLocalSpacePoint2 :: proc(constraint: ^PointConstraint, result: ^Vec3) ---

	@(link_name = "JPH_PointConstraint_GetTotalLambdaPosition")
	PointConstraint_GetTotalLambdaPosition :: proc(constraint: ^PointConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraintSettings_Init")
	HingeConstraintSettings_Init :: proc(settings: [^]HingeConstraintSettings) ---

	@(link_name = "JPH_HingeConstraint_Create")
	HingeConstraint_Create :: proc(settings: [^]HingeConstraintSettings, body1: ^Body, body2: ^Body) -> ^HingeConstraint ---

	@(link_name = "JPH_HingeConstraint_GetSettings")
	HingeConstraint_GetSettings :: proc(constraint: ^HingeConstraint, settings: [^]HingeConstraintSettings) ---

	@(link_name = "JPH_HingeConstraint_GetLocalSpacePoint1")
	HingeConstraint_GetLocalSpacePoint1 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetLocalSpacePoint2")
	HingeConstraint_GetLocalSpacePoint2 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetLocalSpaceHingeAxis1")
	HingeConstraint_GetLocalSpaceHingeAxis1 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetLocalSpaceHingeAxis2")
	HingeConstraint_GetLocalSpaceHingeAxis2 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetLocalSpaceNormalAxis1")
	HingeConstraint_GetLocalSpaceNormalAxis1 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetLocalSpaceNormalAxis2")
	HingeConstraint_GetLocalSpaceNormalAxis2 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetCurrentAngle")
	HingeConstraint_GetCurrentAngle :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_SetMaxFrictionTorque")
	HingeConstraint_SetMaxFrictionTorque :: proc(constraint: ^HingeConstraint, frictionTorque: f32) ---

	@(link_name = "JPH_HingeConstraint_GetMaxFrictionTorque")
	HingeConstraint_GetMaxFrictionTorque :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_SetMotorSettings")
	HingeConstraint_SetMotorSettings :: proc(constraint: ^HingeConstraint, settings: [^]MotorSettings) ---

	@(link_name = "JPH_HingeConstraint_GetMotorSettings")
	HingeConstraint_GetMotorSettings :: proc(constraint: ^HingeConstraint, result: ^MotorSettings) ---

	@(link_name = "JPH_HingeConstraint_SetMotorState")
	HingeConstraint_SetMotorState :: proc(constraint: ^HingeConstraint, state: MotorState) ---

	@(link_name = "JPH_HingeConstraint_GetMotorState")
	HingeConstraint_GetMotorState :: proc(constraint: ^HingeConstraint) -> MotorState ---

	@(link_name = "JPH_HingeConstraint_SetTargetAngularVelocity")
	HingeConstraint_SetTargetAngularVelocity :: proc(constraint: ^HingeConstraint, angularVelocity: f32) ---

	@(link_name = "JPH_HingeConstraint_GetTargetAngularVelocity")
	HingeConstraint_GetTargetAngularVelocity :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_SetTargetAngle")
	HingeConstraint_SetTargetAngle :: proc(constraint: ^HingeConstraint, angle: f32) ---

	@(link_name = "JPH_HingeConstraint_GetTargetAngle")
	HingeConstraint_GetTargetAngle :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_SetLimits")
	HingeConstraint_SetLimits :: proc(constraint: ^HingeConstraint, inLimitsMin: f32, inLimitsMax: f32) ---

	@(link_name = "JPH_HingeConstraint_GetLimitsMin")
	HingeConstraint_GetLimitsMin :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_GetLimitsMax")
	HingeConstraint_GetLimitsMax :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_HasLimits")
	HingeConstraint_HasLimits :: proc(constraint: ^HingeConstraint) -> b8 ---

	@(link_name = "JPH_HingeConstraint_GetLimitsSpringSettings")
	HingeConstraint_GetLimitsSpringSettings :: proc(constraint: ^HingeConstraint, result: ^SpringSettings) ---

	@(link_name = "JPH_HingeConstraint_SetLimitsSpringSettings")
	HingeConstraint_SetLimitsSpringSettings :: proc(constraint: ^HingeConstraint, settings: [^]SpringSettings) ---

	@(link_name = "JPH_HingeConstraint_GetTotalLambdaPosition")
	HingeConstraint_GetTotalLambdaPosition :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_HingeConstraint_GetTotalLambdaRotation")
	HingeConstraint_GetTotalLambdaRotation :: proc(constraint: ^HingeConstraint, rotation: [2]f32) ---

	@(link_name = "JPH_HingeConstraint_GetTotalLambdaRotationLimits")
	HingeConstraint_GetTotalLambdaRotationLimits :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_HingeConstraint_GetTotalLambdaMotor")
	HingeConstraint_GetTotalLambdaMotor :: proc(constraint: ^HingeConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraintSettings_Init")
	SliderConstraintSettings_Init :: proc(settings: [^]SliderConstraintSettings) ---

	@(link_name = "JPH_SliderConstraintSettings_SetSliderAxis")
	SliderConstraintSettings_SetSliderAxis :: proc(settings: [^]SliderConstraintSettings, axis: [^]Vec3) ---

	@(link_name = "JPH_SliderConstraint_Create")
	SliderConstraint_Create :: proc(settings: [^]SliderConstraintSettings, body1: ^Body, body2: ^Body) -> ^SliderConstraint ---

	@(link_name = "JPH_SliderConstraint_GetSettings")
	SliderConstraint_GetSettings :: proc(constraint: ^SliderConstraint, settings: [^]SliderConstraintSettings) ---

	@(link_name = "JPH_SliderConstraint_GetCurrentPosition")
	SliderConstraint_GetCurrentPosition :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_SetMaxFrictionForce")
	SliderConstraint_SetMaxFrictionForce :: proc(constraint: ^SliderConstraint, frictionForce: f32) ---

	@(link_name = "JPH_SliderConstraint_GetMaxFrictionForce")
	SliderConstraint_GetMaxFrictionForce :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_SetMotorSettings")
	SliderConstraint_SetMotorSettings :: proc(constraint: ^SliderConstraint, settings: [^]MotorSettings) ---

	@(link_name = "JPH_SliderConstraint_GetMotorSettings")
	SliderConstraint_GetMotorSettings :: proc(constraint: ^SliderConstraint, result: ^MotorSettings) ---

	@(link_name = "JPH_SliderConstraint_SetMotorState")
	SliderConstraint_SetMotorState :: proc(constraint: ^SliderConstraint, state: MotorState) ---

	@(link_name = "JPH_SliderConstraint_GetMotorState")
	SliderConstraint_GetMotorState :: proc(constraint: ^SliderConstraint) -> MotorState ---

	@(link_name = "JPH_SliderConstraint_SetTargetVelocity")
	SliderConstraint_SetTargetVelocity :: proc(constraint: ^SliderConstraint, velocity: f32) ---

	@(link_name = "JPH_SliderConstraint_GetTargetVelocity")
	SliderConstraint_GetTargetVelocity :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_SetTargetPosition")
	SliderConstraint_SetTargetPosition :: proc(constraint: ^SliderConstraint, position: f32) ---

	@(link_name = "JPH_SliderConstraint_GetTargetPosition")
	SliderConstraint_GetTargetPosition :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_SetLimits")
	SliderConstraint_SetLimits :: proc(constraint: ^SliderConstraint, inLimitsMin: f32, inLimitsMax: f32) ---

	@(link_name = "JPH_SliderConstraint_GetLimitsMin")
	SliderConstraint_GetLimitsMin :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_GetLimitsMax")
	SliderConstraint_GetLimitsMax :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_HasLimits")
	SliderConstraint_HasLimits :: proc(constraint: ^SliderConstraint) -> b8 ---

	@(link_name = "JPH_SliderConstraint_GetLimitsSpringSettings")
	SliderConstraint_GetLimitsSpringSettings :: proc(constraint: ^SliderConstraint, result: ^SpringSettings) ---

	@(link_name = "JPH_SliderConstraint_SetLimitsSpringSettings")
	SliderConstraint_SetLimitsSpringSettings :: proc(constraint: ^SliderConstraint, settings: [^]SpringSettings) ---

	@(link_name = "JPH_SliderConstraint_GetTotalLambdaPosition")
	SliderConstraint_GetTotalLambdaPosition :: proc(constraint: ^SliderConstraint, position: [2]f32) ---

	@(link_name = "JPH_SliderConstraint_GetTotalLambdaPositionLimits")
	SliderConstraint_GetTotalLambdaPositionLimits :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_SliderConstraint_GetTotalLambdaRotation")
	SliderConstraint_GetTotalLambdaRotation :: proc(constraint: ^SliderConstraint, result: ^Vec3) ---

	@(link_name = "JPH_SliderConstraint_GetTotalLambdaMotor")
	SliderConstraint_GetTotalLambdaMotor :: proc(constraint: ^SliderConstraint) -> f32 ---

	@(link_name = "JPH_ConeConstraintSettings_Init")
	ConeConstraintSettings_Init :: proc(settings: [^]ConeConstraintSettings) ---

	@(link_name = "JPH_ConeConstraint_Create")
	ConeConstraint_Create :: proc(settings: [^]ConeConstraintSettings, body1: ^Body, body2: ^Body) -> ^ConeConstraint ---

	@(link_name = "JPH_ConeConstraint_GetSettings")
	ConeConstraint_GetSettings :: proc(constraint: ^ConeConstraint, settings: [^]ConeConstraintSettings) ---

	@(link_name = "JPH_ConeConstraint_SetHalfConeAngle")
	ConeConstraint_SetHalfConeAngle :: proc(constraint: ^ConeConstraint, halfConeAngle: f32) ---

	@(link_name = "JPH_ConeConstraint_GetCosHalfConeAngle")
	ConeConstraint_GetCosHalfConeAngle :: proc(constraint: ^ConeConstraint) -> f32 ---

	@(link_name = "JPH_ConeConstraint_GetTotalLambdaPosition")
	ConeConstraint_GetTotalLambdaPosition :: proc(constraint: ^ConeConstraint, result: ^Vec3) ---

	@(link_name = "JPH_ConeConstraint_GetTotalLambdaRotation")
	ConeConstraint_GetTotalLambdaRotation :: proc(constraint: ^ConeConstraint) -> f32 ---

	@(link_name = "JPH_SwingTwistConstraintSettings_Init")
	SwingTwistConstraintSettings_Init :: proc(settings: [^]SwingTwistConstraintSettings) ---

	@(link_name = "JPH_SwingTwistConstraint_Create")
	SwingTwistConstraint_Create :: proc(settings: [^]SwingTwistConstraintSettings, body1: ^Body, body2: ^Body) -> ^SwingTwistConstraint ---

	@(link_name = "JPH_SwingTwistConstraint_GetSettings")
	SwingTwistConstraint_GetSettings :: proc(constraint: ^SwingTwistConstraint, settings: [^]SwingTwistConstraintSettings) ---

	@(link_name = "JPH_SwingTwistConstraint_GetNormalHalfConeAngle")
	SwingTwistConstraint_GetNormalHalfConeAngle :: proc(constraint: ^SwingTwistConstraint) -> f32 ---

	@(link_name = "JPH_SwingTwistConstraint_GetTotalLambdaPosition")
	SwingTwistConstraint_GetTotalLambdaPosition :: proc(constraint: ^SwingTwistConstraint, result: ^Vec3) ---

	@(link_name = "JPH_SwingTwistConstraint_GetTotalLambdaTwist")
	SwingTwistConstraint_GetTotalLambdaTwist :: proc(constraint: ^SwingTwistConstraint) -> f32 ---

	@(link_name = "JPH_SwingTwistConstraint_GetTotalLambdaSwingY")
	SwingTwistConstraint_GetTotalLambdaSwingY :: proc(constraint: ^SwingTwistConstraint) -> f32 ---

	@(link_name = "JPH_SwingTwistConstraint_GetTotalLambdaSwingZ")
	SwingTwistConstraint_GetTotalLambdaSwingZ :: proc(constraint: ^SwingTwistConstraint) -> f32 ---

	@(link_name = "JPH_SwingTwistConstraint_GetTotalLambdaMotor")
	SwingTwistConstraint_GetTotalLambdaMotor :: proc(constraint: ^SwingTwistConstraint, result: ^Vec3) ---

	@(link_name = "JPH_SixDOFConstraintSettings_Init")
	SixDOFConstraintSettings_Init :: proc(settings: [^]SixDOFConstraintSettings) ---

	@(link_name = "JPH_SixDOFConstraintSettings_MakeFreeAxis")
	SixDOFConstraintSettings_MakeFreeAxis :: proc(settings: [^]SixDOFConstraintSettings, axis: SixDOFConstraintAxis) ---

	@(link_name = "JPH_SixDOFConstraintSettings_IsFreeAxis")
	SixDOFConstraintSettings_IsFreeAxis :: proc(settings: [^]SixDOFConstraintSettings, axis: SixDOFConstraintAxis) -> b8 ---

	@(link_name = "JPH_SixDOFConstraintSettings_MakeFixedAxis")
	SixDOFConstraintSettings_MakeFixedAxis :: proc(settings: [^]SixDOFConstraintSettings, axis: SixDOFConstraintAxis) ---

	@(link_name = "JPH_SixDOFConstraintSettings_IsFixedAxis")
	SixDOFConstraintSettings_IsFixedAxis :: proc(settings: [^]SixDOFConstraintSettings, axis: SixDOFConstraintAxis) -> b8 ---

	@(link_name = "JPH_SixDOFConstraintSettings_SetLimitedAxis")
	SixDOFConstraintSettings_SetLimitedAxis :: proc(settings: [^]SixDOFConstraintSettings, axis: SixDOFConstraintAxis, min: f32, max: f32) ---

	@(link_name = "JPH_SixDOFConstraint_Create")
	SixDOFConstraint_Create :: proc(settings: [^]SixDOFConstraintSettings, body1: ^Body, body2: ^Body) -> ^SixDOFConstraint ---

	@(link_name = "JPH_SixDOFConstraint_GetSettings")
	SixDOFConstraint_GetSettings :: proc(constraint: ^SixDOFConstraint, settings: [^]SixDOFConstraintSettings) ---

	@(link_name = "JPH_SixDOFConstraint_GetLimitsMin")
	SixDOFConstraint_GetLimitsMin :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> f32 ---

	@(link_name = "JPH_SixDOFConstraint_GetLimitsMax")
	SixDOFConstraint_GetLimitsMax :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> f32 ---

	@(link_name = "JPH_SixDOFConstraint_GetTotalLambdaPosition")
	SixDOFConstraint_GetTotalLambdaPosition :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---

	@(link_name = "JPH_SixDOFConstraint_GetTotalLambdaRotation")
	SixDOFConstraint_GetTotalLambdaRotation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---

	@(link_name = "JPH_SixDOFConstraint_GetTotalLambdaMotorTranslation")
	SixDOFConstraint_GetTotalLambdaMotorTranslation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---

	@(link_name = "JPH_SixDOFConstraint_GetTotalLambdaMotorRotation")
	SixDOFConstraint_GetTotalLambdaMotorRotation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---

	@(link_name = "JPH_GearConstraintSettings_Init")
	GearConstraintSettings_Init :: proc(settings: [^]GearConstraintSettings) ---

	@(link_name = "JPH_GearConstraint_Create")
	GearConstraint_Create :: proc(settings: [^]GearConstraintSettings, body1: ^Body, body2: ^Body) -> ^GearConstraint ---

	@(link_name = "JPH_GearConstraint_GetSettings")
	GearConstraint_GetSettings :: proc(constraint: ^GearConstraint, settings: [^]GearConstraintSettings) ---

	@(link_name = "JPH_GearConstraint_SetConstraints")
	GearConstraint_SetConstraints :: proc(constraint: ^GearConstraint, gear1: ^Constraint, gear2: ^Constraint) ---

	@(link_name = "JPH_GearConstraint_GetTotalLambda")
	GearConstraint_GetTotalLambda :: proc(constraint: ^GearConstraint) -> f32 ---

	@(link_name = "JPH_BodyInterface_DestroyBody")
	BodyInterface_DestroyBody :: proc(interface: ^BodyInterface, bodyID: BodyID) ---

	@(link_name = "JPH_BodyInterface_CreateAndAddBody")
	BodyInterface_CreateAndAddBody :: proc(interface: ^BodyInterface, settings: [^]BodyCreationSettings, activationMode: Activation) -> BodyID ---

	@(link_name = "JPH_BodyInterface_CreateBody")
	BodyInterface_CreateBody :: proc(interface: ^BodyInterface, settings: [^]BodyCreationSettings) -> ^Body ---

	@(link_name = "JPH_BodyInterface_CreateBodyWithID")
	BodyInterface_CreateBodyWithID :: proc(interface: ^BodyInterface, bodyID: BodyID, settings: [^]BodyCreationSettings) -> ^Body ---

	@(link_name = "JPH_BodyInterface_CreateBodyWithoutID")
	BodyInterface_CreateBodyWithoutID :: proc(interface: ^BodyInterface, settings: [^]BodyCreationSettings) -> ^Body ---

	@(link_name = "JPH_BodyInterface_DestroyBodyWithoutID")
	BodyInterface_DestroyBodyWithoutID :: proc(interface: ^BodyInterface, body: ^Body) ---

	@(link_name = "JPH_BodyInterface_AssignBodyID")
	BodyInterface_AssignBodyID :: proc(interface: ^BodyInterface, body: ^Body) -> b8 ---

	@(link_name = "JPH_BodyInterface_AssignBodyID2")
	BodyInterface_AssignBodyID2 :: proc(interface: ^BodyInterface, body: ^Body, bodyID: BodyID) -> b8 ---

	@(link_name = "JPH_BodyInterface_UnassignBodyID")
	BodyInterface_UnassignBodyID :: proc(interface: ^BodyInterface, bodyID: BodyID) -> ^Body ---

	@(link_name = "JPH_BodyInterface_CreateSoftBody")
	BodyInterface_CreateSoftBody :: proc(interface: ^BodyInterface, settings: [^]SoftBodyCreationSettings) -> ^Body ---

	@(link_name = "JPH_BodyInterface_CreateSoftBodyWithID")
	BodyInterface_CreateSoftBodyWithID :: proc(interface: ^BodyInterface, bodyID: BodyID, settings: [^]SoftBodyCreationSettings) -> ^Body ---

	@(link_name = "JPH_BodyInterface_CreateSoftBodyWithoutID")
	BodyInterface_CreateSoftBodyWithoutID :: proc(interface: ^BodyInterface, settings: [^]SoftBodyCreationSettings) -> ^Body ---

	@(link_name = "JPH_BodyInterface_CreateAndAddSoftBody")
	BodyInterface_CreateAndAddSoftBody :: proc(interface: ^BodyInterface, settings: [^]SoftBodyCreationSettings, activationMode: Activation) -> BodyID ---

	@(link_name = "JPH_BodyInterface_AddBody")
	BodyInterface_AddBody :: proc(interface: ^BodyInterface, bodyID: BodyID, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_RemoveBody")
	BodyInterface_RemoveBody :: proc(interface: ^BodyInterface, bodyID: BodyID) ---

	@(link_name = "JPH_BodyInterface_RemoveAndDestroyBody")
	BodyInterface_RemoveAndDestroyBody :: proc(interface: ^BodyInterface, bodyID: BodyID) ---

	@(link_name = "JPH_BodyInterface_IsActive")
	BodyInterface_IsActive :: proc(interface: ^BodyInterface, bodyID: BodyID) -> b8 ---

	@(link_name = "JPH_BodyInterface_IsAdded")
	BodyInterface_IsAdded :: proc(interface: ^BodyInterface, bodyID: BodyID) -> b8 ---

	@(link_name = "JPH_BodyInterface_GetBodyType")
	BodyInterface_GetBodyType :: proc(interface: ^BodyInterface, bodyID: BodyID) -> BodyType ---

	@(link_name = "JPH_BodyInterface_SetLinearVelocity")
	BodyInterface_SetLinearVelocity :: proc(interface: ^BodyInterface, bodyID: BodyID, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_GetLinearVelocity")
	BodyInterface_GetLinearVelocity :: proc(interface: ^BodyInterface, bodyID: BodyID, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_GetCenterOfMassPosition")
	BodyInterface_GetCenterOfMassPosition :: proc(interface: ^BodyInterface, bodyID: BodyID, position: ^RVec3) ---

	@(link_name = "JPH_BodyInterface_GetMotionType")
	BodyInterface_GetMotionType :: proc(interface: ^BodyInterface, bodyID: BodyID) -> MotionType ---

	@(link_name = "JPH_BodyInterface_SetMotionType")
	BodyInterface_SetMotionType :: proc(interface: ^BodyInterface, bodyID: BodyID, motionType: MotionType, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_GetRestitution")
	BodyInterface_GetRestitution :: proc(interface: ^BodyInterface, bodyID: BodyID) -> f32 ---

	@(link_name = "JPH_BodyInterface_SetRestitution")
	BodyInterface_SetRestitution :: proc(interface: ^BodyInterface, bodyID: BodyID, restitution: f32) ---

	@(link_name = "JPH_BodyInterface_GetFriction")
	BodyInterface_GetFriction :: proc(interface: ^BodyInterface, bodyID: BodyID) -> f32 ---

	@(link_name = "JPH_BodyInterface_SetFriction")
	BodyInterface_SetFriction :: proc(interface: ^BodyInterface, bodyID: BodyID, friction: f32) ---

	@(link_name = "JPH_BodyInterface_SetPosition")
	BodyInterface_SetPosition :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_GetPosition")
	BodyInterface_GetPosition :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RVec3) ---

	@(link_name = "JPH_BodyInterface_SetRotation")
	BodyInterface_SetRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, rotation: ^Quat, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_GetRotation")
	BodyInterface_GetRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^Quat) ---

	@(link_name = "JPH_BodyInterface_SetPositionAndRotation")
	BodyInterface_SetPositionAndRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_SetPositionAndRotationWhenChanged")
	BodyInterface_SetPositionAndRotationWhenChanged :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_GetPositionAndRotation")
	BodyInterface_GetPositionAndRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat) ---

	@(link_name = "JPH_BodyInterface_SetPositionRotationAndVelocity")
	BodyInterface_SetPositionRotationAndVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_GetShape")
	BodyInterface_GetShape :: proc(interface: ^BodyInterface, bodyId: BodyID) -> ^Shape ---

	@(link_name = "JPH_BodyInterface_SetShape")
	BodyInterface_SetShape :: proc(interface: ^BodyInterface, bodyId: BodyID, shape: ^Shape, updateMassProperties: b8, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_NotifyShapeChanged")
	BodyInterface_NotifyShapeChanged :: proc(interface: ^BodyInterface, bodyId: BodyID, previousCenterOfMass: [^]Vec3, updateMassProperties: b8, activationMode: Activation) ---

	@(link_name = "JPH_BodyInterface_ActivateBody")
	BodyInterface_ActivateBody :: proc(interface: ^BodyInterface, bodyId: BodyID) ---

	@(link_name = "JPH_BodyInterface_DeactivateBody")
	BodyInterface_DeactivateBody :: proc(interface: ^BodyInterface, bodyId: BodyID) ---

	@(link_name = "JPH_BodyInterface_GetObjectLayer")
	BodyInterface_GetObjectLayer :: proc(interface: ^BodyInterface, bodyId: BodyID) -> ObjectLayer ---

	@(link_name = "JPH_BodyInterface_SetObjectLayer")
	BodyInterface_SetObjectLayer :: proc(interface: ^BodyInterface, bodyId: BodyID, layer: ObjectLayer) ---

	@(link_name = "JPH_BodyInterface_GetWorldTransform")
	BodyInterface_GetWorldTransform :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RMatrix4x4) ---

	@(link_name = "JPH_BodyInterface_GetCenterOfMassTransform")
	BodyInterface_GetCenterOfMassTransform :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RMatrix4x4) ---

	@(link_name = "JPH_BodyInterface_MoveKinematic")
	BodyInterface_MoveKinematic :: proc(interface: ^BodyInterface, bodyId: BodyID, targetPosition: ^RVec3, targetRotation: ^Quat, deltaTime: f32) ---

	@(link_name = "JPH_BodyInterface_ApplyBuoyancyImpulse")
	BodyInterface_ApplyBuoyancyImpulse :: proc(interface: ^BodyInterface, bodyId: BodyID, surfacePosition: ^RVec3, surfaceNormal: ^Vec3, buoyancy: f32, linearDrag: f32, angularDrag: f32, fluidVelocity: ^Vec3, gravity: ^Vec3, deltaTime: f32) -> b8 ---

	@(link_name = "JPH_BodyInterface_SetLinearAndAngularVelocity")
	BodyInterface_SetLinearAndAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_GetLinearAndAngularVelocity")
	BodyInterface_GetLinearAndAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddLinearVelocity")
	BodyInterface_AddLinearVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddLinearAndAngularVelocity")
	BodyInterface_AddLinearAndAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_SetAngularVelocity")
	BodyInterface_SetAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, angularVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_GetAngularVelocity")
	BodyInterface_GetAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, angularVelocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_GetPointVelocity")
	BodyInterface_GetPointVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, point: ^RVec3, velocity: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddForce")
	BodyInterface_AddForce :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddForce2")
	BodyInterface_AddForce2 :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3, point: ^RVec3) ---

	@(link_name = "JPH_BodyInterface_AddTorque")
	BodyInterface_AddTorque :: proc(interface: ^BodyInterface, bodyId: BodyID, torque: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddForceAndTorque")
	BodyInterface_AddForceAndTorque :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3, torque: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddImpulse")
	BodyInterface_AddImpulse :: proc(interface: ^BodyInterface, bodyId: BodyID, impulse: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_AddImpulse2")
	BodyInterface_AddImpulse2 :: proc(interface: ^BodyInterface, bodyId: BodyID, impulse: ^Vec3, point: ^RVec3) ---

	@(link_name = "JPH_BodyInterface_AddAngularImpulse")
	BodyInterface_AddAngularImpulse :: proc(interface: ^BodyInterface, bodyId: BodyID, angularImpulse: ^Vec3) ---

	@(link_name = "JPH_BodyInterface_SetMotionQuality")
	BodyInterface_SetMotionQuality :: proc(interface: ^BodyInterface, bodyId: BodyID, quality: MotionQuality) ---

	@(link_name = "JPH_BodyInterface_GetMotionQuality")
	BodyInterface_GetMotionQuality :: proc(interface: ^BodyInterface, bodyId: BodyID) -> MotionQuality ---

	@(link_name = "JPH_BodyInterface_GetInverseInertia")
	BodyInterface_GetInverseInertia :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^Matrix4x4) ---

	@(link_name = "JPH_BodyInterface_SetGravityFactor")
	BodyInterface_SetGravityFactor :: proc(interface: ^BodyInterface, bodyId: BodyID, value: f32) ---

	@(link_name = "JPH_BodyInterface_GetGravityFactor")
	BodyInterface_GetGravityFactor :: proc(interface: ^BodyInterface, bodyId: BodyID) -> f32 ---

	@(link_name = "JPH_BodyInterface_SetUseManifoldReduction")
	BodyInterface_SetUseManifoldReduction :: proc(interface: ^BodyInterface, bodyId: BodyID, value: b8) ---

	@(link_name = "JPH_BodyInterface_GetUseManifoldReduction")
	BodyInterface_GetUseManifoldReduction :: proc(interface: ^BodyInterface, bodyId: BodyID) -> b8 ---

	@(link_name = "JPH_BodyInterface_SetUserData")
	BodyInterface_SetUserData :: proc(interface: ^BodyInterface, bodyId: BodyID, inUserData: u64) ---

	@(link_name = "JPH_BodyInterface_GetUserData")
	BodyInterface_GetUserData :: proc(interface: ^BodyInterface, bodyId: BodyID) -> u64 ---

	@(link_name = "JPH_BodyInterface_GetMaterial")
	BodyInterface_GetMaterial :: proc(interface: ^BodyInterface, bodyId: BodyID, subShapeID: SubShapeID) -> ^PhysicsMaterial ---

	@(link_name = "JPH_BodyInterface_InvalidateContactCache")
	BodyInterface_InvalidateContactCache :: proc(interface: ^BodyInterface, bodyId: BodyID) ---

	@(link_name = "JPH_BodyLockInterface_LockRead")
	BodyLockInterface_LockRead :: proc(lockInterface: ^BodyLockInterface, bodyID: BodyID, outLock: ^BodyLockRead) ---

	@(link_name = "JPH_BodyLockInterface_UnlockRead")
	BodyLockInterface_UnlockRead :: proc(lockInterface: ^BodyLockInterface, ioLock: ^BodyLockRead) ---

	@(link_name = "JPH_BodyLockInterface_LockWrite")
	BodyLockInterface_LockWrite :: proc(lockInterface: ^BodyLockInterface, bodyID: BodyID, outLock: ^BodyLockWrite) ---

	@(link_name = "JPH_BodyLockInterface_UnlockWrite")
	BodyLockInterface_UnlockWrite :: proc(lockInterface: ^BodyLockInterface, ioLock: ^BodyLockWrite) ---

	@(link_name = "JPH_BodyLockInterface_LockMultiRead")
	BodyLockInterface_LockMultiRead :: proc(lockInterface: ^BodyLockInterface, bodyIDs: [^]BodyID, count: u32) -> ^BodyLockMultiRead ---

	@(link_name = "JPH_BodyLockMultiRead_Destroy")
	BodyLockMultiRead_Destroy :: proc(ioLock: ^BodyLockMultiRead) ---

	@(link_name = "JPH_BodyLockMultiRead_GetBody")
	BodyLockMultiRead_GetBody :: proc(ioLock: ^BodyLockMultiRead, bodyIndex: u32) -> ^Body ---

	@(link_name = "JPH_BodyLockInterface_LockMultiWrite")
	BodyLockInterface_LockMultiWrite :: proc(lockInterface: ^BodyLockInterface, bodyIDs: [^]BodyID, count: u32) -> ^BodyLockMultiWrite ---

	@(link_name = "JPH_BodyLockMultiWrite_Destroy")
	BodyLockMultiWrite_Destroy :: proc(ioLock: ^BodyLockMultiWrite) ---

	@(link_name = "JPH_BodyLockMultiWrite_GetBody")
	BodyLockMultiWrite_GetBody :: proc(ioLock: ^BodyLockMultiWrite, bodyIndex: u32) -> ^Body ---

	@(link_name = "JPH_MotionProperties_GetAllowedDOFs")
	MotionProperties_GetAllowedDOFs :: proc(properties: [^]MotionProperties) -> AllowedDOFs ---

	@(link_name = "JPH_MotionProperties_SetLinearDamping")
	MotionProperties_SetLinearDamping :: proc(properties: [^]MotionProperties, damping: f32) ---

	@(link_name = "JPH_MotionProperties_GetLinearDamping")
	MotionProperties_GetLinearDamping :: proc(properties: [^]MotionProperties) -> f32 ---

	@(link_name = "JPH_MotionProperties_SetAngularDamping")
	MotionProperties_SetAngularDamping :: proc(properties: [^]MotionProperties, damping: f32) ---

	@(link_name = "JPH_MotionProperties_GetAngularDamping")
	MotionProperties_GetAngularDamping :: proc(properties: [^]MotionProperties) -> f32 ---

	@(link_name = "JPH_MotionProperties_SetMassProperties")
	MotionProperties_SetMassProperties :: proc(properties: [^]MotionProperties, allowedDOFs: AllowedDOFs, massProperties: [^]MassProperties) ---

	@(link_name = "JPH_MotionProperties_GetInverseMassUnchecked")
	MotionProperties_GetInverseMassUnchecked :: proc(properties: [^]MotionProperties) -> f32 ---

	@(link_name = "JPH_MotionProperties_SetInverseMass")
	MotionProperties_SetInverseMass :: proc(properties: [^]MotionProperties, inverseMass: f32) ---

	@(link_name = "JPH_MotionProperties_GetInverseInertiaDiagonal")
	MotionProperties_GetInverseInertiaDiagonal :: proc(properties: [^]MotionProperties, result: ^Vec3) ---

	@(link_name = "JPH_MotionProperties_GetInertiaRotation")
	MotionProperties_GetInertiaRotation :: proc(properties: [^]MotionProperties, result: ^Quat) ---

	@(link_name = "JPH_MotionProperties_SetInverseInertia")
	MotionProperties_SetInverseInertia :: proc(properties: [^]MotionProperties, diagonal: ^Vec3, rot: ^Quat) ---

	@(link_name = "JPH_MotionProperties_ScaleToMass")
	MotionProperties_ScaleToMass :: proc(properties: [^]MotionProperties, mass: f32) ---

	@(link_name = "JPH_RayCast_GetPointOnRay")
	RayCast_GetPointOnRay :: proc(origin: ^Vec3, direction: ^Vec3, fraction: f32, result: ^Vec3) ---

	@(link_name = "JPH_RRayCast_GetPointOnRay")
	RRayCast_GetPointOnRay :: proc(origin: ^RVec3, direction: ^Vec3, fraction: f32, result: ^RVec3) ---

	@(link_name = "JPH_MassProperties_DecomposePrincipalMomentsOfInertia")
	MassProperties_DecomposePrincipalMomentsOfInertia :: proc(properties: [^]MassProperties, rotation: ^Matrix4x4, diagonal: ^Vec3) ---

	@(link_name = "JPH_MassProperties_ScaleToMass")
	MassProperties_ScaleToMass :: proc(properties: [^]MassProperties, mass: f32) ---

	@(link_name = "JPH_MassProperties_GetEquivalentSolidBoxSize")
	MassProperties_GetEquivalentSolidBoxSize :: proc(mass: f32, inertiaDiagonal: ^Vec3, result: ^Vec3) ---

	@(link_name = "JPH_CollideShapeSettings_Init")
	CollideShapeSettings_Init :: proc(settings: [^]CollideShapeSettings) ---

	@(link_name = "JPH_ShapeCastSettings_Init")
	ShapeCastSettings_Init :: proc(settings: [^]ShapeCastSettings) ---

	@(link_name = "JPH_BroadPhaseQuery_CastRay")
	BroadPhaseQuery_CastRay :: proc(query: ^BroadPhaseQuery, origin: ^Vec3, direction: ^Vec3, callback: ^RayCastBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> b8 ---

	@(link_name = "JPH_BroadPhaseQuery_CastRay2")
	BroadPhaseQuery_CastRay2 :: proc(query: ^BroadPhaseQuery, origin: ^Vec3, direction: ^Vec3, collectorType: CollisionCollectorType, callback: ^RayCastBodyResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> b8 ---

	@(link_name = "JPH_BroadPhaseQuery_CollideAABox")
	BroadPhaseQuery_CollideAABox :: proc(query: ^BroadPhaseQuery, box: ^AABox, callback: ^CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> b8 ---

	@(link_name = "JPH_BroadPhaseQuery_CollideSphere")
	BroadPhaseQuery_CollideSphere :: proc(query: ^BroadPhaseQuery, center: ^Vec3, radius: f32, callback: ^CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> b8 ---

	@(link_name = "JPH_BroadPhaseQuery_CollidePoint")
	BroadPhaseQuery_CollidePoint :: proc(query: ^BroadPhaseQuery, point: ^Vec3, callback: ^CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CastRay")
	NarrowPhaseQuery_CastRay :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, hit: ^RayCastResult, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CastRay2")
	NarrowPhaseQuery_CastRay2 :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, rayCastSettings: [^]RayCastSettings, callback: ^CastRayCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CastRay3")
	NarrowPhaseQuery_CastRay3 :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, rayCastSettings: [^]RayCastSettings, collectorType: CollisionCollectorType, callback: ^CastRayResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CollidePoint")
	NarrowPhaseQuery_CollidePoint :: proc(query: ^NarrowPhaseQuery, point: ^RVec3, callback: ^CollidePointCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CollidePoint2")
	NarrowPhaseQuery_CollidePoint2 :: proc(query: ^NarrowPhaseQuery, point: ^RVec3, collectorType: CollisionCollectorType, callback: ^CollidePointResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CollideShape")
	NarrowPhaseQuery_CollideShape :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, scale: ^Vec3, centerOfMassTransform: ^RMatrix4x4, settings: [^]CollideShapeSettings, baseOffset: ^RVec3, callback: ^CollideShapeCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CollideShape2")
	NarrowPhaseQuery_CollideShape2 :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, scale: ^Vec3, centerOfMassTransform: ^RMatrix4x4, settings: [^]CollideShapeSettings, baseOffset: ^RVec3, collectorType: CollisionCollectorType, callback: ^CollideShapeResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CastShape")
	NarrowPhaseQuery_CastShape :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, worldTransform: ^RMatrix4x4, direction: ^Vec3, settings: [^]ShapeCastSettings, baseOffset: ^RVec3, callback: ^CastShapeCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_NarrowPhaseQuery_CastShape2")
	NarrowPhaseQuery_CastShape2 :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, worldTransform: ^RMatrix4x4, direction: ^Vec3, settings: [^]ShapeCastSettings, baseOffset: ^RVec3, collectorType: CollisionCollectorType, callback: ^CastShapeResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_Body_GetID")
	Body_GetID :: proc(body: ^Body) -> BodyID ---

	@(link_name = "JPH_Body_GetBodyType")
	Body_GetBodyType :: proc(body: ^Body) -> BodyType ---

	@(link_name = "JPH_Body_IsRigidBody")
	Body_IsRigidBody :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_IsSoftBody")
	Body_IsSoftBody :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_IsActive")
	Body_IsActive :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_IsStatic")
	Body_IsStatic :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_IsKinematic")
	Body_IsKinematic :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_IsDynamic")
	Body_IsDynamic :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_CanBeKinematicOrDynamic")
	Body_CanBeKinematicOrDynamic :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_SetIsSensor")
	Body_SetIsSensor :: proc(body: ^Body, value: b8) ---

	@(link_name = "JPH_Body_IsSensor")
	Body_IsSensor :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_SetCollideKinematicVsNonDynamic")
	Body_SetCollideKinematicVsNonDynamic :: proc(body: ^Body, value: b8) ---

	@(link_name = "JPH_Body_GetCollideKinematicVsNonDynamic")
	Body_GetCollideKinematicVsNonDynamic :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_SetUseManifoldReduction")
	Body_SetUseManifoldReduction :: proc(body: ^Body, value: b8) ---

	@(link_name = "JPH_Body_GetUseManifoldReduction")
	Body_GetUseManifoldReduction :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_GetUseManifoldReductionWithBody")
	Body_GetUseManifoldReductionWithBody :: proc(body: ^Body, other: ^Body) -> b8 ---

	@(link_name = "JPH_Body_SetApplyGyroscopicForce")
	Body_SetApplyGyroscopicForce :: proc(body: ^Body, value: b8) ---

	@(link_name = "JPH_Body_GetApplyGyroscopicForce")
	Body_GetApplyGyroscopicForce :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_SetEnhancedInternalEdgeRemoval")
	Body_SetEnhancedInternalEdgeRemoval :: proc(body: ^Body, value: b8) ---

	@(link_name = "JPH_Body_GetEnhancedInternalEdgeRemoval")
	Body_GetEnhancedInternalEdgeRemoval :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_GetEnhancedInternalEdgeRemovalWithBody")
	Body_GetEnhancedInternalEdgeRemovalWithBody :: proc(body: ^Body, other: ^Body) -> b8 ---

	@(link_name = "JPH_Body_GetMotionType")
	Body_GetMotionType :: proc(body: ^Body) -> MotionType ---

	@(link_name = "JPH_Body_SetMotionType")
	Body_SetMotionType :: proc(body: ^Body, motionType: MotionType) ---

	@(link_name = "JPH_Body_GetBroadPhaseLayer")
	Body_GetBroadPhaseLayer :: proc(body: ^Body) -> BroadPhaseLayer ---

	@(link_name = "JPH_Body_GetObjectLayer")
	Body_GetObjectLayer :: proc(body: ^Body) -> ObjectLayer ---

	@(link_name = "JPH_Body_GetAllowSleeping")
	Body_GetAllowSleeping :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_SetAllowSleeping")
	Body_SetAllowSleeping :: proc(body: ^Body, allowSleeping: b8) ---

	@(link_name = "JPH_Body_ResetSleepTimer")
	Body_ResetSleepTimer :: proc(body: ^Body) ---

	@(link_name = "JPH_Body_GetFriction")
	Body_GetFriction :: proc(body: ^Body) -> f32 ---

	@(link_name = "JPH_Body_SetFriction")
	Body_SetFriction :: proc(body: ^Body, friction: f32) ---

	@(link_name = "JPH_Body_GetRestitution")
	Body_GetRestitution :: proc(body: ^Body) -> f32 ---

	@(link_name = "JPH_Body_SetRestitution")
	Body_SetRestitution :: proc(body: ^Body, restitution: f32) ---

	@(link_name = "JPH_Body_GetLinearVelocity")
	Body_GetLinearVelocity :: proc(body: ^Body, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_SetLinearVelocity")
	Body_SetLinearVelocity :: proc(body: ^Body, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_SetLinearVelocityClamped")
	Body_SetLinearVelocityClamped :: proc(body: ^Body, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_GetAngularVelocity")
	Body_GetAngularVelocity :: proc(body: ^Body, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_SetAngularVelocity")
	Body_SetAngularVelocity :: proc(body: ^Body, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_SetAngularVelocityClamped")
	Body_SetAngularVelocityClamped :: proc(body: ^Body, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_GetPointVelocityCOM")
	Body_GetPointVelocityCOM :: proc(body: ^Body, pointRelativeToCOM: ^Vec3, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_GetPointVelocity")
	Body_GetPointVelocity :: proc(body: ^Body, point: ^RVec3, velocity: ^Vec3) ---

	@(link_name = "JPH_Body_AddForce")
	Body_AddForce :: proc(body: ^Body, force: ^Vec3) ---

	@(link_name = "JPH_Body_AddForceAtPosition")
	Body_AddForceAtPosition :: proc(body: ^Body, force: ^Vec3, position: ^RVec3) ---

	@(link_name = "JPH_Body_AddTorque")
	Body_AddTorque :: proc(body: ^Body, force: ^Vec3) ---

	@(link_name = "JPH_Body_GetAccumulatedForce")
	Body_GetAccumulatedForce :: proc(body: ^Body, force: ^Vec3) ---

	@(link_name = "JPH_Body_GetAccumulatedTorque")
	Body_GetAccumulatedTorque :: proc(body: ^Body, force: ^Vec3) ---

	@(link_name = "JPH_Body_ResetForce")
	Body_ResetForce :: proc(body: ^Body) ---

	@(link_name = "JPH_Body_ResetTorque")
	Body_ResetTorque :: proc(body: ^Body) ---

	@(link_name = "JPH_Body_ResetMotion")
	Body_ResetMotion :: proc(body: ^Body) ---

	@(link_name = "JPH_Body_GetInverseInertia")
	Body_GetInverseInertia :: proc(body: ^Body, result: ^Matrix4x4) ---

	@(link_name = "JPH_Body_AddImpulse")
	Body_AddImpulse :: proc(body: ^Body, impulse: ^Vec3) ---

	@(link_name = "JPH_Body_AddImpulseAtPosition")
	Body_AddImpulseAtPosition :: proc(body: ^Body, impulse: ^Vec3, position: ^RVec3) ---

	@(link_name = "JPH_Body_AddAngularImpulse")
	Body_AddAngularImpulse :: proc(body: ^Body, angularImpulse: ^Vec3) ---

	@(link_name = "JPH_Body_MoveKinematic")
	Body_MoveKinematic :: proc(body: ^Body, targetPosition: ^RVec3, targetRotation: ^Quat, deltaTime: f32) ---

	@(link_name = "JPH_Body_ApplyBuoyancyImpulse")
	Body_ApplyBuoyancyImpulse :: proc(body: ^Body, surfacePosition: ^RVec3, surfaceNormal: ^Vec3, buoyancy: f32, linearDrag: f32, angularDrag: f32, fluidVelocity: ^Vec3, gravity: ^Vec3, deltaTime: f32) -> b8 ---

	@(link_name = "JPH_Body_IsInBroadPhase")
	Body_IsInBroadPhase :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_IsCollisionCacheInvalid")
	Body_IsCollisionCacheInvalid :: proc(body: ^Body) -> b8 ---

	@(link_name = "JPH_Body_GetShape")
	Body_GetShape :: proc(body: ^Body) -> ^Shape ---

	@(link_name = "JPH_Body_GetPosition")
	Body_GetPosition :: proc(body: ^Body, result: ^RVec3) ---

	@(link_name = "JPH_Body_GetRotation")
	Body_GetRotation :: proc(body: ^Body, result: ^Quat) ---

	@(link_name = "JPH_Body_GetWorldTransform")
	Body_GetWorldTransform :: proc(body: ^Body, result: ^RMatrix4x4) ---

	@(link_name = "JPH_Body_GetCenterOfMassPosition")
	Body_GetCenterOfMassPosition :: proc(body: ^Body, result: ^RVec3) ---

	@(link_name = "JPH_Body_GetCenterOfMassTransform")
	Body_GetCenterOfMassTransform :: proc(body: ^Body, result: ^RMatrix4x4) ---

	@(link_name = "JPH_Body_GetInverseCenterOfMassTransform")
	Body_GetInverseCenterOfMassTransform :: proc(body: ^Body, result: ^RMatrix4x4) ---

	@(link_name = "JPH_Body_GetWorldSpaceBounds")
	Body_GetWorldSpaceBounds :: proc(body: ^Body, result: ^AABox) ---

	@(link_name = "JPH_Body_GetWorldSpaceSurfaceNormal")
	Body_GetWorldSpaceSurfaceNormal :: proc(body: ^Body, subShapeID: SubShapeID, position: ^RVec3, normal: ^Vec3) ---

	@(link_name = "JPH_Body_GetMotionProperties")
	Body_GetMotionProperties :: proc(body: ^Body) -> ^MotionProperties ---

	@(link_name = "JPH_Body_GetMotionPropertiesUnchecked")
	Body_GetMotionPropertiesUnchecked :: proc(body: ^Body) -> ^MotionProperties ---

	@(link_name = "JPH_Body_SetUserData")
	Body_SetUserData :: proc(body: ^Body, userData: u64) ---

	@(link_name = "JPH_Body_GetUserData")
	Body_GetUserData :: proc(body: ^Body) -> u64 ---

	@(link_name = "JPH_Body_GetFixedToWorldBody")
	Body_GetFixedToWorldBody :: proc() -> ^Body ---

	@(link_name = "JPH_BroadPhaseLayerFilter_SetProcs")
	BroadPhaseLayerFilter_SetProcs :: proc(procs: [^]BroadPhaseLayerFilter_Procs) ---

	@(link_name = "JPH_BroadPhaseLayerFilter_Create")
	BroadPhaseLayerFilter_Create :: proc(userData: rawptr) -> ^BroadPhaseLayerFilter ---

	@(link_name = "JPH_BroadPhaseLayerFilter_Destroy")
	BroadPhaseLayerFilter_Destroy :: proc(filter: ^BroadPhaseLayerFilter) ---

	@(link_name = "JPH_ObjectLayerFilter_SetProcs")
	ObjectLayerFilter_SetProcs :: proc(procs: [^]ObjectLayerFilter_Procs) ---

	@(link_name = "JPH_ObjectLayerFilter_Create")
	ObjectLayerFilter_Create :: proc(userData: rawptr) -> ^ObjectLayerFilter ---

	@(link_name = "JPH_ObjectLayerFilter_Destroy")
	ObjectLayerFilter_Destroy :: proc(filter: ^ObjectLayerFilter) ---

	@(link_name = "JPH_BodyFilter_SetProcs")
	BodyFilter_SetProcs :: proc(procs: [^]BodyFilter_Procs) ---

	@(link_name = "JPH_BodyFilter_Create")
	BodyFilter_Create :: proc(userData: rawptr) -> ^BodyFilter ---

	@(link_name = "JPH_BodyFilter_Destroy")
	BodyFilter_Destroy :: proc(filter: ^BodyFilter) ---

	@(link_name = "JPH_ShapeFilter_SetProcs")
	ShapeFilter_SetProcs :: proc(procs: [^]ShapeFilter_Procs) ---

	@(link_name = "JPH_ShapeFilter_Create")
	ShapeFilter_Create :: proc(userData: rawptr) -> ^ShapeFilter ---

	@(link_name = "JPH_ShapeFilter_Destroy")
	ShapeFilter_Destroy :: proc(filter: ^ShapeFilter) ---

	@(link_name = "JPH_ShapeFilter_GetBodyID2")
	ShapeFilter_GetBodyID2 :: proc(filter: ^ShapeFilter) -> BodyID ---

	@(link_name = "JPH_ShapeFilter_SetBodyID2")
	ShapeFilter_SetBodyID2 :: proc(filter: ^ShapeFilter, id: BodyID) ---

	@(link_name = "JPH_SimShapeFilter_SetProcs")
	SimShapeFilter_SetProcs :: proc(procs: [^]SimShapeFilter_Procs) ---

	@(link_name = "JPH_SimShapeFilter_Create")
	SimShapeFilter_Create :: proc(userData: rawptr) -> ^SimShapeFilter ---

	@(link_name = "JPH_SimShapeFilter_Destroy")
	SimShapeFilter_Destroy :: proc(filter: ^SimShapeFilter) ---

	@(link_name = "JPH_ContactListener_SetProcs")
	ContactListener_SetProcs :: proc(procs: [^]ContactListener_Procs) ---

	@(link_name = "JPH_ContactListener_Create")
	ContactListener_Create :: proc(userData: rawptr) -> ^ContactListener ---

	@(link_name = "JPH_ContactListener_Destroy")
	ContactListener_Destroy :: proc(listener: ^ContactListener) ---

	@(link_name = "JPH_BodyActivationListener_SetProcs")
	BodyActivationListener_SetProcs :: proc(procs: [^]BodyActivationListener_Procs) ---

	@(link_name = "JPH_BodyActivationListener_Create")
	BodyActivationListener_Create :: proc(userData: rawptr) -> ^BodyActivationListener ---

	@(link_name = "JPH_BodyActivationListener_Destroy")
	BodyActivationListener_Destroy :: proc(listener: ^BodyActivationListener) ---

	@(link_name = "JPH_BodyDrawFilter_SetProcs")
	BodyDrawFilter_SetProcs :: proc(procs: [^]BodyDrawFilter_Procs) ---

	@(link_name = "JPH_BodyDrawFilter_Create")
	BodyDrawFilter_Create :: proc(userData: rawptr) -> ^BodyDrawFilter ---

	@(link_name = "JPH_BodyDrawFilter_Destroy")
	BodyDrawFilter_Destroy :: proc(filter: ^BodyDrawFilter) ---

	@(link_name = "JPH_ContactManifold_GetWorldSpaceNormal")
	ContactManifold_GetWorldSpaceNormal :: proc(manifold: ^ContactManifold, result: ^Vec3) ---

	@(link_name = "JPH_ContactManifold_GetPenetrationDepth")
	ContactManifold_GetPenetrationDepth :: proc(manifold: ^ContactManifold) -> f32 ---

	@(link_name = "JPH_ContactManifold_GetSubShapeID1")
	ContactManifold_GetSubShapeID1 :: proc(manifold: ^ContactManifold) -> SubShapeID ---

	@(link_name = "JPH_ContactManifold_GetSubShapeID2")
	ContactManifold_GetSubShapeID2 :: proc(manifold: ^ContactManifold) -> SubShapeID ---

	@(link_name = "JPH_ContactManifold_GetPointCount")
	ContactManifold_GetPointCount :: proc(manifold: ^ContactManifold) -> u32 ---

	@(link_name = "JPH_ContactManifold_GetWorldSpaceContactPointOn1")
	ContactManifold_GetWorldSpaceContactPointOn1 :: proc(manifold: ^ContactManifold, index: u32, result: ^RVec3) ---

	@(link_name = "JPH_ContactManifold_GetWorldSpaceContactPointOn2")
	ContactManifold_GetWorldSpaceContactPointOn2 :: proc(manifold: ^ContactManifold, index: u32, result: ^RVec3) ---

	@(link_name = "JPH_ContactSettings_GetFriction")
	ContactSettings_GetFriction :: proc(settings: [^]ContactSettings) -> f32 ---

	@(link_name = "JPH_ContactSettings_SetFriction")
	ContactSettings_SetFriction :: proc(settings: [^]ContactSettings, friction: f32) ---

	@(link_name = "JPH_ContactSettings_GetRestitution")
	ContactSettings_GetRestitution :: proc(settings: [^]ContactSettings) -> f32 ---

	@(link_name = "JPH_ContactSettings_SetRestitution")
	ContactSettings_SetRestitution :: proc(settings: [^]ContactSettings, restitution: f32) ---

	@(link_name = "JPH_ContactSettings_GetInvMassScale1")
	ContactSettings_GetInvMassScale1 :: proc(settings: [^]ContactSettings) -> f32 ---

	@(link_name = "JPH_ContactSettings_SetInvMassScale1")
	ContactSettings_SetInvMassScale1 :: proc(settings: [^]ContactSettings, scale: f32) ---

	@(link_name = "JPH_ContactSettings_GetInvInertiaScale1")
	ContactSettings_GetInvInertiaScale1 :: proc(settings: [^]ContactSettings) -> f32 ---

	@(link_name = "JPH_ContactSettings_SetInvInertiaScale1")
	ContactSettings_SetInvInertiaScale1 :: proc(settings: [^]ContactSettings, scale: f32) ---

	@(link_name = "JPH_ContactSettings_GetInvMassScale2")
	ContactSettings_GetInvMassScale2 :: proc(settings: [^]ContactSettings) -> f32 ---

	@(link_name = "JPH_ContactSettings_SetInvMassScale2")
	ContactSettings_SetInvMassScale2 :: proc(settings: [^]ContactSettings, scale: f32) ---

	@(link_name = "JPH_ContactSettings_GetInvInertiaScale2")
	ContactSettings_GetInvInertiaScale2 :: proc(settings: [^]ContactSettings) -> f32 ---

	@(link_name = "JPH_ContactSettings_SetInvInertiaScale2")
	ContactSettings_SetInvInertiaScale2 :: proc(settings: [^]ContactSettings, scale: f32) ---

	@(link_name = "JPH_ContactSettings_GetIsSensor")
	ContactSettings_GetIsSensor :: proc(settings: [^]ContactSettings) -> b8 ---

	@(link_name = "JPH_ContactSettings_SetIsSensor")
	ContactSettings_SetIsSensor :: proc(settings: [^]ContactSettings, sensor: b8) ---

	@(link_name = "JPH_ContactSettings_GetRelativeLinearSurfaceVelocity")
	ContactSettings_GetRelativeLinearSurfaceVelocity :: proc(settings: [^]ContactSettings, result: ^Vec3) ---

	@(link_name = "JPH_ContactSettings_SetRelativeLinearSurfaceVelocity")
	ContactSettings_SetRelativeLinearSurfaceVelocity :: proc(settings: [^]ContactSettings, velocity: ^Vec3) ---

	@(link_name = "JPH_ContactSettings_GetRelativeAngularSurfaceVelocity")
	ContactSettings_GetRelativeAngularSurfaceVelocity :: proc(settings: [^]ContactSettings, result: ^Vec3) ---

	@(link_name = "JPH_ContactSettings_SetRelativeAngularSurfaceVelocity")
	ContactSettings_SetRelativeAngularSurfaceVelocity :: proc(settings: [^]ContactSettings, velocity: ^Vec3) ---

	@(link_name = "JPH_CharacterBase_Destroy")
	CharacterBase_Destroy :: proc(character: ^CharacterBase) ---

	@(link_name = "JPH_CharacterBase_GetCosMaxSlopeAngle")
	CharacterBase_GetCosMaxSlopeAngle :: proc(character: ^CharacterBase) -> f32 ---

	@(link_name = "JPH_CharacterBase_SetMaxSlopeAngle")
	CharacterBase_SetMaxSlopeAngle :: proc(character: ^CharacterBase, maxSlopeAngle: f32) ---

	@(link_name = "JPH_CharacterBase_GetUp")
	CharacterBase_GetUp :: proc(character: ^CharacterBase, result: ^Vec3) ---

	@(link_name = "JPH_CharacterBase_SetUp")
	CharacterBase_SetUp :: proc(character: ^CharacterBase, value: ^Vec3) ---

	@(link_name = "JPH_CharacterBase_IsSlopeTooSteep")
	CharacterBase_IsSlopeTooSteep :: proc(character: ^CharacterBase, value: ^Vec3) -> b8 ---

	@(link_name = "JPH_CharacterBase_GetShape")
	CharacterBase_GetShape :: proc(character: ^CharacterBase) -> ^Shape ---

	@(link_name = "JPH_CharacterBase_GetGroundState")
	CharacterBase_GetGroundState :: proc(character: ^CharacterBase) -> GroundState ---

	@(link_name = "JPH_CharacterBase_IsSupported")
	CharacterBase_IsSupported :: proc(character: ^CharacterBase) -> b8 ---

	@(link_name = "JPH_CharacterBase_GetGroundPosition")
	CharacterBase_GetGroundPosition :: proc(character: ^CharacterBase, position: ^RVec3) ---

	@(link_name = "JPH_CharacterBase_GetGroundNormal")
	CharacterBase_GetGroundNormal :: proc(character: ^CharacterBase, normal: ^Vec3) ---

	@(link_name = "JPH_CharacterBase_GetGroundVelocity")
	CharacterBase_GetGroundVelocity :: proc(character: ^CharacterBase, velocity: ^Vec3) ---

	@(link_name = "JPH_CharacterBase_GetGroundMaterial")
	CharacterBase_GetGroundMaterial :: proc(character: ^CharacterBase) -> ^PhysicsMaterial ---

	@(link_name = "JPH_CharacterBase_GetGroundBodyId")
	CharacterBase_GetGroundBodyId :: proc(character: ^CharacterBase) -> BodyID ---

	@(link_name = "JPH_CharacterBase_GetGroundSubShapeId")
	CharacterBase_GetGroundSubShapeId :: proc(character: ^CharacterBase) -> SubShapeID ---

	@(link_name = "JPH_CharacterBase_GetGroundUserData")
	CharacterBase_GetGroundUserData :: proc(character: ^CharacterBase) -> u64 ---

	@(link_name = "JPH_CharacterSettings_Init")
	CharacterSettings_Init :: proc(settings: [^]CharacterSettings) ---

	@(link_name = "JPH_Character_Create")
	Character_Create :: proc(settings: [^]CharacterSettings, position: ^RVec3, rotation: ^Quat, userData: u64, system: ^PhysicsSystem) -> ^Character ---

	@(link_name = "JPH_Character_AddToPhysicsSystem")
	Character_AddToPhysicsSystem :: proc(character: ^Character, activationMode: Activation, lockBodies: b8) ---

	@(link_name = "JPH_Character_RemoveFromPhysicsSystem")
	Character_RemoveFromPhysicsSystem :: proc(character: ^Character, lockBodies: b8) ---

	@(link_name = "JPH_Character_Activate")
	Character_Activate :: proc(character: ^Character, lockBodies: b8) ---

	@(link_name = "JPH_Character_PostSimulation")
	Character_PostSimulation :: proc(character: ^Character, maxSeparationDistance: f32, lockBodies: b8) ---

	@(link_name = "JPH_Character_SetLinearAndAngularVelocity")
	Character_SetLinearAndAngularVelocity :: proc(character: ^Character, linearVelocity: ^Vec3, angularVelocity: ^Vec3, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetLinearVelocity")
	Character_GetLinearVelocity :: proc(character: ^Character, result: ^Vec3) ---

	@(link_name = "JPH_Character_SetLinearVelocity")
	Character_SetLinearVelocity :: proc(character: ^Character, value: ^Vec3, lockBodies: b8) ---

	@(link_name = "JPH_Character_AddLinearVelocity")
	Character_AddLinearVelocity :: proc(character: ^Character, value: ^Vec3, lockBodies: b8) ---

	@(link_name = "JPH_Character_AddImpulse")
	Character_AddImpulse :: proc(character: ^Character, value: ^Vec3, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetBodyID")
	Character_GetBodyID :: proc(character: ^Character) -> BodyID ---

	@(link_name = "JPH_Character_GetPositionAndRotation")
	Character_GetPositionAndRotation :: proc(character: ^Character, position: ^RVec3, rotation: ^Quat, lockBodies: b8) ---

	@(link_name = "JPH_Character_SetPositionAndRotation")
	Character_SetPositionAndRotation :: proc(character: ^Character, position: ^RVec3, rotation: ^Quat, activationMode: Activation, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetPosition")
	Character_GetPosition :: proc(character: ^Character, position: ^RVec3, lockBodies: b8) ---

	@(link_name = "JPH_Character_SetPosition")
	Character_SetPosition :: proc(character: ^Character, position: ^RVec3, activationMode: Activation, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetRotation")
	Character_GetRotation :: proc(character: ^Character, rotation: ^Quat, lockBodies: b8) ---

	@(link_name = "JPH_Character_SetRotation")
	Character_SetRotation :: proc(character: ^Character, rotation: ^Quat, activationMode: Activation, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetCenterOfMassPosition")
	Character_GetCenterOfMassPosition :: proc(character: ^Character, result: ^RVec3, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetWorldTransform")
	Character_GetWorldTransform :: proc(character: ^Character, result: ^RMatrix4x4, lockBodies: b8) ---

	@(link_name = "JPH_Character_GetLayer")
	Character_GetLayer :: proc(character: ^Character) -> ObjectLayer ---

	@(link_name = "JPH_Character_SetLayer")
	Character_SetLayer :: proc(character: ^Character, value: ObjectLayer, lockBodies: b8) ---

	@(link_name = "JPH_Character_SetShape")
	Character_SetShape :: proc(character: ^Character, shape: ^Shape, maxPenetrationDepth: f32, lockBodies: b8) ---

	@(link_name = "JPH_CharacterVirtualSettings_Init")
	CharacterVirtualSettings_Init :: proc(settings: [^]CharacterVirtualSettings) ---

	@(link_name = "JPH_CharacterVirtual_Create")
	CharacterVirtual_Create :: proc(settings: [^]CharacterVirtualSettings, position: ^RVec3, rotation: ^Quat, userData: u64, system: ^PhysicsSystem) -> ^CharacterVirtual ---

	@(link_name = "JPH_CharacterVirtual_GetID")
	CharacterVirtual_GetID :: proc(character: ^CharacterVirtual) -> CharacterID ---

	@(link_name = "JPH_CharacterVirtual_SetListener")
	CharacterVirtual_SetListener :: proc(character: ^CharacterVirtual, listener: ^CharacterContactListener) ---

	@(link_name = "JPH_CharacterVirtual_SetCharacterVsCharacterCollision")
	CharacterVirtual_SetCharacterVsCharacterCollision :: proc(character: ^CharacterVirtual, characterVsCharacterCollision: ^CharacterVsCharacterCollision) ---

	@(link_name = "JPH_CharacterVirtual_GetLinearVelocity")
	CharacterVirtual_GetLinearVelocity :: proc(character: ^CharacterVirtual, velocity: ^Vec3) ---

	@(link_name = "JPH_CharacterVirtual_SetLinearVelocity")
	CharacterVirtual_SetLinearVelocity :: proc(character: ^CharacterVirtual, velocity: ^Vec3) ---

	@(link_name = "JPH_CharacterVirtual_GetPosition")
	CharacterVirtual_GetPosition :: proc(character: ^CharacterVirtual, position: ^RVec3) ---

	@(link_name = "JPH_CharacterVirtual_SetPosition")
	CharacterVirtual_SetPosition :: proc(character: ^CharacterVirtual, position: ^RVec3) ---

	@(link_name = "JPH_CharacterVirtual_GetRotation")
	CharacterVirtual_GetRotation :: proc(character: ^CharacterVirtual, rotation: ^Quat) ---

	@(link_name = "JPH_CharacterVirtual_SetRotation")
	CharacterVirtual_SetRotation :: proc(character: ^CharacterVirtual, rotation: ^Quat) ---

	@(link_name = "JPH_CharacterVirtual_GetWorldTransform")
	CharacterVirtual_GetWorldTransform :: proc(character: ^CharacterVirtual, result: ^RMatrix4x4) ---

	@(link_name = "JPH_CharacterVirtual_GetCenterOfMassTransform")
	CharacterVirtual_GetCenterOfMassTransform :: proc(character: ^CharacterVirtual, result: ^RMatrix4x4) ---

	@(link_name = "JPH_CharacterVirtual_GetMass")
	CharacterVirtual_GetMass :: proc(character: ^CharacterVirtual) -> f32 ---

	@(link_name = "JPH_CharacterVirtual_SetMass")
	CharacterVirtual_SetMass :: proc(character: ^CharacterVirtual, value: f32) ---

	@(link_name = "JPH_CharacterVirtual_GetMaxStrength")
	CharacterVirtual_GetMaxStrength :: proc(character: ^CharacterVirtual) -> f32 ---

	@(link_name = "JPH_CharacterVirtual_SetMaxStrength")
	CharacterVirtual_SetMaxStrength :: proc(character: ^CharacterVirtual, value: f32) ---

	@(link_name = "JPH_CharacterVirtual_GetPenetrationRecoverySpeed")
	CharacterVirtual_GetPenetrationRecoverySpeed :: proc(character: ^CharacterVirtual) -> f32 ---

	@(link_name = "JPH_CharacterVirtual_SetPenetrationRecoverySpeed")
	CharacterVirtual_SetPenetrationRecoverySpeed :: proc(character: ^CharacterVirtual, value: f32) ---

	@(link_name = "JPH_CharacterVirtual_GetEnhancedInternalEdgeRemoval")
	CharacterVirtual_GetEnhancedInternalEdgeRemoval :: proc(character: ^CharacterVirtual) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_SetEnhancedInternalEdgeRemoval")
	CharacterVirtual_SetEnhancedInternalEdgeRemoval :: proc(character: ^CharacterVirtual, value: b8) ---

	@(link_name = "JPH_CharacterVirtual_GetCharacterPadding")
	CharacterVirtual_GetCharacterPadding :: proc(character: ^CharacterVirtual) -> f32 ---

	@(link_name = "JPH_CharacterVirtual_GetMaxNumHits")
	CharacterVirtual_GetMaxNumHits :: proc(character: ^CharacterVirtual) -> u32 ---

	@(link_name = "JPH_CharacterVirtual_SetMaxNumHits")
	CharacterVirtual_SetMaxNumHits :: proc(character: ^CharacterVirtual, value: u32) ---

	@(link_name = "JPH_CharacterVirtual_GetHitReductionCosMaxAngle")
	CharacterVirtual_GetHitReductionCosMaxAngle :: proc(character: ^CharacterVirtual) -> f32 ---

	@(link_name = "JPH_CharacterVirtual_SetHitReductionCosMaxAngle")
	CharacterVirtual_SetHitReductionCosMaxAngle :: proc(character: ^CharacterVirtual, value: f32) ---

	@(link_name = "JPH_CharacterVirtual_GetMaxHitsExceeded")
	CharacterVirtual_GetMaxHitsExceeded :: proc(character: ^CharacterVirtual) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_GetShapeOffset")
	CharacterVirtual_GetShapeOffset :: proc(character: ^CharacterVirtual, result: ^Vec3) ---

	@(link_name = "JPH_CharacterVirtual_SetShapeOffset")
	CharacterVirtual_SetShapeOffset :: proc(character: ^CharacterVirtual, value: ^Vec3) ---

	@(link_name = "JPH_CharacterVirtual_GetUserData")
	CharacterVirtual_GetUserData :: proc(character: ^CharacterVirtual) -> u64 ---

	@(link_name = "JPH_CharacterVirtual_SetUserData")
	CharacterVirtual_SetUserData :: proc(character: ^CharacterVirtual, value: u64) ---

	@(link_name = "JPH_CharacterVirtual_GetInnerBodyID")
	CharacterVirtual_GetInnerBodyID :: proc(character: ^CharacterVirtual) -> BodyID ---

	@(link_name = "JPH_CharacterVirtual_CancelVelocityTowardsSteepSlopes")
	CharacterVirtual_CancelVelocityTowardsSteepSlopes :: proc(character: ^CharacterVirtual, desiredVelocity: ^Vec3, velocity: ^Vec3) ---

	@(link_name = "JPH_CharacterVirtual_StartTrackingContactChanges")
	CharacterVirtual_StartTrackingContactChanges :: proc(character: ^CharacterVirtual) ---

	@(link_name = "JPH_CharacterVirtual_FinishTrackingContactChanges")
	CharacterVirtual_FinishTrackingContactChanges :: proc(character: ^CharacterVirtual) ---

	@(link_name = "JPH_CharacterVirtual_Update")
	CharacterVirtual_Update :: proc(character: ^CharacterVirtual, deltaTime: f32, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---

	@(link_name = "JPH_CharacterVirtual_ExtendedUpdate")
	CharacterVirtual_ExtendedUpdate :: proc(character: ^CharacterVirtual, deltaTime: f32, settings: [^]ExtendedUpdateSettings, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---

	@(link_name = "JPH_CharacterVirtual_RefreshContacts")
	CharacterVirtual_RefreshContacts :: proc(character: ^CharacterVirtual, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---

	@(link_name = "JPH_CharacterVirtual_CanWalkStairs")
	CharacterVirtual_CanWalkStairs :: proc(character: ^CharacterVirtual, linearVelocity: ^Vec3) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_WalkStairs")
	CharacterVirtual_WalkStairs :: proc(character: ^CharacterVirtual, deltaTime: f32, stepUp: ^Vec3, stepForward: ^Vec3, stepForwardTest: ^Vec3, stepDownExtra: ^Vec3, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_StickToFloor")
	CharacterVirtual_StickToFloor :: proc(character: ^CharacterVirtual, stepDown: ^Vec3, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_UpdateGroundVelocity")
	CharacterVirtual_UpdateGroundVelocity :: proc(character: ^CharacterVirtual) ---

	@(link_name = "JPH_CharacterVirtual_SetShape")
	CharacterVirtual_SetShape :: proc(character: ^CharacterVirtual, shape: ^Shape, maxPenetrationDepth: f32, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_SetInnerBodyShape")
	CharacterVirtual_SetInnerBodyShape :: proc(character: ^CharacterVirtual, shape: ^Shape) ---

	@(link_name = "JPH_CharacterVirtual_GetNumActiveContacts")
	CharacterVirtual_GetNumActiveContacts :: proc(character: ^CharacterVirtual) -> u32 ---

	@(link_name = "JPH_CharacterVirtual_GetActiveContact")
	CharacterVirtual_GetActiveContact :: proc(character: ^CharacterVirtual, index: u32, result: ^CharacterVirtualContact) ---

	@(link_name = "JPH_CharacterVirtual_HasCollidedWithBody")
	CharacterVirtual_HasCollidedWithBody :: proc(character: ^CharacterVirtual, body: BodyID) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_HasCollidedWith")
	CharacterVirtual_HasCollidedWith :: proc(character: ^CharacterVirtual, other: CharacterID) -> b8 ---

	@(link_name = "JPH_CharacterVirtual_HasCollidedWithCharacter")
	CharacterVirtual_HasCollidedWithCharacter :: proc(character: ^CharacterVirtual, other: ^CharacterVirtual) -> b8 ---

	@(link_name = "JPH_CharacterContactListener_SetProcs")
	CharacterContactListener_SetProcs :: proc(procs: [^]CharacterContactListener_Procs) ---

	@(link_name = "JPH_CharacterContactListener_Create")
	CharacterContactListener_Create :: proc(userData: rawptr) -> ^CharacterContactListener ---

	@(link_name = "JPH_CharacterContactListener_Destroy")
	CharacterContactListener_Destroy :: proc(listener: ^CharacterContactListener) ---

	@(link_name = "JPH_CharacterVsCharacterCollision_SetProcs")
	CharacterVsCharacterCollision_SetProcs :: proc(procs: [^]CharacterVsCharacterCollision_Procs) ---

	@(link_name = "JPH_CharacterVsCharacterCollision_Create")
	CharacterVsCharacterCollision_Create :: proc(userData: rawptr) -> ^CharacterVsCharacterCollision ---

	@(link_name = "JPH_CharacterVsCharacterCollision_CreateSimple")
	CharacterVsCharacterCollision_CreateSimple :: proc() -> ^CharacterVsCharacterCollision ---

	@(link_name = "JPH_CharacterVsCharacterCollisionSimple_AddCharacter")
	CharacterVsCharacterCollisionSimple_AddCharacter :: proc(characterVsCharacter: ^CharacterVsCharacterCollision, character: ^CharacterVirtual) ---

	@(link_name = "JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter")
	CharacterVsCharacterCollisionSimple_RemoveCharacter :: proc(characterVsCharacter: ^CharacterVsCharacterCollision, character: ^CharacterVirtual) ---

	@(link_name = "JPH_CharacterVsCharacterCollision_Destroy")
	CharacterVsCharacterCollision_Destroy :: proc(listener: ^CharacterVsCharacterCollision) ---

	@(link_name = "JPH_CollisionDispatch_CollideShapeVsShape")
	CollisionDispatch_CollideShapeVsShape :: proc(shape1: ^Shape, shape2: ^Shape, scale1: ^Vec3, scale2: ^Vec3, centerOfMassTransform1: ^Matrix4x4, centerOfMassTransform2: ^Matrix4x4, collideShapeSettings: [^]CollideShapeSettings, callback: ^CollideShapeCollectorCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_CollisionDispatch_CastShapeVsShapeLocalSpace")
	CollisionDispatch_CastShapeVsShapeLocalSpace :: proc(direction: ^Vec3, shape1: ^Shape, shape2: ^Shape, scale1InShape2LocalSpace: ^Vec3, scale2: ^Vec3, centerOfMassTransform1InShape2LocalSpace: ^Matrix4x4, centerOfMassWorldTransform2: ^Matrix4x4, shapeCastSettings: [^]ShapeCastSettings, callback: ^CastShapeCollectorCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_CollisionDispatch_CastShapeVsShapeWorldSpace")
	CollisionDispatch_CastShapeVsShapeWorldSpace :: proc(direction: ^Vec3, shape1: ^Shape, shape2: ^Shape, scale1: ^Vec3, inScale2: ^Vec3, centerOfMassWorldTransform1: ^Matrix4x4, centerOfMassWorldTransform2: ^Matrix4x4, shapeCastSettings: [^]ShapeCastSettings, callback: ^CastShapeCollectorCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> b8 ---

	@(link_name = "JPH_DebugRenderer_SetProcs")
	DebugRenderer_SetProcs :: proc(procs: [^]DebugRenderer_Procs) ---

	@(link_name = "JPH_DebugRenderer_Create")
	DebugRenderer_Create :: proc(userData: rawptr) -> ^DebugRenderer ---

	@(link_name = "JPH_DebugRenderer_Destroy")
	DebugRenderer_Destroy :: proc(renderer: ^DebugRenderer) ---

	@(link_name = "JPH_DebugRenderer_NextFrame")
	DebugRenderer_NextFrame :: proc(renderer: ^DebugRenderer) ---

	@(link_name = "JPH_DebugRenderer_DrawLine")
	DebugRenderer_DrawLine :: proc(renderer: ^DebugRenderer, from: ^RVec3, to: ^RVec3, color: Color) ---

	@(link_name = "JPH_DebugRenderer_DrawWireBox")
	DebugRenderer_DrawWireBox :: proc(renderer: ^DebugRenderer, box: ^AABox, color: Color) ---

	@(link_name = "JPH_DebugRenderer_DrawWireBox2")
	DebugRenderer_DrawWireBox2 :: proc(renderer: ^DebugRenderer, matrix_p: ^RMatrix4x4, box: ^AABox, color: Color) ---

	@(link_name = "JPH_DebugRenderer_DrawMarker")
	DebugRenderer_DrawMarker :: proc(renderer: ^DebugRenderer, position: ^RVec3, color: Color, size: f32) ---

	@(link_name = "JPH_DebugRenderer_DrawArrow")
	DebugRenderer_DrawArrow :: proc(renderer: ^DebugRenderer, from: ^RVec3, to: ^RVec3, color: Color, size: f32) ---

	@(link_name = "JPH_DebugRenderer_DrawCoordinateSystem")
	DebugRenderer_DrawCoordinateSystem :: proc(renderer: ^DebugRenderer, matrix_p: ^RMatrix4x4, size: f32) ---

	@(link_name = "JPH_DebugRenderer_DrawPlane")
	DebugRenderer_DrawPlane :: proc(renderer: ^DebugRenderer, point: ^RVec3, normal: ^Vec3, color: Color, size: f32) ---

	@(link_name = "JPH_DebugRenderer_DrawWireTriangle")
	DebugRenderer_DrawWireTriangle :: proc(renderer: ^DebugRenderer, v1: ^RVec3, v2: ^RVec3, v3: ^RVec3, color: Color) ---

	@(link_name = "JPH_DebugRenderer_DrawWireSphere")
	DebugRenderer_DrawWireSphere :: proc(renderer: ^DebugRenderer, center: ^RVec3, radius: f32, color: Color, level: i32) ---

	@(link_name = "JPH_DebugRenderer_DrawWireUnitSphere")
	DebugRenderer_DrawWireUnitSphere :: proc(renderer: ^DebugRenderer, matrix_p: ^RMatrix4x4, color: Color, level: i32) ---

	@(link_name = "JPH_Skeleton_Create")
	Skeleton_Create :: proc() -> ^Skeleton ---

	@(link_name = "JPH_Skeleton_Destroy")
	Skeleton_Destroy :: proc(skeleton: ^Skeleton) ---

	@(link_name = "JPH_Skeleton_AddJoint")
	Skeleton_AddJoint :: proc(skeleton: ^Skeleton, name: cstring) -> u32 ---

	@(link_name = "JPH_Skeleton_AddJoint2")
	Skeleton_AddJoint2 :: proc(skeleton: ^Skeleton, name: cstring, parentIndex: i32) -> u32 ---

	@(link_name = "JPH_Skeleton_AddJoint3")
	Skeleton_AddJoint3 :: proc(skeleton: ^Skeleton, name: cstring, parentName: cstring) -> u32 ---

	@(link_name = "JPH_Skeleton_GetJointCount")
	Skeleton_GetJointCount :: proc(skeleton: ^Skeleton) -> i32 ---

	@(link_name = "JPH_Skeleton_GetJoint")
	Skeleton_GetJoint :: proc(skeleton: ^Skeleton, index: i32, joint: ^SkeletonJoint) ---

	@(link_name = "JPH_Skeleton_GetJointIndex")
	Skeleton_GetJointIndex :: proc(skeleton: ^Skeleton, name: cstring) -> i32 ---

	@(link_name = "JPH_Skeleton_CalculateParentJointIndices")
	Skeleton_CalculateParentJointIndices :: proc(skeleton: ^Skeleton) ---

	@(link_name = "JPH_Skeleton_AreJointsCorrectlyOrdered")
	Skeleton_AreJointsCorrectlyOrdered :: proc(skeleton: ^Skeleton) -> b8 ---

	@(link_name = "JPH_RagdollSettings_Create")
	RagdollSettings_Create :: proc() -> ^RagdollSettings ---

	@(link_name = "JPH_RagdollSettings_Destroy")
	RagdollSettings_Destroy :: proc(settings: [^]RagdollSettings) ---

	@(link_name = "JPH_RagdollSettings_GetSkeleton")
	RagdollSettings_GetSkeleton :: proc(character: ^RagdollSettings) -> ^Skeleton ---

	@(link_name = "JPH_RagdollSettings_SetSkeleton")
	RagdollSettings_SetSkeleton :: proc(character: ^RagdollSettings, skeleton: ^Skeleton) ---

	@(link_name = "JPH_RagdollSettings_Stabilize")
	RagdollSettings_Stabilize :: proc(settings: [^]RagdollSettings) -> b8 ---

	@(link_name = "JPH_RagdollSettings_DisableParentChildCollisions")
	RagdollSettings_DisableParentChildCollisions :: proc(settings: [^]RagdollSettings, jointMatrices: [^]Matrix4x4, minSeparationDistance: f32) ---

	@(link_name = "JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex")
	RagdollSettings_CalculateBodyIndexToConstraintIndex :: proc(settings: [^]RagdollSettings) ---

	@(link_name = "JPH_RagdollSettings_GetConstraintIndexForBodyIndex")
	RagdollSettings_GetConstraintIndexForBodyIndex :: proc(settings: [^]RagdollSettings, bodyIndex: i32) -> i32 ---

	@(link_name = "JPH_RagdollSettings_CalculateConstraintIndexToBodyIdxPair")
	RagdollSettings_CalculateConstraintIndexToBodyIdxPair :: proc(settings: [^]RagdollSettings) ---

	@(link_name = "JPH_RagdollSettings_CreateRagdoll")
	RagdollSettings_CreateRagdoll :: proc(settings: [^]RagdollSettings, system: ^PhysicsSystem, collisionGroup: CollisionGroupID, userData: u64) -> ^Ragdoll ---

	@(link_name = "JPH_Ragdoll_Destroy")
	Ragdoll_Destroy :: proc(ragdoll: ^Ragdoll) ---

	@(link_name = "JPH_Ragdoll_AddToPhysicsSystem")
	Ragdoll_AddToPhysicsSystem :: proc(ragdoll: ^Ragdoll, activationMode: Activation, lockBodies: b8) ---

	@(link_name = "JPH_Ragdoll_RemoveFromPhysicsSystem")
	Ragdoll_RemoveFromPhysicsSystem :: proc(ragdoll: ^Ragdoll, lockBodies: b8) ---

	@(link_name = "JPH_Ragdoll_Activate")
	Ragdoll_Activate :: proc(ragdoll: ^Ragdoll, lockBodies: b8) ---

	@(link_name = "JPH_Ragdoll_IsActive")
	Ragdoll_IsActive :: proc(ragdoll: ^Ragdoll, lockBodies: b8) -> b8 ---

	@(link_name = "JPH_Ragdoll_ResetWarmStart")
	Ragdoll_ResetWarmStart :: proc(ragdoll: ^Ragdoll) ---

	@(link_name = "JPH_EstimateCollisionResponse")
	EstimateCollisionResponse :: proc(body1: ^Body, body2: ^Body, manifold: ^ContactManifold, combinedFriction: f32, combinedRestitution: f32, minVelocityForRestitution: f32, numIterations: u32, result: ^CollisionEstimationResult) ---

}

when (ODIN_OS == .Windows) {

	API_CALL :: `__cdecl`

	PhysicsUpdateError :: enum i32 {
		PhysicsUpdateError_None                   = 0,
		PhysicsUpdateError_ManifoldCacheFull      = 1,
		PhysicsUpdateError_BodyPairCacheFull      = 2,
		PhysicsUpdateError_ContactConstraintsFull = 4,
		_JPH_PhysicsUpdateError_Count             = 5,
		_JPH_PhysicsUpdateError_Force32           = 2147483647,
	}
	BodyType :: enum i32 {
		BodyType_Rigid        = 0,
		BodyType_Soft         = 1,
		_JPH_BodyType_Count   = 2,
		_JPH_BodyType_Force32 = 2147483647,
	}
	MotionType :: enum i32 {
		MotionType_Static       = 0,
		MotionType_Kinematic    = 1,
		MotionType_Dynamic      = 2,
		_JPH_MotionType_Count   = 3,
		_JPH_MotionType_Force32 = 2147483647,
	}
	Activation :: enum i32 {
		Activation_Activate     = 0,
		Activation_DontActivate = 1,
		_JPH_Activation_Count   = 2,
		_JPH_Activation_Force32 = 2147483647,
	}
	ValidateResult :: enum i32 {
		ValidateResult_AcceptAllContactsForThisBodyPair = 0,
		ValidateResult_AcceptContact                    = 1,
		ValidateResult_RejectContact                    = 2,
		ValidateResult_RejectAllContactsForThisBodyPair = 3,
		_JPH_ValidateResult_Count                       = 4,
		_JPH_ValidateResult_Force32                     = 2147483647,
	}
	ShapeType :: enum i32 {
		ShapeType_Convex       = 0,
		ShapeType_Compound     = 1,
		ShapeType_Decorated    = 2,
		ShapeType_Mesh         = 3,
		ShapeType_HeightField  = 4,
		ShapeType_SoftBody     = 5,
		ShapeType_User1        = 6,
		ShapeType_User2        = 7,
		ShapeType_User3        = 8,
		ShapeType_User4        = 9,
		_JPH_ShapeType_Count   = 10,
		_JPH_ShapeType_Force32 = 2147483647,
	}
	ShapeSubType :: enum i32 {
		ShapeSubType_Sphere             = 0,
		ShapeSubType_Box                = 1,
		ShapeSubType_Triangle           = 2,
		ShapeSubType_Capsule            = 3,
		ShapeSubType_TaperedCapsule     = 4,
		ShapeSubType_Cylinder           = 5,
		ShapeSubType_ConvexHull         = 6,
		ShapeSubType_StaticCompound     = 7,
		ShapeSubType_MutableCompound    = 8,
		ShapeSubType_RotatedTranslated  = 9,
		ShapeSubType_Scaled             = 10,
		ShapeSubType_OffsetCenterOfMass = 11,
		ShapeSubType_Mesh               = 12,
		ShapeSubType_HeightField        = 13,
		ShapeSubType_SoftBody           = 14,
		_JPH_ShapeSubType_Count         = 15,
		_JPH_ShapeSubType_Force32       = 2147483647,
	}
	ConstraintType :: enum i32 {
		ConstraintType_Constraint        = 0,
		ConstraintType_TwoBodyConstraint = 1,
		_JPH_ConstraintType_Count        = 2,
		_JPH_ConstraintType_Force32      = 2147483647,
	}
	ConstraintSubType :: enum i32 {
		ConstraintSubType_Fixed         = 0,
		ConstraintSubType_Point         = 1,
		ConstraintSubType_Hinge         = 2,
		ConstraintSubType_Slider        = 3,
		ConstraintSubType_Distance      = 4,
		ConstraintSubType_Cone          = 5,
		ConstraintSubType_SwingTwist    = 6,
		ConstraintSubType_SixDOF        = 7,
		ConstraintSubType_Path          = 8,
		ConstraintSubType_Vehicle       = 9,
		ConstraintSubType_RackAndPinion = 10,
		ConstraintSubType_Gear          = 11,
		ConstraintSubType_Pulley        = 12,
		ConstraintSubType_User1         = 13,
		ConstraintSubType_User2         = 14,
		ConstraintSubType_User3         = 15,
		ConstraintSubType_User4         = 16,
		_JPH_ConstraintSubType_Count    = 17,
		_JPH_ConstraintSubType_Force32  = 2147483647,
	}
	ConstraintSpace :: enum i32 {
		ConstraintSpace_LocalToBodyCOM = 0,
		ConstraintSpace_WorldSpace     = 1,
		_JPH_ConstraintSpace_Count     = 2,
		_JPH_ConstraintSpace_Force32   = 2147483647,
	}
	MotionQuality :: enum i32 {
		MotionQuality_Discrete     = 0,
		MotionQuality_LinearCast   = 1,
		_JPH_MotionQuality_Count   = 2,
		_JPH_MotionQuality_Force32 = 2147483647,
	}
	OverrideMassProperties :: enum i32 {
		OverrideMassProperties_CalculateMassAndInertia = 0,
		OverrideMassProperties_CalculateInertia        = 1,
		OverrideMassProperties_MassAndInertiaProvided  = 2,
		_JPH_JPH_OverrideMassProperties_Count          = 3,
		_JPH_JPH_OverrideMassProperties_Force32        = 2147483647,
	}
	AllowedDOFs :: enum i32 {
		AllowedDOFs_All          = 63,
		AllowedDOFs_TranslationX = 1,
		AllowedDOFs_TranslationY = 2,
		AllowedDOFs_TranslationZ = 4,
		AllowedDOFs_RotationX    = 8,
		AllowedDOFs_RotationY    = 16,
		AllowedDOFs_RotationZ    = 32,
		AllowedDOFs_Plane2D      = 35,
		_JPH_AllowedDOFs_Count   = 36,
		_JPH_AllowedDOFs_Force32 = 2147483647,
	}
	GroundState :: enum i32 {
		GroundState_OnGround      = 0,
		GroundState_OnSteepGround = 1,
		GroundState_NotSupported  = 2,
		GroundState_InAir         = 3,
		_JPH_GroundState_Count    = 4,
		_JPH_GroundState_Force32  = 2147483647,
	}
	BackFaceMode :: enum i32 {
		BackFaceMode_IgnoreBackFaces      = 0,
		BackFaceMode_CollideWithBackFaces = 1,
		_JPH_BackFaceMode_Count           = 2,
		_JPH_BackFaceMode_Force32         = 2147483647,
	}
	ActiveEdgeMode :: enum i32 {
		ActiveEdgeMode_CollideOnlyWithActive = 0,
		ActiveEdgeMode_CollideWithAll        = 1,
		_JPH_ActiveEdgeMode_Count            = 2,
		_JPH_ActiveEdgeMode_Force32          = 2147483647,
	}
	CollectFacesMode :: enum i32 {
		CollectFacesMode_CollectFaces = 0,
		CollectFacesMode_NoFaces      = 1,
		_JPH_CollectFacesMode_Count   = 2,
		_JPH_CollectFacesMode_Force32 = 2147483647,
	}
	MotorState :: enum i32 {
		MotorState_Off          = 0,
		MotorState_Velocity     = 1,
		MotorState_Position     = 2,
		_JPH_MotorState_Count   = 3,
		_JPH_MotorState_Force32 = 2147483647,
	}
	CollisionCollectorType :: enum i32 {
		CollisionCollectorType_AllHit       = 0,
		CollisionCollectorType_AllHitSorted = 1,
		CollisionCollectorType_ClosestHit   = 2,
		CollisionCollectorType_AnyHit       = 3,
		_JPH_CollisionCollectorType_Count   = 4,
		_JPH_CollisionCollectorType_Force32 = 2147483647,
	}
	SwingType :: enum i32 {
		SwingType_Cone         = 0,
		SwingType_Pyramid      = 1,
		_JPH_SwingType_Count   = 2,
		_JPH_SwingType_Force32 = 2147483647,
	}
	SixDOFConstraintAxis :: enum i32 {
		SixDOFConstraintAxis_TranslationX        = 0,
		SixDOFConstraintAxis_TranslationY        = 1,
		SixDOFConstraintAxis_TranslationZ        = 2,
		SixDOFConstraintAxis_RotationX           = 3,
		SixDOFConstraintAxis_RotationY           = 4,
		SixDOFConstraintAxis_RotationZ           = 5,
		_JPH_SixDOFConstraintAxis_Num            = 6,
		_JPH_SixDOFConstraintAxis_NumTranslation = 3,
		_JPH_SixDOFConstraintAxis_Force32        = 2147483647,
	}
	SpringMode :: enum i32 {
		SpringMode_FrequencyAndDamping = 0,
		SpringMode_StiffnessAndDamping = 1,
		_JPH_SpringMode_Count          = 2,
		_JPH_SpringMode_Force32        = 2147483647,
	}
	SoftBodyConstraintColor :: enum i32 {
		SoftBodyConstraintColor_ConstraintType  = 0,
		SoftBodyConstraintColor_ConstraintGroup = 1,
		SoftBodyConstraintColor_ConstraintOrder = 2,
		_JPH_SoftBodyConstraintColor_Count      = 3,
		_JPH_SoftBodyConstraintColor_Force32    = 2147483647,
	}
	BodyManager_ShapeColor :: enum i32 {
		BodyManager_ShapeColor_InstanceColor   = 0,
		BodyManager_ShapeColor_ShapeTypeColor  = 1,
		BodyManager_ShapeColor_MotionTypeColor = 2,
		BodyManager_ShapeColor_SleepColor      = 3,
		BodyManager_ShapeColor_IslandColor     = 4,
		BodyManager_ShapeColor_MaterialColor   = 5,
		_JPH_BodyManager_ShapeColor_Count      = 6,
		_JPH_BodyManager_ShapeColor_Force32    = 2147483647,
	}
	DebugRenderer_CastShadow :: enum i32 {
		DebugRenderer_CastShadow_On           = 0,
		DebugRenderer_CastShadow_Off          = 1,
		_JPH_DebugRenderer_CastShadow_Count   = 2,
		_JPH_DebugRenderer_CastShadow_Force32 = 2147483647,
	}
	DebugRenderer_DrawMode :: enum i32 {
		DebugRenderer_DrawMode_Solid        = 0,
		DebugRenderer_DrawMode_Wireframe    = 1,
		_JPH_DebugRenderer_DrawMode_Count   = 2,
		_JPH_DebugRenderer_DrawMode_Force32 = 2147483647,
	}
	Mesh_Shape_BuildQuality :: enum i32 {
		Mesh_Shape_BuildQuality_FavorRuntimePerformance = 0,
		Mesh_Shape_BuildQuality_FavorBuildSpeed         = 1,
		_JPH_Mesh_Shape_BuildQuality_Count              = 2,
		_JPH_Mesh_Shape_BuildQuality_Force32            = 2147483647,
	}

} else {

	PhysicsUpdateError :: enum u32 {
		PhysicsUpdateError_None                   = 0,
		PhysicsUpdateError_ManifoldCacheFull      = 1,
		PhysicsUpdateError_BodyPairCacheFull      = 2,
		PhysicsUpdateError_ContactConstraintsFull = 4,
		_JPH_PhysicsUpdateError_Count             = 5,
		_JPH_PhysicsUpdateError_Force32           = 2147483647,
	}
	BodyType :: enum u32 {
		BodyType_Rigid        = 0,
		BodyType_Soft         = 1,
		_JPH_BodyType_Count   = 2,
		_JPH_BodyType_Force32 = 2147483647,
	}
	MotionType :: enum u32 {
		MotionType_Static       = 0,
		MotionType_Kinematic    = 1,
		MotionType_Dynamic      = 2,
		_JPH_MotionType_Count   = 3,
		_JPH_MotionType_Force32 = 2147483647,
	}
	Activation :: enum u32 {
		Activation_Activate     = 0,
		Activation_DontActivate = 1,
		_JPH_Activation_Count   = 2,
		_JPH_Activation_Force32 = 2147483647,
	}
	ValidateResult :: enum u32 {
		ValidateResult_AcceptAllContactsForThisBodyPair = 0,
		ValidateResult_AcceptContact                    = 1,
		ValidateResult_RejectContact                    = 2,
		ValidateResult_RejectAllContactsForThisBodyPair = 3,
		_JPH_ValidateResult_Count                       = 4,
		_JPH_ValidateResult_Force32                     = 2147483647,
	}
	ShapeType :: enum u32 {
		ShapeType_Convex       = 0,
		ShapeType_Compound     = 1,
		ShapeType_Decorated    = 2,
		ShapeType_Mesh         = 3,
		ShapeType_HeightField  = 4,
		ShapeType_SoftBody     = 5,
		ShapeType_User1        = 6,
		ShapeType_User2        = 7,
		ShapeType_User3        = 8,
		ShapeType_User4        = 9,
		_JPH_ShapeType_Count   = 10,
		_JPH_ShapeType_Force32 = 2147483647,
	}
	ShapeSubType :: enum u32 {
		ShapeSubType_Sphere             = 0,
		ShapeSubType_Box                = 1,
		ShapeSubType_Triangle           = 2,
		ShapeSubType_Capsule            = 3,
		ShapeSubType_TaperedCapsule     = 4,
		ShapeSubType_Cylinder           = 5,
		ShapeSubType_ConvexHull         = 6,
		ShapeSubType_StaticCompound     = 7,
		ShapeSubType_MutableCompound    = 8,
		ShapeSubType_RotatedTranslated  = 9,
		ShapeSubType_Scaled             = 10,
		ShapeSubType_OffsetCenterOfMass = 11,
		ShapeSubType_Mesh               = 12,
		ShapeSubType_HeightField        = 13,
		ShapeSubType_SoftBody           = 14,
		_JPH_ShapeSubType_Count         = 15,
		_JPH_ShapeSubType_Force32       = 2147483647,
	}
	ConstraintType :: enum u32 {
		ConstraintType_Constraint        = 0,
		ConstraintType_TwoBodyConstraint = 1,
		_JPH_ConstraintType_Count        = 2,
		_JPH_ConstraintType_Force32      = 2147483647,
	}
	ConstraintSubType :: enum u32 {
		ConstraintSubType_Fixed         = 0,
		ConstraintSubType_Point         = 1,
		ConstraintSubType_Hinge         = 2,
		ConstraintSubType_Slider        = 3,
		ConstraintSubType_Distance      = 4,
		ConstraintSubType_Cone          = 5,
		ConstraintSubType_SwingTwist    = 6,
		ConstraintSubType_SixDOF        = 7,
		ConstraintSubType_Path          = 8,
		ConstraintSubType_Vehicle       = 9,
		ConstraintSubType_RackAndPinion = 10,
		ConstraintSubType_Gear          = 11,
		ConstraintSubType_Pulley        = 12,
		ConstraintSubType_User1         = 13,
		ConstraintSubType_User2         = 14,
		ConstraintSubType_User3         = 15,
		ConstraintSubType_User4         = 16,
		_JPH_ConstraintSubType_Count    = 17,
		_JPH_ConstraintSubType_Force32  = 2147483647,
	}
	ConstraintSpace :: enum u32 {
		ConstraintSpace_LocalToBodyCOM = 0,
		ConstraintSpace_WorldSpace     = 1,
		_JPH_ConstraintSpace_Count     = 2,
		_JPH_ConstraintSpace_Force32   = 2147483647,
	}
	MotionQuality :: enum u32 {
		MotionQuality_Discrete     = 0,
		MotionQuality_LinearCast   = 1,
		_JPH_MotionQuality_Count   = 2,
		_JPH_MotionQuality_Force32 = 2147483647,
	}
	OverrideMassProperties :: enum u32 {
		OverrideMassProperties_CalculateMassAndInertia = 0,
		OverrideMassProperties_CalculateInertia        = 1,
		OverrideMassProperties_MassAndInertiaProvided  = 2,
		_JPH_JPH_OverrideMassProperties_Count          = 3,
		_JPH_JPH_OverrideMassProperties_Force32        = 2147483647,
	}
	AllowedDOFs :: enum u32 {
		AllowedDOFs_All          = 63,
		AllowedDOFs_TranslationX = 1,
		AllowedDOFs_TranslationY = 2,
		AllowedDOFs_TranslationZ = 4,
		AllowedDOFs_RotationX    = 8,
		AllowedDOFs_RotationY    = 16,
		AllowedDOFs_RotationZ    = 32,
		AllowedDOFs_Plane2D      = 35,
		_JPH_AllowedDOFs_Count   = 36,
		_JPH_AllowedDOFs_Force32 = 2147483647,
	}
	GroundState :: enum u32 {
		GroundState_OnGround      = 0,
		GroundState_OnSteepGround = 1,
		GroundState_NotSupported  = 2,
		GroundState_InAir         = 3,
		_JPH_GroundState_Count    = 4,
		_JPH_GroundState_Force32  = 2147483647,
	}
	BackFaceMode :: enum u32 {
		BackFaceMode_IgnoreBackFaces      = 0,
		BackFaceMode_CollideWithBackFaces = 1,
		_JPH_BackFaceMode_Count           = 2,
		_JPH_BackFaceMode_Force32         = 2147483647,
	}
	ActiveEdgeMode :: enum u32 {
		ActiveEdgeMode_CollideOnlyWithActive = 0,
		ActiveEdgeMode_CollideWithAll        = 1,
		_JPH_ActiveEdgeMode_Count            = 2,
		_JPH_ActiveEdgeMode_Force32          = 2147483647,
	}
	CollectFacesMode :: enum u32 {
		CollectFacesMode_CollectFaces = 0,
		CollectFacesMode_NoFaces      = 1,
		_JPH_CollectFacesMode_Count   = 2,
		_JPH_CollectFacesMode_Force32 = 2147483647,
	}
	MotorState :: enum u32 {
		MotorState_Off          = 0,
		MotorState_Velocity     = 1,
		MotorState_Position     = 2,
		_JPH_MotorState_Count   = 3,
		_JPH_MotorState_Force32 = 2147483647,
	}
	CollisionCollectorType :: enum u32 {
		CollisionCollectorType_AllHit       = 0,
		CollisionCollectorType_AllHitSorted = 1,
		CollisionCollectorType_ClosestHit   = 2,
		CollisionCollectorType_AnyHit       = 3,
		_JPH_CollisionCollectorType_Count   = 4,
		_JPH_CollisionCollectorType_Force32 = 2147483647,
	}
	SwingType :: enum u32 {
		SwingType_Cone         = 0,
		SwingType_Pyramid      = 1,
		_JPH_SwingType_Count   = 2,
		_JPH_SwingType_Force32 = 2147483647,
	}
	SixDOFConstraintAxis :: enum u32 {
		SixDOFConstraintAxis_TranslationX        = 0,
		SixDOFConstraintAxis_TranslationY        = 1,
		SixDOFConstraintAxis_TranslationZ        = 2,
		SixDOFConstraintAxis_RotationX           = 3,
		SixDOFConstraintAxis_RotationY           = 4,
		SixDOFConstraintAxis_RotationZ           = 5,
		_JPH_SixDOFConstraintAxis_Num            = 6,
		_JPH_SixDOFConstraintAxis_NumTranslation = 3,
		_JPH_SixDOFConstraintAxis_Force32        = 2147483647,
	}
	SpringMode :: enum u32 {
		SpringMode_FrequencyAndDamping = 0,
		SpringMode_StiffnessAndDamping = 1,
		_JPH_SpringMode_Count          = 2,
		_JPH_SpringMode_Force32        = 2147483647,
	}
	SoftBodyConstraintColor :: enum u32 {
		SoftBodyConstraintColor_ConstraintType  = 0,
		SoftBodyConstraintColor_ConstraintGroup = 1,
		SoftBodyConstraintColor_ConstraintOrder = 2,
		_JPH_SoftBodyConstraintColor_Count      = 3,
		_JPH_SoftBodyConstraintColor_Force32    = 2147483647,
	}
	BodyManager_ShapeColor :: enum u32 {
		BodyManager_ShapeColor_InstanceColor   = 0,
		BodyManager_ShapeColor_ShapeTypeColor  = 1,
		BodyManager_ShapeColor_MotionTypeColor = 2,
		BodyManager_ShapeColor_SleepColor      = 3,
		BodyManager_ShapeColor_IslandColor     = 4,
		BodyManager_ShapeColor_MaterialColor   = 5,
		_JPH_BodyManager_ShapeColor_Count      = 6,
		_JPH_BodyManager_ShapeColor_Force32    = 2147483647,
	}
	DebugRenderer_CastShadow :: enum u32 {
		DebugRenderer_CastShadow_On           = 0,
		DebugRenderer_CastShadow_Off          = 1,
		_JPH_DebugRenderer_CastShadow_Count   = 2,
		_JPH_DebugRenderer_CastShadow_Force32 = 2147483647,
	}
	DebugRenderer_DrawMode :: enum u32 {
		DebugRenderer_DrawMode_Solid        = 0,
		DebugRenderer_DrawMode_Wireframe    = 1,
		_JPH_DebugRenderer_DrawMode_Count   = 2,
		_JPH_DebugRenderer_DrawMode_Force32 = 2147483647,
	}
	Mesh_Shape_BuildQuality :: enum u32 {
		Mesh_Shape_BuildQuality_FavorRuntimePerformance = 0,
		Mesh_Shape_BuildQuality_FavorBuildSpeed         = 1,
		_JPH_Mesh_Shape_BuildQuality_Count              = 2,
		_JPH_Mesh_Shape_BuildQuality_Force32            = 2147483647,
	}
}

when (ODIN_OS == .Linux) {
	foreign import jolt_runic "joltc-zig/zig-out/lib/linux/libjoltc.so"
} else when (ODIN_OS == .Windows) {
	foreign import jolt_runic "joltc-zig/zig-out/lib/windows/joltc.lib"
} else {
	foreign import jolt_runic "joltc-zig/zig-out/lib/macos_x86_64/libjoltc.dylib"
}
