package jolt

import c "core:c"

import "base:runtime"
import "core:fmt"
import "core:testing"


when ODIN_OS == .Linux {
	foreign import lib {"libjoltc.so", "system:stdc++", "system:pthread"}
} else when ODIN_OS == .Windows {
	foreign import lib {"joltc.dll", "system:stdc++", "system:pthread"}
} else when ODIN_OS == .Darwin {
	foreign import lib {"libjoltc.dynlib", "system:stdc++", "system:pthread"}
}

//-------------------------------------------------------------------------------------------------
// Constants
//-------------------------------------------------------------------------------------------------
DEFAULT_COLLISION_TOLERANCE :: 1.0e-4
DEFAULT_PENETRATION_TOLERANCE :: 1.0e-4
DEFAULT_CONVEX_RADIUS :: 0.05
CAPSULE_PROJECTION_SLOP :: 0.02
MAX_PHYSICS_JOBS :: 2048
MAX_PHYSICS_BARRIERS :: 2048

BodyID :: c.uint32_t
SubShapeID :: c.uint32_t
ObjectLayer :: c.uint16_t
BroadPhaseLayer :: c.uint8_t
CharacterID :: c.uint32_t

//-------------------------------------------------------------------------------------------------
// Types
//-------------------------------------------------------------------------------------------------
Vec3 :: [3]c.float
Vec4 :: [4]c.float
Quat :: quaternion128
Matrix4x4 :: matrix[4, 4]c.float

USE_DOUBLE_PRECISION :: #config(USE_DOUBLE_PRECISION, false)

when USE_DOUBLE_PRECISION {
	RVec3 :: [3]c.double
	RMatrix4x4 :: matrix[4, 4]c.double
} else {
	RVec3 :: Vec3
	RMatrix4x4 :: Matrix4x4
}

Color :: c.uint32_t

//-------------------------------------------------------------------------------------------------
// Enums
//-------------------------------------------------------------------------------------------------
PhysicsUpdateError :: enum (c.uint32_t) {
	None                   = 0,
	ManifoldCacheFull      = 1 << 0,
	BodyPairCacheFull      = 1 << 1,
	ContactConstraintsFull = 1 << 2,
}

BodyType :: enum (c.uint32_t) {
	Rigid = 0,
	Soft  = 1,
}

MotionType :: enum (c.uint32_t) {
	Static    = 0,
	Kinematic = 1,
	Dynamic   = 2,
}

Activation :: enum (c.uint32_t) {
	Activate     = 0,
	DontActivate = 1,
}

ValidateResult :: enum (c.uint32_t) {
	AcceptAllContactsForThisBodyPair = 0,
	AcceptContact                    = 1,
	RejectContact                    = 2,
	RejectAllContactsForThisBodyPair = 3,
}

ShapeType :: enum (c.uint32_t) {
	Convex      = 0,
	Compound    = 1,
	Decorated   = 2,
	Mesh        = 3,
	HeightField = 4,
	SoftBody    = 5,
	User1       = 6,
	User2       = 7,
	User3       = 8,
	User4       = 9,
}

ShapeSubType :: enum (c.uint32_t) {
	Sphere             = 0,
	Box                = 1,
	Triangle           = 2,
	Capsule            = 3,
	TaperedCapsule     = 4,
	Cylinder           = 5,
	ConvexHull         = 6,
	StaticCompound     = 7,
	MutableCompound    = 8,
	RotatedTranslated  = 9,
	Scaled             = 10,
	OffsetCenterOfMass = 11,
	Mesh               = 12,
	HeightField        = 13,
	SoftBody           = 14,
}

ConstraintType :: enum (c.uint32_t) {
	Constraint        = 0,
	TwoBodyConstraint = 1,
}

ConstraintSubType :: enum (c.uint32_t) {
	Fixed         = 0,
	Point         = 1,
	Hinge         = 2,
	Slider        = 3,
	Distance      = 4,
	Cone          = 5,
	SwingTwist    = 6,
	SixDOF        = 7,
	Path          = 8,
	Vehicle       = 9,
	RackAndPinion = 10,
	Gear          = 11,
	Pulley        = 12,
	User1         = 13,
	User2         = 14,
	User3         = 15,
	User4         = 16,
}

ConstraintSpace :: enum (c.uint32_t) {
	LocalToBodyCOM = 0,
	WorldSpace     = 1,
}

MotionQuality :: enum (c.uint32_t) {
	Discrete   = 0,
	LinearCast = 1,
}

OverrideMassProperties :: enum (c.uint32_t) {
	CalculateMassAndInertia = 0,
	CalculateInertia        = 1,
	MassAndInertiaProvided  = 2,
}

AllowedDOFs :: enum (c.uint32_t) {
	All          = 0b111111,
	TranslationX = 0b000001,
	TranslationY = 0b000010,
	TranslationZ = 0b000100,
	RotationX    = 0b001000,
	RotationY    = 0b010000,
	RotationZ    = 0b100000,
	Plane2D      = TranslationX | TranslationY | RotationZ,
}

GroundState :: enum (c.uint32_t) {
	OnGround      = 0,
	OnSteepGround = 1,
	NotSupported  = 2,
	InAir         = 3,
}

BackFaceMode :: enum (c.uint32_t) {
	IgnoreBackFaces      = 0,
	CollideWithBackFaces = 1,
}

ActiveEdgeMode :: enum (c.uint32_t) {
	CollideOnlyWithActive = 0,
	CollideWithAll        = 1,
}

CollectFacesMode :: enum (c.uint32_t) {
	CollectFaces = 0,
	NoFaces      = 1,
}

MotorState :: enum (c.uint32_t) {
	Off      = 0,
	Velocity = 1,
	Position = 2,
}

CollisionCollectorType :: enum (c.uint32_t) {
	AllHit       = 0,
	AllHitSorted = 1,
	ClosestHit   = 2,
	AnyHit       = 3,
}

SwingType :: enum (c.uint32_t) {
	Cone    = 0,
	Pyramid = 1,
}

SixDOFConstraintAxis :: enum (c.uint32_t) {
	TranslationX,
	TranslationY,
	TranslationZ,
	RotationX,
	RotationY,
	RotationZ,
}
SIX_DOF_CONSTRAINT_AXIS_NUM :: 6
SIX_DOF_CONSTRAINT_AXIS_NUM_TRANSLATION :: 3

SpringMode :: enum (c.uint32_t) {
	FrequencyAndDamping = 0,
	StiffnessAndDamping = 1,
}

SoftBodyConstraintColor :: enum (c.uint32_t) {
	ConstraintType  = 0,
	ConstraintGroup = 1,
	ConstraintOrder = 2,
}

BodyManagerShapeColor :: enum (c.uint32_t) {
	InstanceColor   = 0,
	ShapeTypeColor  = 1,
	MotionTypeColor = 2,
	SleepColor      = 3,
	IslandColor     = 4,
	MaterialColor   = 5,
}

DebugRendererCastShadow :: enum (c.uint32_t) {
	On  = 0,
	Off = 1,
}

DebugRendererDrawMode :: enum (c.uint32_t) {
	Solid     = 0,
	Wireframe = 1,
}

TransmissionMode :: enum (c.uint8_t) {
	Auto,
	Manual,
}

//-------------------------------------------------------------------------------------------------
// Functions
//-------------------------------------------------------------------------------------------------
CastRayResultCallback :: #type proc "c" (ctx: rawptr, result: ^RayCastResult)
RayCastBodyResultCallback :: #type proc "c" (ctx: rawptr, result: ^BroadPhaseCastResult)
CollideShapeBodyResultCallback :: #type proc "c" (ctx: rawptr, result: BodyID)
CollidePointResultCallback :: #type proc "c" (ctx: rawptr, result: ^CollidePointResult)
CollideShapeResultCallback :: #type proc "c" (ctx: rawptr, result: ^CollideShapeResult)
CastShapeResultCallback :: #type proc "c" (ctx: rawptr, result: ^ShapeCastResult)

CastRayCollectorCallback :: #type proc(ctx: rawptr, result: ^RayCastResult) -> c.float
RayCastBodyCollectorCallback :: #type proc(ctx: rawptr, result: ^BroadPhaseCastResult) -> c.float
CollideShapeBodyCollectorCallback :: #type proc(ctx: rawptr, result: BodyID) -> c.float
CollidePointCollectorCallback :: #type proc(ctx: rawptr, result: ^CollidePointResult) -> c.float
CollideShapeCollectorCallback :: #type proc(ctx: rawptr, result: ^CollideShapeResult) -> c.float
CastShapeCollectorCallback :: #type proc(ctx: rawptr, result: ^ShapeCastResult) -> c.float

TraceFunc :: #type proc "c" (mssage: cstring)
AssertFailureFunc :: #type proc "c" (
	expression: cstring,
	mssage: cstring,
	file: cstring,
	line: c.uint32_t,
) -> c.bool

JobFunction :: #type proc "c" (arg: rawptr)
QueueJobCallback :: #type proc "c" (ctx: rawptr, job: JobFunction, arg: rawptr)
QueueJobsCallback :: #type proc "c" (
	ctx: rawptr,
	job: JobFunction,
	args: [^]rawptr,
	count: c.uint32_t,
)

//-------------------------------------------------------------------------------------------------
// Opaque types
//-------------------------------------------------------------------------------------------------
JobSystem :: struct {}

BroadPhaseLayerInterface :: struct {}
ObjectVsBroadPhaseLayerFilter :: struct {}
ObjectLayerPairFilter :: struct {}
BroadPhaseLayerFilter :: struct {}
ObjectLayerFilter :: struct {}
PhysicsSystem :: struct {}
BodyFilter :: struct {}
ShapeFilter :: struct {}
PhysicsMaterial :: struct {}
ShapeSettings :: struct {}
ConvexShapeSettings :: struct {}
SphereShapeSettings :: struct {}
BoxShapeSettings :: struct {}
PlaneShapeSettings :: struct {}
TriangleShapeSettings :: struct {}
CapsuleShapeSettings :: struct {}
TaperedCapsuleShapeSettings :: struct {}
CylinderShapeSettings :: struct {}
TaperedCylinderShapeSettings :: struct {}
ConvexHullShapeSettings :: struct {}
CompoundShapeSettings :: struct {}
StaticCompoundShapeSettings :: struct {}
MutableCompoundShapeSettings :: struct {}
MeshShapeSettings :: struct {}
HeightFieldShapeSettings :: struct {}
RotatedTranslatedShapeSettings :: struct {}
ScaledShapeSettings :: struct {}
OffsetCenterOfMassShapeSettings :: struct {}
EmptyShapeSettings :: struct {}
Shape :: struct {}
ConvexShape :: struct {}
SphereShape :: struct {}
BoxShape :: struct {}
PlaneShape :: struct {}
CapsuleShape :: struct {}
CylinderShape :: struct {}
TaperedCylinderShape :: struct {}
TriangleShape :: struct {}
TaperedCapsuleShape :: struct {}
ConvexHullShape :: struct {}
CompoundShape :: struct {}
StaticCompoundShape :: struct {}
MutableCompoundShape :: struct {}
MeshShape :: struct {}
HeightFieldShape :: struct {}
DecoratedShape :: struct {}
RotatedTranslatedShape :: struct {}
ScaledShape :: struct {}
OffsetCenterOfMassShape :: struct {}
EmptyShape :: struct {}
BodyCreationSettings :: struct {}
SoftBodyCreationSettings :: struct {}
BodyInterface :: struct {}
BodyLockInterface :: struct {}
BroadPhaseQuery :: struct {}
NarrowPhaseQuery :: struct {}
MotionProperties :: struct {}
Body :: struct {}
Constraint :: struct {}
TwoBodyConstraint :: struct {}
FixedConstraint :: struct {}
DistanceConstraint :: struct {}
PointConstraint :: struct {}
HingeConstraint :: struct {}
SliderConstraint :: struct {}
ConeConstraint :: struct {}
SwingTwistConstraint :: struct {}
SixDOFConstraint :: struct {}
GearConstraint :: struct {}
ContactListener :: struct {}
ContactManifold :: struct {}
ContactSettings :: struct {}
BodyActivationListener :: struct {}
BodyDrawFilter :: struct {}
SharedMutex :: struct {}
DebugRenderer :: struct {}
BodyLockMultiRead :: struct {}
BodyLockMultiWrite :: struct {}
CharacterBase :: struct {}
Character :: struct {}
CharacterContactListener :: struct {}
CharacterVirtual :: struct {}
CharacterVsCharacterCollision :: struct {}

VehicleTransmissionSettings :: struct {}
VehicleTransmission :: struct {}

//-------------------------------------------------------------------------------------------------
// Structures
//-------------------------------------------------------------------------------------------------
Plane :: struct {
	normal:   Vec3,
	distance: c.float,
}

AABox :: struct {
	min: Vec3,
	max: Vec3,
}

Triangle :: struct {
	v1:            Vec3,
	v2:            Vec3,
	v3:            Vec3,
	materialIndex: c.uint32_t,
}

IndexedTriangleNoMaterial :: struct {
	i1: c.uint32_t,
	i2: c.uint32_t,
	i3: c.uint32_t,
}

IndexedTriangle :: struct {
	i1:            c.uint32_t,
	i2:            c.uint32_t,
	i3:            c.uint32_t,
	materialIndex: c.uint32_t,
	userData:      c.uint32_t,
}

MassProperties :: struct {
	mass:    c.float,
	inertia: Matrix4x4,
}

CollideSettingsBase :: struct {
	activeEdgeMode:              ActiveEdgeMode,
	collectFacesMode:            CollectFacesMode,
	collisionTolerance:          c.float,
	penetrationTolerance:        c.float,
	activeEdgeMovementDirection: Vec3,
}

CollideShapeSettings :: struct {
	base:                  CollideSettingsBase,
	maxSeparationDistance: c.float,
	backFaceMode:          BackFaceMode,
}

ShapeCastSettings :: struct {
	base:                            CollideSettingsBase,
	backFaceModeTriangles:           BackFaceMode,
	backFaceModeConvex:              BackFaceMode,
	useShrunkenShapeAndConvexRadius: c.bool,
	returnDeepestPoint:              c.bool,
}

RayCastSettings :: struct {
	backFaceModeTriangles: BackFaceMode,
	backFaceModeConvex:    BackFaceMode,
	treatConvexAsSolid:    c.bool,
}

SpringSettings :: struct {
	mode:                 SpringMode,
	frequencyOrStiffness: c.float,
	damping:              c.float,
}

MotorSettings :: struct {
	springSettings: SpringSettings,
	minForceLimit:  c.float,
	maxForceLimit:  c.float,
	minTorqueLimit: c.float,
	maxTorqueLimit: c.float,
}

SubShapeIDPair :: struct {
	Body1ID:     BodyID,
	subShapeID1: SubShapeID,
	Body2ID:     BodyID,
	subShapeID2: SubShapeID,
}

BroadPhaseCastResult :: struct {
	bodyID:   BodyID,
	fraction: c.float,
}

RayCastResult :: struct {
	bodyID:      BodyID,
	fraction:    c.float,
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
	penetrationDepth: c.float,
	subShapeID1:      SubShapeID,
	subShapeID2:      SubShapeID,
	bodyID2:          BodyID,
}

ShapeCastResult :: struct {
	contactPointOn1:  Vec3,
	contactPointOn2:  Vec3,
	penetrationAxis:  Vec3,
	penetrationDepth: c.float,
	subShapeID1:      SubShapeID,
	subShapeID2:      SubShapeID,
	bodyID2:          BodyID,
	fraction:         c.float,
	isBackFaceHit:    c.bool,
}

DrawSettings :: struct {
	drawGetSupportFunction:        c.bool,
	drawSupportDirection:          c.bool,
	drawGetSupportingFace:         c.bool,
	drawShape:                     c.bool,
	drawShapeWireframe:            c.bool,
	drawShapeColor:                BodyManagerShapeColor,
	drawBoundingBox:               c.bool,
	drawCenterOfMassTransform:     c.bool,
	drawWorldTransform:            c.bool,
	drawVelocity:                  c.bool,
	drawMassAndInertia:            c.bool,
	drawSleepStats:                c.bool,
	drawSoftBodyVertices:          c.bool,
	drawSoftBodyVertexVelocities:  c.bool,
	drawSoftBodyEdgeConstraints:   c.bool,
	drawSoftBodyBendConstraints:   c.bool,
	drawSoftBodyVolumeConstraints: c.bool,
	drawSoftBodySkinConstraints:   c.bool,
	drawSoftBodyLRAConstraints:    c.bool,
	drawSoftBodyPredictedBounds:   c.bool,
	drawSoftBodyConstraintColor:   SoftBodyConstraintColor,
}

ConstraintSettings :: struct {
	enabled:                  c.bool,
	constraintPriority:       c.uint32_t,
	numVelocityStepsOverride: c.uint32_t,
	numPositionStepsOverride: c.uint32_t,
	drawConstraintSize:       c.float,
	userData:                 c.uint64_t,
}

FixedConstraintSettings :: struct {
	base:            ConstraintSettings,
	space:           ConstraintSpace,
	autoDetectPoint: c.bool,
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
	minDistance:          c.float,
	maxDistance:          c.float,
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
	limitsMin:            c.float,
	limitsMax:            c.float,
	limitsSpringSettings: SpringSettings,
	maxFrictionTorque:    c.float,
	motorSettings:        MotorSettings,
}

SliderConstraintSettings :: struct {
	base:                 ConstraintSettings,
	space:                ConstraintSpace,
	autoDetectPoint:      c.bool,
	point1:               RVec3,
	sliderAxis1:          Vec3,
	normalAxis1:          Vec3,
	point2:               RVec3,
	sliderAxis2:          Vec3,
	normalAxis2:          Vec3,
	limitsMin:            c.float,
	limitsMax:            c.float,
	limitsSpringSettings: SpringSettings,
	maxFrictionForce:     c.float,
	motorSettings:        MotorSettings,
}

ConeConstraintSettings :: struct {
	base:          ConstraintSettings,
	space:         ConstraintSpace,
	point1:        RVec3,
	twistAxis1:    Vec3,
	point2:        RVec3,
	twistAxis2:    Vec3,
	halfConeAngle: c.float,
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
	normalHalfConeAngle: c.float,
	planeHalfConeAngle:  c.float,
	twistMinAngle:       c.float,
	twistMaxAngle:       c.float,
	maxFrictionTorque:   c.float,
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
	maxFriction:          [SIX_DOF_CONSTRAINT_AXIS_NUM]c.float,
	swingType:            SwingType,
	limitMin:             [SIX_DOF_CONSTRAINT_AXIS_NUM]c.float,
	limitMax:             [SIX_DOF_CONSTRAINT_AXIS_NUM]c.float,
	limitsSpringSettings: [SIX_DOF_CONSTRAINT_AXIS_NUM_TRANSLATION]SpringSettings,
	motorSettings:        [SIX_DOF_CONSTRAINT_AXIS_NUM]MotorSettings,
}

GearConstraintSettings :: struct {
	base:       ConstraintSettings,
	space:      ConstraintSpace,
	hingeAxis1: Vec3,
	hingeAxis2: Vec3,
	ratio:      c.float,
}

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
	walkStairsMinStepForward:         c.float,
	walkStairsStepForwardTest:        c.float,
	walkStairsCosAngleForwardContact: c.float,
	walkStairsStepDownExtra:          Vec3,
}

CharacterBaseSettings :: struct {
	up:                          Vec3,
	supportingVolume:            Plane,
	maxSlopeAngle:               c.float,
	enhancedInternalEdgeRemoval: c.bool,
	shape:                       ^Shape,
}

CharacterSettings :: struct {
	using base:    CharacterBaseSettings,
	layer:         ObjectLayer,
	mass:          c.float,
	friction:      c.float,
	gravityFactor: c.float,
	allowedDOFs:   AllowedDOFs,
}

CharacterVirtualSettings :: struct {
	using base:                CharacterBaseSettings,
	ID:                        CharacterID,
	mass:                      c.float,
	maxStrength:               c.float,
	shapeOffset:               Vec3,
	backFaceMode:              BackFaceMode,
	predictiveContactDistance: c.float,
	maxCollisionIterations:    u32,
	maxConstraintIterations:   u32,
	minTimeRemaining:          c.float,
	collisionTolerance:        c.float,
	characterPadding:          c.float,
	maxNumHits:                u32,
	hitReductionCosMaxAngle:   c.float,
	penetrationRecoverySpeed:  c.float,
	innerBodyShape:            ^Shape,
	innerBodyLayer:            ObjectLayer,
}

CharacterContactSettings :: struct {
	canPushCharacter:   c.bool,
	canReceiveImpulses: c.bool,
}

PhysicsSystemSettings :: struct {
	maxBodies:                     c.uint32_t, //10240
	numBodyMutexes:                c.uint32_t, //0
	maxBodyPairs:                  c.uint32_t, //65536
	maxContactConstraints:         c.uint32_t, //10240
	_padding:                      c.uint32_t,
	broadPhaseLayerInterface:      ^BroadPhaseLayerInterface,
	objectLayerPairFilter:         ^ObjectLayerPairFilter,
	objectVsBroadPhaseLayerFilter: ^ObjectVsBroadPhaseLayerFilter,
}

PhysicsSettings :: struct {
	maxInFlightBodyPairs:                 c.int,
	stepListenersBatchSize:               c.int,
	stepListenerBatchesPerJob:            c.int,
	baumgarte:                            c.float,
	speculativeContactDistance:           c.float,
	penetrationSlop:                      c.float,
	linearCastThreshold:                  c.float,
	linearCastMaxPenetration:             c.float,
	manifoldToleranceSq:                  c.float,
	maxPenetrationDistance:               c.float,
	bodyPairCacheMaxDeltaPositionSq:      c.float,
	bodyPairCacheCosMaxDeltaRotationDiv2: c.float,
	contactNormalCosMaxDeltaRotation:     c.float,
	contactPointPreserveLambdaMaxDistSq:  c.float,
	numVelocitySteps:                     c.uint32_t,
	numPositionSteps:                     c.uint32_t,
	minVelocityForRestitution:            c.float,
	timeBeforeSleep:                      c.float,
	pointVelocitySleepThreshold:          c.float,
	deterministicSimulation:              c.bool,
	constraintWarmStart:                  c.bool,
	useBodyPairContactCache:              c.bool,
	useManifoldReduction:                 c.bool,
	useLargeIslandSplitter:               c.bool,
	allowSleeping:                        c.bool,
	checkActiveEdges:                     c.bool,
}


JobSystemThreadPoolConfig :: struct {
	maxJobs:     c.uint32_t,
	maxBarriers: c.uint32_t,
	numThreads:  c.int32_t,
}

JobSystemConfig :: struct {
	ctx:            rawptr,
	queueJob:       QueueJobCallback,
	queueJobs:      QueueJobsCallback,
	maxConcurrency: c.uint32_t,
	maxBarriers:    c.uint32_t,
}


//---------------------------------------------------------------------------------------------------------------------
// Procs
//---------------------------------------------------------------------------------------------------------------------
BroadPhaseLayerFilter_Procs :: struct {
	ShouldCollide: proc "c" (useData: rawptr, layer: BroadPhaseLayer) -> c.bool,
}

ObjectLayerFilter_Procs :: struct {
	ShouldCollide: proc "c" (userData: rawptr, layer: ObjectLayer) -> c.bool,
}

BodyFilter_Procs :: struct {
	ShouldCollide:       proc "c" (userData: rawptr, bodyId: BodyID) -> c.bool,
	ShouldCollideLocked: proc "c" (userData: rawptr, bodyId: ^BodyID) -> c.bool,
}

ShapeFilter_Procs :: struct {
	ShouldCollide:  proc "c" (
		userData: rawptr,
		shape2: ^Shape,
		subShapeIDOfShape2: ^SubShapeID,
	) -> c.bool,
	ShouldCollide2: proc "c" (
		userData: rawptr,
		shape1: ^Shape,
		subShapeIDOfShape1: ^SubShapeID,
		shape2: ^Shape,
		subShapeIDOfShape2: ^SubShapeID,
	) -> c.bool,
}

ContactListener_Procs :: struct {
	OnContactValidate:  proc "c" (
		userData: rawptr,
		body1: ^Body,
		body2: ^Body,
		baseOffset: ^RVec3,
		collisionResult: ^CollideShapeResult,
	) -> ValidateResult,
	OnContactAdded:     proc "c" (
		userData: rawptr,
		body1: ^Body,
		body2: ^Body,
		manifold: ^ContactManifold,
		settings: ^ContactSettings,
	),
	OnContactPersisted: proc "c" (
		userData: rawptr,
		body1: ^Body,
		body2: ^Body,
		manifold: ^ContactManifold,
		settings: ^ContactSettings,
	),
	OnContactRemoved:   proc "c" (userData: rawptr, subShapePair: ^SubShapeIDPair),
}

BodyActivationListener_Procs :: struct {
	OnBodyActivated:   proc "c" (userData: rawptr, bodyID: BodyID, bodyUserData: c.uint64_t),
	OnBodyDeactivated: proc "c" (userData: rawptr, bodyID: BodyID, bodyUserData: c.uint64_t),
}

BodyDrawFilter_Procs :: struct {
	ShouldDraw: proc "c" (userData: rawptr, body: ^Body) -> c.bool,
}

CharacterContactListener_Procs :: struct {
	OnAdjustBodyVelocity:        proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		body2: ^Body,
		ioLinearVelocity: ^Vec3,
		ioAngularVelocity: ^Vec3,
	),
	OnContactValidate:           proc "c" (
		userData: rawptr,
		character: ^Character,
		bodyID2: BodyID,
		subShapeID2: SubShapeID,
	) -> c.bool,
	OnCharacterContactValidate:  proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		otherCharacter: ^CharacterVirtual,
		subShapeID2: SubShapeID,
	) -> c.bool,
	OnContactAdded:              proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		bodyID2: BodyID,
		subShapeID2: SubShapeID,
		contactPosition: ^RVec3,
		contactNormal: ^Vec3,
		ioSettings: ^CharacterContactSettings,
	),
	OnContactPersisted:          proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		bodyID2: BodyID,
		subShapeID2: SubShapeID,
		contactPosition: ^RVec3,
		contactNormal: ^Vec3,
		ioSettings: ^CharacterContactSettings,
	),
	OnContactRemoved:            proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		bodyID2: BodyID,
		subShapeID2: SubShapeID,
	),
	OnCharacterContactAdded:     proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		otherCharacter: ^CharacterVirtual,
		subShapeID2: SubShapeID,
		contactPosition: ^RVec3,
		contactNormal: ^Vec3,
		ioSettings: ^CharacterContactSettings,
	),
	OnCharacterContactPersisted: proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		otherCharacter: ^CharacterVirtual,
		subShapeID2: SubShapeID,
		contactPosition: ^RVec3,
		contactNormal: ^Vec3,
		ioSettings: ^CharacterContactSettings,
	),
	OnCharacterContactRemoved:   proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		otherCharacterID: CharacterID,
		subShapeID: SubShapeID,
	),
	OnContactSolve:              proc "c" (
		userData: rawptr,
		bodyID2: BodyID,
		subShapeID2: SubShapeID,
		contactPosition: ^RVec3,
		contactNormal: ^Vec3,
		contactVelocity: ^Vec3,
		contactMaterial: ^PhysicsMaterial,
		characterVelocity: ^Vec3,
		newCharacterVelocity: ^Vec3,
	),
	OnCharacterContactSolve:     proc "c" (
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
	),
}

CharacterVsCharacterCollision_Procs :: struct {
	CollideCharacter: proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		centerOfMassTransform: ^RMatrix4x4,
		collideShapeSettings: ^CollideShapeSettings,
		baseOffset: ^RVec3,
	),
	CastCharacter:    proc "c" (
		userData: rawptr,
		character: ^CharacterVirtual,
		centerOfMassTransform: ^RMatrix4x4,
		direction: ^Vec3,
		shapeCastSettings: ^ShapeCastSettings,
		baseOffset: ^RVec3,
	),
}
DebugRenderer_Procs :: struct {
	DrawLine:     proc "c" (userData: rawptr, from: ^RVec3, to: ^RVec3, color: Color),
	DrawTriangle: proc "c" (
		userData: rawptr,
		v1: ^RVec3,
		v2: ^RVec3,
		v3: ^RVec3,
		color: Color,
		castShadow: DebugRendererCastShadow,
	),
	DrawText3D:   proc "c" (
		userData: rawptr,
		position: ^RVec3,
		str: cstring,
		color: Color,
		height: c.float,
	),
}

@(default_calling_convention = "c", link_prefix = "JPH_")
foreign lib {
	JobSystemThreadPool_Create :: proc(config: ^JobSystemThreadPoolConfig) -> ^JobSystem ---
	JobSystemCallback_Create :: proc(config: ^JobSystemConfig) -> ^JobSystem ---
	JobSystem_Destroy :: proc(jobSystem: ^JobSystem) ---

	Init :: proc() -> c.bool ---
	Shutdown :: proc() ---
	SetTraceHandler :: proc(handler: TraceFunc) ---
	SetAssertFailureHandler :: proc(handler: AssertFailureFunc) ---

	//-------------------------------------------------------------------------------------------------
	// BroadPhaseLayerInterface
	//-------------------------------------------------------------------------------------------------
	BroadPhaseLayerInterfaceMask_Create :: proc(numBroadPhaseLayers: c.uint32_t) -> ^BroadPhaseLayerInterface ---
	BroadPhaseLayerInterfaceMask_ConfigureLayer :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface, broadPhaseLayer: BroadPhaseLayer, groupsToInclude: c.uint32_t, groupsToExclude: c.uint32_t) ---

	BroadPhaseLayerInterfaceTable_Create :: proc(numObjectLayers: c.uint32_t, numBroadPhaseLayers: c.uint32_t) -> ^BroadPhaseLayerInterface ---
	BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface, objectLayer: ObjectLayer, broadPhaseLayer: BroadPhaseLayer) ---

	//-------------------------------------------------------------------------------------------------
	// ObjectLayerPairFilter
	//-------------------------------------------------------------------------------------------------
	ObjectLayerPairFilterMask_Create :: proc() -> ^ObjectLayerPairFilter ---
	ObjectLayerPairFilterMask_GetObjectLayer :: proc(group: c.uint32_t, mask: c.uint32_t) -> ObjectLayer ---
	ObjectLayerPairFilterMask_GetGroup :: proc(layer: ObjectLayer) -> c.uint32_t ---
	ObjectLayerPairFilterMask_GetMask :: proc(layer: ObjectLayer) -> c.uint32_t ---

	ObjectLayerPairFilterTable_Create :: proc(numObjectLayers: c.uint32_t) -> ^ObjectLayerPairFilter ---
	ObjectLayerPairFilterTable_DisableCollision :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---
	ObjectLayerPairFilterTable_EnableCollision :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---
	ObjectLayerPairFilterTable_ShouldCollide :: proc(objectFilter: ^ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) ---

	//-------------------------------------------------------------------------------------------------
	// ObjectVsBroadPhaseLayerFilter
	//-------------------------------------------------------------------------------------------------
	ObjectVsBroadPhaseLayerFilterMask_Create :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface) -> ^ObjectVsBroadPhaseLayerFilter ---
	ObjectVsBroadPhaseLayerFilterTable_Create :: proc(broadPhaseLayerInterface: ^BroadPhaseLayerInterface, numBroadPhaseLayers: c.uint32_t, objectLayerPairFilter: ^ObjectLayerPairFilter, numObjectLayers: c.uint32_t) -> ^ObjectVsBroadPhaseLayerFilter ---

	//-------------------------------------------------------------------------------------------------
	// DrawSettings
	//-------------------------------------------------------------------------------------------------
	DrawSettings_InitDefault :: proc(settings: ^DrawSettings) ---

	//-------------------------------------------------------------------------------------------------
	// PhysicsSystem
	//-------------------------------------------------------------------------------------------------
	PhysicsSystem_Create :: proc(settings: ^PhysicsSystemSettings) -> ^PhysicsSystem ---
	PhysicsSystem_Destroy :: proc(system: ^PhysicsSystem) ---
	PhysicsSystem_SetPhysicsSettings :: proc(system: ^PhysicsSystem, settings: ^PhysicsSettings) ---
	PhysicsSystem_GetPhysicsSettings :: proc(system: ^PhysicsSystem, result: ^PhysicsSettings) ---
	PhysicsSystem_OptimizeBroadPhase :: proc(system: ^PhysicsSystem) ---
	PhysicsSystem_Update :: proc(system: ^PhysicsSystem, deltaTime: c.float, collisionSteps: c.int, jobSystem: ^JobSystem) -> PhysicsUpdateError ---
	PhysicsSystem_GetBodyInterface :: proc(system: ^PhysicsSystem) -> ^BodyInterface ---
	PhysicsSystem_GetBodyInterfaceNoLock :: proc(system: ^PhysicsSystem) -> ^BodyInterface ---
	PhysicsSystem_GetBodyLockInterface :: proc(system: ^PhysicsSystem) -> ^BodyLockInterface ---
	PhysicsSystem_GetBodyLockInterfaceNoLock :: proc(system: ^PhysicsSystem) -> ^BodyLockInterface ---
	PhysicsSystem_GetBroadPhaseQuery :: proc(system: ^PhysicsSystem) -> ^BroadPhaseQuery ---
	PhysicsSystem_GetNarrowPhaseQuery :: proc(system: ^PhysicsSystem) -> ^NarrowPhaseQuery ---
	PhysicsSystem_GetNarrowPhaseQueryNoLock :: proc(system: ^PhysicsSystem) -> ^NarrowPhaseQuery ---
	PhysicsSystem_SetContactListener :: proc(system: ^PhysicsSystem, listener: ^ContactListener) ---
	PhysicsSystem_SetBodyActivationListener :: proc(system: ^PhysicsSystem, listener: ^BodyActivationListener) ---
	PhysicsSystem_WereBodiesInContact :: proc(system: ^PhysicsSystem, body1: BodyID, body2: BodyID) -> c.bool ---
	PhysicsSystem_GetNumBodies :: proc(system: ^PhysicsSystem) -> c.uint32_t ---
	PhysicsSystem_GetNumActiveBodies :: proc(system: ^PhysicsSystem, type: BodyType) -> c.uint32_t ---
	PhysicsSystem_GetMaxBodies :: proc(system: ^PhysicsSystem) -> c.uint32_t ---
	PhysicsSystem_GetNumConstraints :: proc(system: ^PhysicsSystem) -> c.uint32_t ---
	PhysicsSystem_SetGravity :: proc(system: ^PhysicsSystem, value: ^Vec3) ---
	PhysicsSystem_GetGravity :: proc(system: ^PhysicsSystem, result: ^Vec3) ---
	PhysicsSystem_AddConstraint :: proc(system: ^PhysicsSystem, constraint: ^Constraint) ---
	PhysicsSystem_RemoveConstraint :: proc(system: ^PhysicsSystem, constraint: ^Constraint) ---
	PhysicsSystem_AddConstraints :: proc(system: ^PhysicsSystem, constraints: ^^Constraint, count: c.uint32_t) ---
	PhysicsSystem_RemoveConstraints :: proc(system: ^PhysicsSystem, constraints: ^^Constraint, count: c.uint32_t) ---
	PhysicsSystem_GetBodies :: proc(system: ^PhysicsSystem, ids: ^BodyID, count: c.uint32_t) ---
	PhysicsSystem_GetConstraints :: proc(system: ^PhysicsSystem, constraints: [^]rawptr) ---
	PhysicsSystem_DrawBodies :: proc(system: ^PhysicsSystem, settings: ^DrawSettings, renderer: ^DebugRenderer, bodyFilter: ^BodyDrawFilter = nil) ---
	PhysicsSystem_DrawConstraints :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---
	PhysicsSystem_DrawConstraintLimits :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---
	PhysicsSystem_DrawConstraintReferenceFrame :: proc(system: ^PhysicsSystem, renderer: ^DebugRenderer) ---

	//-------------------------------------------------------------------------------------------------
	//Math
	//Odin can do all these, just added for completeness...
	//-------------------------------------------------------------------------------------------------
	Quaternion_FromTo :: proc(from: ^Vec3, to: ^Vec3, quat: ^Quat) ---
	Quat_GetAxisAngle :: proc(quat: ^Quat, outAxis: ^Vec3, outAngle: ^f32) ---
	Quat_GetEulerAngles :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_RotateAxisX :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_RotateAxisY :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_RotateAxisZ :: proc(quat: ^Quat, result: ^Vec3) ---
	Quat_Inversed :: proc(quat: ^Quat, result: ^Quat) ---
	Quat_GetPerpendicular :: proc(quat: ^Quat, result: ^Quat) ---
	Quat_GetRotationAngle :: proc(quat: ^Quat, axis: ^Vec3) -> f32 ---

	Vec3_IsClose :: proc(v1: ^Vec3, v2: ^Vec3, maxDistSq: f32) -> bool ---
	Vec3_IsNearZero :: proc(v: ^Vec3, maxDistSq: f32) -> bool ---
	Vec3_IsNormalized :: proc(v: ^Vec3, tolerance: f32) -> bool ---
	Vec3_IsNaN :: proc(v: ^Vec3) -> bool ---

	Vec3_Negate :: proc(v: ^Vec3, result: ^Vec3) ---
	Vec3_Normalized :: proc(v: ^Vec3, result: ^Vec3) ---
	Vec3_Cross :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_Abs :: proc(v: ^Vec3, result: ^Vec3) ---

	Vec3_Length :: proc(v: ^Vec3) -> f32 ---
	Vec3_LengthSquared :: proc(v: ^Vec3) -> f32 ---

	Vec3_Multiply :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_MultiplyScalar :: proc(v: ^Vec3, scalar: f32, result: ^Vec3) ---
	Vec3_DotProduct :: proc(v1: ^Vec3, v2: ^Vec3, result: ^f32) ---
	Vec3_Normalize :: proc(v: ^Vec3, result: ^Vec3) ---

	Vec3_Add :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---
	Vec3_Subtract :: proc(v1: ^Vec3, v2: ^Vec3, result: ^Vec3) ---

	Matrix4x4_Zero :: proc(result: ^Matrix4x4) ---
	Matrix4x4_Identity :: proc(result: ^Matrix4x4) ---
	Matrix4x4_Rotation :: proc(result: ^Matrix4x4, rotation: ^Quat) ---
	Matrix4x4_Translation :: proc(result: ^Matrix4x4, translation: ^Vec3) ---
	Matrix4x4_RotationTranslation :: proc(result: ^Matrix4x4, rotation: ^Quat, translation: ^Vec3) ---
	Matrix4x4_InverseRotationTranslation :: proc(result: ^Matrix4x4, rotation: ^Quat, translation: ^Vec3) ---
	Matrix4x4_Scale :: proc(result: ^Matrix4x4, scale: ^Vec3) ---

	RMatrix4x4_Zero :: proc(result: ^RMatrix4x4) ---
	RMatrix4x4_Identity :: proc(result: ^RMatrix4x4) ---
	RMatrix4x4_Rotation :: proc(result: ^RMatrix4x4, rotation: ^Quat) ---
	RMatrix4x4_Translation :: proc(result: ^RMatrix4x4, translation: ^RVec3) ---
	RMatrix4x4_RotationTranslation :: proc(result: ^RMatrix4x4, rotation: ^Quat, translation: ^RVec3) ---
	RMatrix4x4_InverseRotationTranslation :: proc(result: ^RMatrix4x4, rotation: ^Quat, translation: ^RVec3) ---
	RMatrix4x4_Scale :: proc(result: ^RMatrix4x4, scale: ^Vec3) ---

	Matrix4x4_GetAxisX :: proc(mat: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetAxisY :: proc(mat: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetAxisZ :: proc(mat: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetTranslation :: proc(mat: ^Matrix4x4, result: ^Vec3) ---
	Matrix4x4_GetQuaternion :: proc(mat: ^Matrix4x4, result: ^Quat) ---

	//-------------------------------------------------------------------------------------------------
	// Material
	//-------------------------------------------------------------------------------------------------
	PhysicsMaterial_Create :: proc(name: cstring, color: c.uint32_t) -> ^PhysicsMaterial ---
	PhysicsMaterial_Destroy :: proc(material: ^PhysicsMaterial) ---
	PhysicsMaterial_GetDebugName :: proc(material: ^PhysicsMaterial) -> ^c.char ---
	PhysicsMaterial_GetDebugColor :: proc(material: ^PhysicsMaterial) -> c.uint32_t ---

	//-------------------------------------------------------------------------------------------------
	// ShapeSettings
	//-------------------------------------------------------------------------------------------------
	ShapeSettings_Destroy :: proc(settings: ^ShapeSettings) ---
	ShapeSettings_GetUserData :: proc(settings: ^ShapeSettings) -> c.uint64_t ---
	ShapeSettings_SetUserData :: proc(settings: ^ShapeSettings, userData: c.uint64_t) ---

	//-------------------------------------------------------------------------------------------------
	// Shape
	//-------------------------------------------------------------------------------------------------
	Shape_Destroy :: proc(shape: ^Shape) ---
	Shape_GetType :: proc(shape: ^Shape) -> ShapeType ---
	Shape_GetSubType :: proc(shape: ^Shape) -> ShapeSubType ---
	Shape_GetUserData :: proc(shape: ^Shape) -> c.uint64_t ---
	Shape_SetUserData :: proc(shape: ^Shape, userData: c.uint64_t) ---
	Shape_MustBeStatic :: proc(shape: ^Shape) -> c.bool ---
	Shape_GetCenterOfMass :: proc(shape: ^Shape, result: ^Vec3) ---
	Shape_GetLocalBounds :: proc(shape: ^Shape, result: ^AABox) ---
	Shape_GetSubShapeIDBitsRecursive :: proc(shape: ^Shape) -> c.uint32_t ---
	Shape_GetWorldSpaceBounds :: proc(shape: ^Shape, centerOfMassTransform: ^RMatrix4x4, scale: ^Vec3, result: ^AABox) ---
	Shape_GetInnerRadius :: proc(shape: ^Shape) -> c.float ---
	Shape_GetMassProperties :: proc(shape: ^Shape, result: ^MassProperties) ---
	Shape_GetLeafShape :: proc(shape: ^Shape, subShapeID: SubShapeID, remainder: ^SubShapeID) -> ^Shape ---
	Shape_GetMaterial :: proc(shape: ^Shape, subShapeID: SubShapeID) -> ^PhysicsMaterial ---
	Shape_GetSurfaceNormal :: proc(shape: ^Shape, subShapeID: SubShapeID, localPosition: ^Vec3, normal: ^Vec3) ---
	Shape_GetVolume :: proc(shape: ^Shape) -> c.float ---
	Shape_IsValidScale :: proc(shape: ^Shape, scale: ^Vec3) -> c.bool ---
	Shape_MakeScaleValid :: proc(shape: ^Shape, scale: ^Vec3, result: ^Vec3) ---
	Shape_ScaleShape :: proc(shape: ^Shape, scale: ^Vec3) -> ^Shape ---
	Shape_CastRay :: proc(shape: ^Shape, origin: ^Vec3, direction: ^Vec3, hit: ^RayCastResult) -> c.bool ---
	Shape_CastRay2 :: proc(shape: ^Shape, origin: ^Vec3, direction: ^Vec3, rayCastSettings: ^RayCastSettings, collectorType: CollisionCollectorType, callback: CastRayResultCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> c.bool ---
	Shape_CollidePoint :: proc(shape: ^Shape, point: ^Vec3, shapeFilter: ^ShapeFilter) -> c.bool ---
	Shape_CollidePoint2 :: proc(shape: ^Shape, point: ^Vec3, collectorType: CollisionCollectorType, callback: CollidePointResultCallback, userData: rawptr, shapeFilter: ^ShapeFilter) -> c.bool ---

	//-------------------------------------------------------------------------------------------------
	// ConvexShape
	//-------------------------------------------------------------------------------------------------
	ConvexShapeSettings_GetDensity :: proc(shape: ^ConvexShapeSettings) -> c.float ---
	ConvexShapeSettings_SetDensity :: proc(shape: ^ConvexShapeSettings, value: c.float) ---
	ConvexShape_GetDensity :: proc(shape: ^ConvexShape) -> c.float ---
	ConvexShape_SetDensity :: proc(shape: ^ConvexShape, inDensity: c.float) ---

	//-------------------------------------------------------------------------------------------------
	// BoxShape
	//-------------------------------------------------------------------------------------------------
	BoxShapeSettings_Create :: proc(halfExtent: ^Vec3, convexRadius: c.float) -> ^BoxShapeSettings ---
	BoxShapeSettings_CreateShape :: proc(settings: ^BoxShapeSettings) -> ^BoxShape ---
	BoxShape_Create :: proc(halfExtent: ^Vec3, convexRadius: c.float = DEFAULT_CONVEX_RADIUS) -> ^BoxShape ---
	BoxShape_GetHalfExtent :: proc(shape: ^BoxShape, halfExtent: ^Vec3) ---
	BoxShape_GetConvexRadius :: proc(shape: ^BoxShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// SphereShape
	//-------------------------------------------------------------------------------------------------
	SphereShapeSettings_Create :: proc(radius: c.float) -> ^SphereShapeSettings ---
	SphereShapeSettings_CreateShape :: proc(settings: ^SphereShapeSettings) -> ^SphereShape ---
	SphereShapeSettings_GetRadius :: proc(settings: ^SphereShapeSettings) -> c.float ---
	SphereShapeSettings_SetRadius :: proc(settings: ^SphereShapeSettings, radius: c.float) ---
	SphereShape_Create :: proc(radius: c.float) -> ^SphereShape ---
	SphereShape_GetRadius :: proc(shape: ^SphereShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// PlaneShape
	//-------------------------------------------------------------------------------------------------
	PlaneShapeSettings_Create :: proc(plane: ^Plane, material: ^PhysicsMaterial, halfExtent: c.float) -> ^PlaneShapeSettings ---
	PlaneShapeSettings_CreateShape :: proc(settings: ^PlaneShapeSettings) -> ^PlaneShape ---
	PlaneShape_Create :: proc(plane: ^Plane, material: ^PhysicsMaterial, halfExtent: c.float) -> ^PlaneShape ---
	PlaneShape_GetPlane :: proc(shape: ^PlaneShape, result: ^Plane) ---
	PlaneShape_GetHalfExtent :: proc(shape: ^PlaneShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// TriangleShape
	//-------------------------------------------------------------------------------------------------
	TriangleShapeSettings_Create :: proc(v1: ^Vec3, v2: ^Vec3, v3: ^Vec3, convexRadius: c.float) -> ^TriangleShapeSettings ---
	TriangleShapeSettings_CreateShape :: proc(settings: ^TriangleShapeSettings) -> ^TriangleShape ---
	TriangleShape_Create :: proc(v1: ^Vec3, v2: ^Vec3, v3: ^Vec3, convexRadius: c.float) -> ^TriangleShape ---
	TriangleShape_GetConvexRadius :: proc(shape: ^TriangleShape) -> c.float ---
	TriangleShape_GetVertex1 :: proc(shape: ^TriangleShape, result: ^Vec3) ---
	TriangleShape_GetVertex2 :: proc(shape: ^TriangleShape, result: ^Vec3) ---
	TriangleShape_GetVertex3 :: proc(shape: ^TriangleShape, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// CapsuleShape
	//-------------------------------------------------------------------------------------------------
	CapsuleShapeSettings_Create :: proc(halfHeightOfCylinder: c.float, radius: c.float) -> ^CapsuleShapeSettings ---
	CapsuleShapeSettings_CreateShape :: proc(settings: ^CapsuleShapeSettings) -> ^CapsuleShape ---
	CapsuleShape_Create :: proc(halfHeightOfCylinder: c.float, radius: c.float) -> ^CapsuleShape ---
	CapsuleShape_GetRadius :: proc(shape: ^CapsuleShape) -> c.float ---
	CapsuleShape_GetHalfHeightOfCylinder :: proc(shape: ^CapsuleShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// CylinderShape
	//-------------------------------------------------------------------------------------------------
	CylinderShapeSettings_Create :: proc(halfHeight: c.float, radius: c.float, convexRadius: c.float) -> ^CylinderShapeSettings ---
	CylinderShapeSettings_CreateShape :: proc(settings: ^CylinderShapeSettings) -> ^CylinderShape ---
	CylinderShape_Create :: proc(halfHeight: c.float, radius: c.float) -> ^CylinderShape ---
	CylinderShape_GetRadius :: proc(shape: ^CylinderShape) -> c.float ---
	CylinderShape_GetHalfHeight :: proc(shape: ^CylinderShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// TaperedCylinderShape
	//-------------------------------------------------------------------------------------------------
	TaperedCylinderShapeSettings_Create :: proc(halfHeightOfTaperedCylinder: c.float, topRadius: c.float, bottomRadius: c.float, convexRadius: c.float, material: ^PhysicsMaterial) -> ^TaperedCylinderShapeSettings ---
	TaperedCylinderShapeSettings_CreateShape :: proc(settings: ^TaperedCylinderShapeSettings) -> ^TaperedCylinderShape ---
	TaperedCylinderShape_GetTopRadius :: proc(shape: ^TaperedCylinderShape) -> c.float ---
	TaperedCylinderShape_GetBottomRadius :: proc(shape: ^TaperedCylinderShape) -> c.float ---
	TaperedCylinderShape_GetConvexRadius :: proc(shape: ^TaperedCylinderShape) -> c.float ---
	TaperedCylinderShape_GetHalfHeight :: proc(shape: ^TaperedCylinderShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// ConvexHullShape
	//-------------------------------------------------------------------------------------------------
	ConvexHullShapeSettings_Create :: proc(points: ^Vec3, pointsCount: c.uint32_t, maxConvexRadius: c.float) -> ^ConvexHullShapeSettings ---
	ConvexHullShapeSettings_CreateShape :: proc(settings: ^ConvexHullShapeSettings) -> ^ConvexHullShape ---
	ConvexHullShape_GetNumPoints :: proc(shape: ^ConvexHullShape) -> c.uint32_t ---
	ConvexHullShape_GetPoint :: proc(shape: ^ConvexHullShape, index: c.uint32_t, result: ^Vec3) ---
	ConvexHullShape_GetNumFaces :: proc(shape: ^ConvexHullShape) -> c.uint32_t ---
	ConvexHullShape_GetNumVerticesInFace :: proc(shape: ^ConvexHullShape, faceIndex: c.uint32_t) -> c.uint32_t ---
	ConvexHullShape_GetFaceVertices :: proc(shape: ^ConvexHullShape, faceIndex: c.uint32_t, maxVertices: c.uint32_t, vertices: ^c.uint32_t) -> c.uint32_t ---

	//-------------------------------------------------------------------------------------------------
	// MeshShape
	//-------------------------------------------------------------------------------------------------
	MeshShapeSettings_Create :: proc(triangles: ^Triangle, triangleCount: c.uint32_t) -> ^MeshShapeSettings ---
	MeshShapeSettings_Create2 :: proc(vertices: ^Vec3, verticesCount: c.uint32_t, triangles: ^IndexedTriangle, triangleCount: c.uint32_t) -> ^MeshShapeSettings ---
	MeshShapeSettings_GetPerTriangleUserData :: proc(settings: ^MeshShapeSettings) -> c.bool ---
	MeshShapeSettings_SetPerTriangleUserData :: proc(settings: ^MeshShapeSettings, perTriangleUserData: c.bool) ---
	MeshShapeSettings_Sanitize :: proc(settings: ^MeshShapeSettings) ---
	MeshShapeSettings_CreateShape :: proc(settings: ^MeshShapeSettings) -> ^MeshShape ---
	MeshShape_GetTriangleUserData :: proc(shape: ^MeshShape, id: SubShapeID) -> c.uint32_t ---

	//-------------------------------------------------------------------------------------------------
	// HeightFieldShape
	//-------------------------------------------------------------------------------------------------
	HeightFieldShapeSettings_Create :: proc(samples: ^c.float, offset: ^Vec3, scale: ^Vec3, sampleCount: c.uint32_t) -> ^HeightFieldShapeSettings ---
	HeightFieldShapeSettings_CreateShape :: proc(settings: ^HeightFieldShapeSettings) -> ^HeightFieldShape ---
	HeightFieldShapeSettings_DetermineMinAndMaxSample :: proc(settings: ^HeightFieldShapeSettings, pOutMinValue: ^c.float, pOutMaxValue: ^c.float, pOutQuantizationScale: ^c.float) ---
	HeightFieldShapeSettings_CalculateBitsPerSampleForError :: proc(settings: ^HeightFieldShapeSettings, maxError: c.float) -> c.uint32_t ---
	HeightFieldShape_GetSampleCount :: proc(shape: ^HeightFieldShape) -> c.uint32_t ---
	HeightFieldShape_GetBlockSize :: proc(shape: ^HeightFieldShape) -> c.uint32_t ---
	HeightFieldShape_GetMaterial :: proc(shape: ^HeightFieldShape, x: c.uint32_t, y: c.uint32_t) -> ^PhysicsMaterial ---
	HeightFieldShape_GetPosition :: proc(shape: ^HeightFieldShape, x: c.uint32_t, y: c.uint32_t, result: ^Vec3) ---
	HeightFieldShape_IsNoCollision :: proc(shape: ^HeightFieldShape, x: c.uint32_t, y: c.uint32_t) -> c.bool ---
	HeightFieldShape_ProjectOntoSurface :: proc(shape: ^HeightFieldShape, localPosition: ^Vec3, outSurfacePosition: ^Vec3, outSubShapeID: ^SubShapeID) -> c.bool ---
	HeightFieldShape_GetMinHeightValue :: proc(shape: ^HeightFieldShape) -> c.float ---
	HeightFieldShape_GetMaxHeightValue :: proc(shape: ^HeightFieldShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// TaperedCapsuleShape
	//-------------------------------------------------------------------------------------------------
	TaperedCapsuleShapeSettings_Create :: proc(halfHeightOfTaperedCylinder: c.float, topRadius: c.float, bottomRadius: c.float) -> ^TaperedCapsuleShapeSettings ---
	TaperedCapsuleShapeSettings_CreateShape :: proc(settings: ^TaperedCapsuleShapeSettings) -> ^TaperedCapsuleShape ---
	TaperedCapsuleShape_GetTopRadius :: proc(shape: ^TaperedCapsuleShape) -> c.float ---
	TaperedCapsuleShape_GetBottomRadius :: proc(shape: ^TaperedCapsuleShape) -> c.float ---
	TaperedCapsuleShape_GetHalfHeight :: proc(shape: ^TaperedCapsuleShape) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// CompoundShape
	//-------------------------------------------------------------------------------------------------
	CompoundShapeSettings_AddShape :: proc(settings: ^CompoundShapeSettings, position: ^Vec3, rotation: ^Quat, shape: ^ShapeSettings, userData: c.uint32_t) ---
	CompoundShapeSettings_AddShape2 :: proc(settings: ^CompoundShapeSettings, position: ^Vec3, rotation: ^Quat, shape: ^Shape, userData: c.uint32_t) ---
	CompoundShape_GetNumSubShapes :: proc(shape: ^CompoundShape) -> c.uint32_t ---
	CompoundShape_GetSubShape :: proc(shape: ^CompoundShape, index: c.uint32_t, subShape: ^^Shape, positionCOM: ^Vec3, rotation: ^Quat, userData: ^c.uint32_t) ---
	CompoundShape_GetSubShapeIndexFromID :: proc(shape: ^CompoundShape, id: SubShapeID, remainder: ^SubShapeID) -> c.uint32_t ---

	//-------------------------------------------------------------------------------------------------
	// StaticCompoundShape
	//-------------------------------------------------------------------------------------------------
	StaticCompoundShapeSettings_Create :: proc() -> ^StaticCompoundShapeSettings ---
	StaticCompoundShape_Create :: proc(settings: ^StaticCompoundShapeSettings) -> ^StaticCompoundShape ---

	//-------------------------------------------------------------------------------------------------
	// MutableCompoundShape
	//-------------------------------------------------------------------------------------------------
	MutableCompoundShapeSettings_Create :: proc() -> ^MutableCompoundShapeSettings ---
	MutableCompoundShape_Create :: proc(settings: ^MutableCompoundShapeSettings) -> ^MutableCompoundShape ---
	MutableCompoundShape_AddShape :: proc(shape: ^MutableCompoundShape, position: ^Vec3, rotation: ^Quat, child: ^Shape, userData: c.uint32_t, index: c.uint32_t) -> c.uint32_t ---
	MutableCompoundShape_RemoveShape :: proc(shape: ^MutableCompoundShape, index: c.uint32_t) ---
	MutableCompoundShape_ModifyShape :: proc(shape: ^MutableCompoundShape, index: c.uint32_t, position: ^Vec3, rotation: ^Quat) ---
	MutableCompoundShape_ModifyShape2 :: proc(shape: ^MutableCompoundShape, index: c.uint32_t, position: ^Vec3, rotation: ^Quat, newShape: ^Shape) ---
	MutableCompoundShape_AdjustCenterOfMass :: proc(shape: ^MutableCompoundShape) ---

	//-------------------------------------------------------------------------------------------------
	// DecoratedShape
	//-------------------------------------------------------------------------------------------------
	DecoratedShape_GetInnerShape :: proc(shape: ^DecoratedShape) -> ^Shape ---

	//-------------------------------------------------------------------------------------------------
	// RotatedTranslatedShape
	//-------------------------------------------------------------------------------------------------
	RotatedTranslatedShapeSettings_Create :: proc(position: ^Vec3, rotation: ^Quat, shapeSettings: ^ShapeSettings) -> ^RotatedTranslatedShapeSettings ---
	RotatedTranslatedShapeSettings_Create2 :: proc(position: ^Vec3, rotation: ^Quat, shape: ^Shape) -> ^RotatedTranslatedShapeSettings ---
	RotatedTranslatedShapeSettings_CreateShape :: proc(settings: ^RotatedTranslatedShapeSettings) -> ^RotatedTranslatedShape ---
	RotatedTranslatedShape_Create :: proc(position: ^Vec3, rotation: ^Quat, shape: ^Shape) -> ^RotatedTranslatedShape ---
	RotatedTranslatedShape_GetPosition :: proc(shape: ^RotatedTranslatedShape, position: ^Vec3) ---
	RotatedTranslatedShape_GetRotation :: proc(shape: ^RotatedTranslatedShape, rotation: ^Quat) ---

	//-------------------------------------------------------------------------------------------------
	// ScaledShape
	//-------------------------------------------------------------------------------------------------
	ScaledShapeSettings_Create :: proc(shapeSettings: ^ShapeSettings, scale: ^Vec3) -> ^ScaledShapeSettings ---
	ScaledShapeSettings_Create2 :: proc(shape: ^Shape, scale: ^Vec3) -> ^ScaledShapeSettings ---
	ScaledShapeSettings_CreateShape :: proc(settings: ^ScaledShapeSettings) -> ^ScaledShape ---
	ScaledShape_Create :: proc(shape: ^Shape, scale: ^Vec3) -> ^ScaledShape ---
	ScaledShape_GetScale :: proc(shape: ^ScaledShape, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// OffsetCenterOfMassShape
	//-------------------------------------------------------------------------------------------------
	OffsetCenterOfMassShapeSettings_Create :: proc(offset: ^Vec3, shapeSettings: ^ShapeSettings) -> ^OffsetCenterOfMassShapeSettings ---
	OffsetCenterOfMassShapeSettings_Create2 :: proc(offset: ^Vec3, shape: ^Shape) -> ^OffsetCenterOfMassShapeSettings ---
	OffsetCenterOfMassShapeSettings_CreateShape :: proc(settings: ^OffsetCenterOfMassShapeSettings) -> ^OffsetCenterOfMassShape ---
	OffsetCenterOfMassShape_Create :: proc(offset: ^Vec3, shape: ^Shape) -> ^OffsetCenterOfMassShape ---
	OffsetCenterOfMassShape_GetOffset :: proc(shape: ^OffsetCenterOfMassShape, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// EmptyShape
	//-------------------------------------------------------------------------------------------------
	EmptyShapeSettings_Create :: proc(centerOfMass: ^Vec3) -> ^EmptyShapeSettings ---
	EmptyShapeSettings_CreateShape :: proc(settings: ^EmptyShapeSettings) -> ^EmptyShape ---

	//-------------------------------------------------------------------------------------------------
	// BodyCreationSettings
	//-------------------------------------------------------------------------------------------------
	BodyCreationSettings_Create :: proc() -> ^BodyCreationSettings ---
	BodyCreationSettings_Create2 :: proc(settings: ^ShapeSettings, position: ^RVec3, rotation: ^Quat, motionType: MotionType, objectLayer: ObjectLayer) -> ^BodyCreationSettings ---
	BodyCreationSettings_Create3 :: proc(shape: ^Shape, position: ^RVec3, rotation: ^Quat, motionType: MotionType, objectLayer: ObjectLayer) -> ^BodyCreationSettings ---
	BodyCreationSettings_Destroy :: proc(settings: ^BodyCreationSettings) ---

	//-------------------------------------------------------------------------------------------------
	// BodyCreationSettings
	//-------------------------------------------------------------------------------------------------
	BodyCreationSettings_GetPosition :: proc(settings: ^BodyCreationSettings, result: ^RVec3) ---
	BodyCreationSettings_SetPosition :: proc(settings: ^BodyCreationSettings, value: ^RVec3) ---
	BodyCreationSettings_GetRotation :: proc(settings: ^BodyCreationSettings, result: ^Quat) ---
	BodyCreationSettings_SetRotation :: proc(settings: ^BodyCreationSettings, value: ^Quat) ---
	BodyCreationSettings_GetLinearVelocity :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_SetLinearVelocity :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_GetAngularVelocity :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_SetAngularVelocity :: proc(settings: ^BodyCreationSettings, velocity: ^Vec3) ---
	BodyCreationSettings_GetUserData :: proc(settings: ^BodyCreationSettings) -> u64 ---
	BodyCreationSettings_SetUserData :: proc(settings: ^BodyCreationSettings, value: u64) ---
	BodyCreationSettings_GetObjectLayer :: proc(settings: ^BodyCreationSettings) -> ObjectLayer ---
	BodyCreationSettings_SetObjectLayer :: proc(settings: ^BodyCreationSettings, value: ObjectLayer) ---
	BodyCreationSettings_GetMotionType :: proc(settings: ^BodyCreationSettings) -> MotionType ---
	BodyCreationSettings_SetMotionType :: proc(settings: ^BodyCreationSettings, value: MotionType) ---
	BodyCreationSettings_GetAllowedDOFs :: proc(settings: ^BodyCreationSettings) -> AllowedDOFs ---
	BodyCreationSettings_SetAllowedDOFs :: proc(settings: ^BodyCreationSettings, value: AllowedDOFs) ---
	BodyCreationSettings_GetAllowDynamicOrKinematic :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetAllowDynamicOrKinematic :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetIsSensor :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetIsSensor :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetCollideKinematicVsNonDynamic :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetCollideKinematicVsNonDynamic :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetUseManifoldReduction :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetUseManifoldReduction :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetApplyGyroscopicForce :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetApplyGyroscopicForce :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetMotionQuality :: proc(settings: ^BodyCreationSettings) -> MotionQuality ---
	BodyCreationSettings_SetMotionQuality :: proc(settings: ^BodyCreationSettings, value: MotionQuality) ---
	BodyCreationSettings_GetEnhancedInternalEdgeRemoval :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetEnhancedInternalEdgeRemoval :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetAllowSleeping :: proc(settings: ^BodyCreationSettings) -> bool ---
	BodyCreationSettings_SetAllowSleeping :: proc(settings: ^BodyCreationSettings, value: bool) ---
	BodyCreationSettings_GetFriction :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetFriction :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetRestitution :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetRestitution :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetLinearDamping :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetLinearDamping :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetAngularDamping :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetAngularDamping :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetMaxLinearVelocity :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetMaxLinearVelocity :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetMaxAngularVelocity :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetMaxAngularVelocity :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetGravityFactor :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetGravityFactor :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetNumVelocityStepsOverride :: proc(settings: ^BodyCreationSettings) -> u32 ---
	BodyCreationSettings_SetNumVelocityStepsOverride :: proc(settings: ^BodyCreationSettings, value: u32) ---
	BodyCreationSettings_GetNumPositionStepsOverride :: proc(settings: ^BodyCreationSettings) -> u32 ---
	BodyCreationSettings_SetNumPositionStepsOverride :: proc(settings: ^BodyCreationSettings, value: u32) ---
	BodyCreationSettings_GetOverrideMassProperties :: proc(settings: ^BodyCreationSettings) -> OverrideMassProperties ---
	BodyCreationSettings_SetOverrideMassProperties :: proc(settings: ^BodyCreationSettings, value: OverrideMassProperties) ---
	BodyCreationSettings_GetInertiaMultiplier :: proc(settings: ^BodyCreationSettings) -> f32 ---
	BodyCreationSettings_SetInertiaMultiplier :: proc(settings: ^BodyCreationSettings, value: f32) ---
	BodyCreationSettings_GetMassPropertiesOverride :: proc(settings: ^BodyCreationSettings, result: ^MassProperties) ---
	BodyCreationSettings_SetMassPropertiesOverride :: proc(settings: ^BodyCreationSettings, massProperties: ^MassProperties) ---

	//-------------------------------------------------------------------------------------------------
	// SoftBodyCreationSettings
	//-------------------------------------------------------------------------------------------------
	SoftBodyCreationSettings_Create :: proc() -> ^SoftBodyCreationSettings ---
	SoftBodyCreationSettings_Destroy :: proc(settings: ^SoftBodyCreationSettings) ---

	//-------------------------------------------------------------------------------------------------
	// Constraint
	//-------------------------------------------------------------------------------------------------
	Constraint_Destroy :: proc(constraint: ^Constraint) ---
	Constraint_GetType :: proc(constraint: ^Constraint) -> ConstraintType ---
	Constraint_GetSubType :: proc(constraint: ^Constraint) -> ConstraintSubType ---
	Constraint_GetConstraintPriority :: proc(constraint: ^Constraint) -> c.uint32_t ---
	Constraint_SetConstraintPriority :: proc(constraint: ^Constraint, priority: c.uint32_t) ---
	Constraint_GetNumVelocityStepsOverride :: proc(constraint: ^Constraint) -> c.uint32_t ---
	Constraint_SetNumVelocityStepsOverride :: proc(constraint: ^Constraint, value: c.uint32_t) ---
	Constraint_GetNumPositionStepsOverride :: proc(constraint: ^Constraint) -> c.uint32_t ---
	Constraint_SetNumPositionStepsOverride :: proc(constraint: ^Constraint, value: c.uint32_t) ---
	Constraint_GetEnabled :: proc(constraint: ^Constraint) -> c.bool ---
	Constraint_SetEnabled :: proc(constraint: ^Constraint, enabled: c.bool) ---
	Constraint_GetUserData :: proc(constraint: ^Constraint) -> c.uint64_t ---
	Constraint_SetUserData :: proc(constraint: ^Constraint, userData: c.uint64_t) ---
	Constraint_NotifyShapeChanged :: proc(constraint: ^Constraint, bodyID: BodyID, deltaCOM: ^Vec3) ---
	Constraint_ResetWarmStart :: proc(constraint: ^Constraint) ---
	Constraint_IsActive :: proc(constraint: ^Constraint) -> c.bool ---
	Constraint_SetupVelocityConstraint :: proc(constraint: ^Constraint, deltaTime: c.float) ---
	Constraint_WarmStartVelocityConstraint :: proc(constraint: ^Constraint, warmStartImpulseRatio: c.float) ---
	Constraint_SolveVelocityConstraint :: proc(constraint: ^Constraint, deltaTime: c.float) -> c.bool ---
	Constraint_SolvePositionConstraint :: proc(constraint: ^Constraint, deltaTime: c.float, baumgarte: c.float) -> c.bool ---

	//-------------------------------------------------------------------------------------------------
	// TwoBodyConstraint
	//-------------------------------------------------------------------------------------------------
	TwoBodyConstraint_GetBody1 :: proc(constraint: ^TwoBodyConstraint) -> ^Body ---
	TwoBodyConstraint_GetBody2 :: proc(constraint: ^TwoBodyConstraint) -> ^Body ---
	TwoBodyConstraint_GetConstraintToBody1Matrix :: proc(constraint: ^TwoBodyConstraint, result: ^Matrix4x4) ---
	TwoBodyConstraint_GetConstraintToBody2Matrix :: proc(constraint: ^TwoBodyConstraint, result: ^Matrix4x4) ---

	//-------------------------------------------------------------------------------------------------
	// FixedConstraint
	//-------------------------------------------------------------------------------------------------
	FixedConstraintSettings_Init :: proc(settings: ^FixedConstraintSettings) ---
	FixedConstraint_Create :: proc(settings: ^FixedConstraintSettings, body1: ^Body, body2: ^Body) -> ^FixedConstraint ---
	FixedConstraint_GetSettings :: proc(constraint: ^FixedConstraint, settings: ^FixedConstraintSettings) ---
	FixedConstraint_GetTotalLambdaPosition :: proc(constraint: ^FixedConstraint, result: ^Vec3) ---
	FixedConstraint_GetTotalLambdaRotation :: proc(constraint: ^FixedConstraint, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// DistanceConstraint
	//-------------------------------------------------------------------------------------------------
	DistanceConstraintSettings_Init :: proc(settings: ^DistanceConstraintSettings) ---
	DistanceConstraint_Create :: proc(settings: ^DistanceConstraintSettings, body1: ^Body, body2: ^Body) -> ^DistanceConstraint ---
	DistanceConstraint_GetSettings :: proc(constraint: ^DistanceConstraint, settings: ^DistanceConstraintSettings) ---
	DistanceConstraint_SetDistance :: proc(constraint: ^DistanceConstraint, minDistance: c.float, maxDistance: c.float) ---
	DistanceConstraint_GetMinDistance :: proc(constraint: ^DistanceConstraint) -> c.float ---
	DistanceConstraint_GetMaxDistance :: proc(constraint: ^DistanceConstraint) -> c.float ---
	DistanceConstraint_GetLimitsSpringSettings :: proc(constraint: ^DistanceConstraint, result: ^SpringSettings) ---
	DistanceConstraint_SetLimitsSpringSettings :: proc(constraint: ^DistanceConstraint, settings: ^SpringSettings) ---
	DistanceConstraint_GetTotalLambdaPosition :: proc(constraint: ^DistanceConstraint) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// PointConstraint
	//-------------------------------------------------------------------------------------------------
	PointConstraintSettings_Init :: proc(settings: ^PointConstraintSettings) ---
	PointConstraint_Create :: proc(settings: ^PointConstraintSettings, body1: ^Body, body2: ^Body) -> ^PointConstraint ---
	PointConstraint_GetSettings :: proc(constraint: ^PointConstraint, settings: ^PointConstraintSettings) ---
	PointConstraint_SetPoint1 :: proc(constraint: ^PointConstraint, space: ConstraintSpace, value: ^RVec3) ---
	PointConstraint_SetPoint2 :: proc(constraint: ^PointConstraint, space: ConstraintSpace, value: ^RVec3) ---
	PointConstraint_GetLocalSpacePoint1 :: proc(constraint: ^PointConstraint, result: ^Vec3) ---
	PointConstraint_GetLocalSpacePoint2 :: proc(constraint: ^PointConstraint, result: ^Vec3) ---
	PointConstraint_GetTotalLambdaPosition :: proc(constraint: ^PointConstraint, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// HingeConstraint
	//-------------------------------------------------------------------------------------------------
	HingeConstraintSettings_Init :: proc(settings: ^HingeConstraintSettings) ---
	HingeConstraint_Create :: proc(settings: ^HingeConstraintSettings, body1: ^Body, body2: ^Body) -> ^HingeConstraint ---
	HingeConstraint_GetSettings :: proc(constraint: ^HingeConstraint, settings: ^HingeConstraintSettings) ---
	HingeConstraint_GetLocalSpacePoint1 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpacePoint2 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceHingeAxis1 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceHingeAxis2 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceNormalAxis1 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetLocalSpaceNormalAxis2 :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetCurrentAngle :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_SetMaxFrictionTorque :: proc(constraint: ^HingeConstraint, frictionTorque: c.float) ---
	HingeConstraint_GetMaxFrictionTorque :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_SetMotorSettings :: proc(constraint: ^HingeConstraint, settings: ^MotorSettings) ---
	HingeConstraint_GetMotorSettings :: proc(constraint: ^HingeConstraint, result: ^MotorSettings) ---
	HingeConstraint_SetMotorState :: proc(constraint: ^HingeConstraint, state: MotorState) ---
	HingeConstraint_GetMotorState :: proc(constraint: ^HingeConstraint) -> MotorState ---
	HingeConstraint_SetTargetAngularVelocity :: proc(constraint: ^HingeConstraint, angularVelocity: c.float) ---
	HingeConstraint_GetTargetAngularVelocity :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_SetTargetAngle :: proc(constraint: ^HingeConstraint, angle: c.float) ---
	HingeConstraint_GetTargetAngle :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_SetLimits :: proc(constraint: ^HingeConstraint, inLimitsMin: c.float, inLimitsMax: c.float) ---
	HingeConstraint_GetLimitsMin :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_GetLimitsMax :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_HasLimits :: proc(constraint: ^HingeConstraint) -> c.bool ---
	HingeConstraint_GetLimitsSpringSettings :: proc(constraint: ^HingeConstraint, result: ^SpringSettings) ---
	HingeConstraint_SetLimitsSpringSettings :: proc(constraint: ^HingeConstraint, settings: ^SpringSettings) ---
	HingeConstraint_GetTotalLambdaPosition :: proc(constraint: ^HingeConstraint, result: ^Vec3) ---
	HingeConstraint_GetTotalLambdaRotation :: proc(constraint: ^HingeConstraint, rotation: [2]c.float) ---
	HingeConstraint_GetTotalLambdaRotationLimits :: proc(constraint: ^HingeConstraint) -> c.float ---
	HingeConstraint_GetTotalLambdaMotor :: proc(constraint: ^HingeConstraint) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// SliderConstraint
	//-------------------------------------------------------------------------------------------------
	SliderConstraintSettings_Init :: proc(settings: ^SliderConstraintSettings) ---
	SliderConstraintSettings_SetSliderAxis :: proc(settings: ^SliderConstraintSettings, axis: ^Vec3) ---
	SliderConstraint_Create :: proc(settings: ^SliderConstraintSettings, body1: ^Body, body2: ^Body) -> ^SliderConstraint ---
	SliderConstraint_GetSettings :: proc(constraint: ^SliderConstraint, settings: ^SliderConstraintSettings) ---
	SliderConstraint_GetCurrentPosition :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_SetMaxFrictionForce :: proc(constraint: ^SliderConstraint, frictionForce: c.float) ---
	SliderConstraint_GetMaxFrictionForce :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_SetMotorSettings :: proc(constraint: ^SliderConstraint, settings: ^MotorSettings) ---
	SliderConstraint_GetMotorSettings :: proc(constraint: ^SliderConstraint, result: ^MotorSettings) ---
	SliderConstraint_SetMotorState :: proc(constraint: ^SliderConstraint, state: MotorState) ---
	SliderConstraint_GetMotorState :: proc(constraint: ^SliderConstraint) -> MotorState ---
	SliderConstraint_SetTargetVelocity :: proc(constraint: ^SliderConstraint, velocity: c.float) ---
	SliderConstraint_GetTargetVelocity :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_SetTargetPosition :: proc(constraint: ^SliderConstraint, position: c.float) ---
	SliderConstraint_GetTargetPosition :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_SetLimits :: proc(constraint: ^SliderConstraint, inLimitsMin: c.float, inLimitsMax: c.float) ---
	SliderConstraint_GetLimitsMin :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_GetLimitsMax :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_HasLimits :: proc(constraint: ^SliderConstraint) -> c.bool ---
	SliderConstraint_GetLimitsSpringSettings :: proc(constraint: ^SliderConstraint, result: ^SpringSettings) ---
	SliderConstraint_SetLimitsSpringSettings :: proc(constraint: ^SliderConstraint, settings: ^SpringSettings) ---
	SliderConstraint_GetTotalLambdaPosition :: proc(constraint: ^SliderConstraint, position: [2]c.float) ---
	SliderConstraint_GetTotalLambdaPositionLimits :: proc(constraint: ^SliderConstraint) -> c.float ---
	SliderConstraint_GetTotalLambdaRotation :: proc(constraint: ^SliderConstraint, result: ^Vec3) ---
	SliderConstraint_GetTotalLambdaMotor :: proc(constraint: ^SliderConstraint) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// ConeConstraint
	//-------------------------------------------------------------------------------------------------
	ConeConstraintSettings_Init :: proc(settings: ^ConeConstraintSettings) ---
	ConeConstraint_Create :: proc(settings: ^ConeConstraintSettings, body1: ^Body, body2: ^Body) -> ^ConeConstraint ---
	ConeConstraint_GetSettings :: proc(constraint: ^ConeConstraint, settings: ^ConeConstraintSettings) ---
	ConeConstraint_SetHalfConeAngle :: proc(constraint: ^ConeConstraint, halfConeAngle: c.float) ---
	ConeConstraint_GetCosHalfConeAngle :: proc(constraint: ^ConeConstraint) -> c.float ---
	ConeConstraint_GetTotalLambdaPosition :: proc(constraint: ^ConeConstraint, result: ^Vec3) ---
	ConeConstraint_GetTotalLambdaRotation :: proc(constraint: ^ConeConstraint) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// SwingTwistConstraint
	//-------------------------------------------------------------------------------------------------
	SwingTwistConstraintSettings_Init :: proc(settings: ^SwingTwistConstraintSettings) ---
	SwingTwistConstraint_Create :: proc(settings: ^SwingTwistConstraintSettings, body1: ^Body, body2: ^Body) -> ^SwingTwistConstraint ---
	SwingTwistConstraint_GetSettings :: proc(constraint: ^SwingTwistConstraint, settings: ^SwingTwistConstraintSettings) ---
	SwingTwistConstraint_GetNormalHalfConeAngle :: proc(constraint: ^SwingTwistConstraint) -> c.float ---
	SwingTwistConstraint_GetTotalLambdaPosition :: proc(constraint: ^SwingTwistConstraint, result: ^Vec3) ---
	SwingTwistConstraint_GetTotalLambdaTwist :: proc(constraint: ^SwingTwistConstraint) -> c.float ---
	SwingTwistConstraint_GetTotalLambdaSwingY :: proc(constraint: ^SwingTwistConstraint) -> c.float ---
	SwingTwistConstraint_GetTotalLambdaSwingZ :: proc(constraint: ^SwingTwistConstraint) -> c.float ---
	SwingTwistConstraint_GetTotalLambdaMotor :: proc(constraint: ^SwingTwistConstraint, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// SixDOFConstraint
	//-------------------------------------------------------------------------------------------------
	SixDOFConstraintSettings_Init :: proc(settings: ^SixDOFConstraintSettings) ---
	SixDOFConstraintSettings_MakeFreeAxis :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) ---
	SixDOFConstraintSettings_IsFreeAxis :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) -> c.bool ---
	SixDOFConstraintSettings_MakeFixedAxis :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) ---
	SixDOFConstraintSettings_IsFixedAxis :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis) -> c.bool ---
	SixDOFConstraintSettings_SetLimitedAxis :: proc(settings: ^SixDOFConstraintSettings, axis: SixDOFConstraintAxis, min: c.float, max: c.float) ---
	SixDOFConstraint_Create :: proc(settings: ^SixDOFConstraintSettings, body1: ^Body, body2: ^Body) -> ^SixDOFConstraint ---
	SixDOFConstraint_GetSettings :: proc(constraint: ^SixDOFConstraint, settings: ^SixDOFConstraintSettings) ---
	SixDOFConstraint_GetLimitsMin :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> c.float ---
	SixDOFConstraint_GetLimitsMax :: proc(constraint: ^SixDOFConstraint, axis: SixDOFConstraintAxis) -> c.float ---
	SixDOFConstraint_GetTotalLambdaPosition :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTotalLambdaRotation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTotalLambdaMotorTranslation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---
	SixDOFConstraint_GetTotalLambdaMotorRotation :: proc(constraint: ^SixDOFConstraint, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// GearConstraint
	//-------------------------------------------------------------------------------------------------
	GearConstraintSettings_Init :: proc(settings: ^GearConstraintSettings) ---
	GearConstraint_Create :: proc(settings: ^GearConstraintSettings, body1: ^Body, body2: ^Body) -> ^GearConstraint ---
	GearConstraint_GetSettings :: proc(constraint: ^GearConstraint, settings: ^GearConstraintSettings) ---
	GearConstraint_SetConstraints :: proc(constraint: ^GearConstraint, gear1: ^Constraint, gear2: ^Constraint) ---
	GearConstraint_GetTotalLambda :: proc(constraint: ^GearConstraint) -> c.float ---

	//-------------------------------------------------------------------------------------------------
	// BodyInterface
	//-------------------------------------------------------------------------------------------------
	BodyInterface_DestroyBody :: proc(interface: ^BodyInterface, bodyID: BodyID) ---
	BodyInterface_CreateAndAddBody :: proc(interface: ^BodyInterface, settings: ^BodyCreationSettings, activationMode: Activation) -> BodyID ---
	BodyInterface_CreateBody :: proc(interface: ^BodyInterface, settings: ^BodyCreationSettings) -> ^Body ---
	BodyInterface_CreateBodyWithID :: proc(interface: ^BodyInterface, bodyID: BodyID, settings: ^BodyCreationSettings) -> ^Body ---
	BodyInterface_CreateBodyWithoutID :: proc(interface: ^BodyInterface, settings: ^BodyCreationSettings) -> ^Body ---
	BodyInterface_DestroyBodyWithoutID :: proc(interface: ^BodyInterface, body: ^Body) ---
	BodyInterface_AssignBodyID :: proc(interface: ^BodyInterface, body: ^Body) -> c.bool ---
	BodyInterface_AssignBodyID2 :: proc(interface: ^BodyInterface, body: ^Body, bodyID: BodyID) -> c.bool ---
	BodyInterface_UnassignBodyID :: proc(interface: ^BodyInterface, bodyID: BodyID) -> ^Body ---
	BodyInterface_CreateSoftBody :: proc(interface: ^BodyInterface, settings: ^SoftBodyCreationSettings) -> ^Body ---
	BodyInterface_CreateSoftBodyWithID :: proc(interface: ^BodyInterface, bodyID: BodyID, settings: ^SoftBodyCreationSettings) -> ^Body ---
	BodyInterface_CreateSoftBodyWithoutID :: proc(interface: ^BodyInterface, settings: ^SoftBodyCreationSettings) -> ^Body ---
	BodyInterface_CreateAndAddSoftBody :: proc(interface: ^BodyInterface, settings: ^SoftBodyCreationSettings, activationMode: Activation) -> BodyID ---
	BodyInterface_AddBody :: proc(interface: ^BodyInterface, bodyID: BodyID, activationMode: Activation) ---
	BodyInterface_RemoveBody :: proc(interface: ^BodyInterface, bodyID: BodyID) ---
	BodyInterface_RemoveAndDestroyBody :: proc(interface: ^BodyInterface, bodyID: BodyID) ---
	BodyInterface_IsActive :: proc(interface: ^BodyInterface, bodyID: BodyID) -> c.bool ---
	BodyInterface_IsAdded :: proc(interface: ^BodyInterface, bodyID: BodyID) -> c.bool ---
	BodyInterface_GetBodyType :: proc(interface: ^BodyInterface, bodyID: BodyID) -> BodyType ---
	BodyInterface_SetLinearVelocity :: proc(interface: ^BodyInterface, bodyID: BodyID, velocity: ^Vec3) ---
	BodyInterface_GetLinearVelocity :: proc(interface: ^BodyInterface, bodyID: BodyID, velocity: ^Vec3) ---
	BodyInterface_GetCenterOfMassPosition :: proc(interface: ^BodyInterface, bodyID: BodyID, position: ^RVec3) ---
	BodyInterface_GetMotionType :: proc(interface: ^BodyInterface, bodyID: BodyID) -> MotionType ---
	BodyInterface_SetMotionType :: proc(interface: ^BodyInterface, bodyID: BodyID, motionType: MotionType, activationMode: Activation) ---
	BodyInterface_GetRestitution :: proc(interface: ^BodyInterface, bodyID: BodyID) -> c.float ---
	BodyInterface_SetRestitution :: proc(interface: ^BodyInterface, bodyID: BodyID, restitution: c.float) ---
	BodyInterface_GetFriction :: proc(interface: ^BodyInterface, bodyID: BodyID) -> c.float ---
	BodyInterface_SetFriction :: proc(interface: ^BodyInterface, bodyID: BodyID, friction: c.float) ---
	BodyInterface_SetPosition :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, activationMode: Activation) ---
	BodyInterface_GetPosition :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RVec3) ---
	BodyInterface_SetRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, rotation: ^Quat, activationMode: Activation) ---
	BodyInterface_GetRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^Quat) ---
	BodyInterface_SetPositionAndRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, activationMode: Activation) ---
	BodyInterface_SetPositionAndRotationWhenChanged :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, activationMode: Activation) ---
	BodyInterface_GetPositionAndRotation :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat) ---
	BodyInterface_SetPositionRotationAndVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, position: ^RVec3, rotation: ^Quat, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_GetShape :: proc(interface: ^BodyInterface, bodyId: BodyID) -> ^Shape ---
	BodyInterface_SetShape :: proc(interface: ^BodyInterface, bodyId: BodyID, shape: ^Shape, updateMassProperties: c.bool, activationMode: Activation) ---
	BodyInterface_NotifyShapeChanged :: proc(interface: ^BodyInterface, bodyId: BodyID, previousCenterOfMass: ^Vec3, updateMassProperties: c.bool, activationMode: Activation) ---
	BodyInterface_ActivateBody :: proc(interface: ^BodyInterface, bodyId: BodyID) ---
	BodyInterface_DeactivateBody :: proc(interface: ^BodyInterface, bodyId: BodyID) ---
	BodyInterface_GetObjectLayer :: proc(interface: ^BodyInterface, bodyId: BodyID) -> ObjectLayer ---
	BodyInterface_SetObjectLayer :: proc(interface: ^BodyInterface, bodyId: BodyID, layer: ObjectLayer) ---
	BodyInterface_GetWorldTransform :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RMatrix4x4) ---
	BodyInterface_GetCenterOfMassTransform :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^RMatrix4x4) ---
	BodyInterface_MoveKinematic :: proc(interface: ^BodyInterface, bodyId: BodyID, targetPosition: ^RVec3, targetRotation: ^Quat, deltaTime: c.float) ---
	BodyInterface_ApplyBuoyancyImpulse :: proc(interface: ^BodyInterface, bodyId: BodyID, surfacePosition: ^RVec3, surfaceNormal: ^Vec3, buoyancy: c.float, linearDrag: c.float, angularDrag: c.float, fluidVelocity: ^Vec3, gravity: ^Vec3, deltaTime: c.float) -> c.bool ---
	BodyInterface_SetLinearAndAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_GetLinearAndAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_AddLinearVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3) ---
	BodyInterface_AddLinearAndAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, linearVelocity: ^Vec3, angularVelocity: ^Vec3) ---
	BodyInterface_SetAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, angularVelocity: ^Vec3) ---
	BodyInterface_GetAngularVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, angularVelocity: ^Vec3) ---
	BodyInterface_GetPointVelocity :: proc(interface: ^BodyInterface, bodyId: BodyID, point: ^RVec3, velocity: ^Vec3) ---
	BodyInterface_AddForce :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3) ---
	BodyInterface_AddForce2 :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3, point: ^RVec3) ---
	BodyInterface_AddTorque :: proc(interface: ^BodyInterface, bodyId: BodyID, torque: ^Vec3) ---
	BodyInterface_AddForceAndTorque :: proc(interface: ^BodyInterface, bodyId: BodyID, force: ^Vec3, torque: ^Vec3) ---
	BodyInterface_AddImpulse :: proc(interface: ^BodyInterface, bodyId: BodyID, impulse: ^Vec3) ---
	BodyInterface_AddImpulse2 :: proc(interface: ^BodyInterface, bodyId: BodyID, impulse: ^Vec3, point: ^RVec3) ---
	BodyInterface_AddAngularImpulse :: proc(interface: ^BodyInterface, bodyId: BodyID, angularImpulse: ^Vec3) ---
	BodyInterface_SetMotionQuality :: proc(interface: ^BodyInterface, bodyId: BodyID, quality: MotionQuality) ---
	BodyInterface_GetMotionQuality :: proc(interface: ^BodyInterface, bodyId: BodyID) -> MotionQuality ---
	BodyInterface_GetInverseInertia :: proc(interface: ^BodyInterface, bodyId: BodyID, result: ^Matrix4x4) ---
	BodyInterface_SetGravityFactor :: proc(interface: ^BodyInterface, bodyId: BodyID, value: c.float) ---
	BodyInterface_GetGravityFactor :: proc(interface: ^BodyInterface, bodyId: BodyID) -> c.float ---
	BodyInterface_SetUseManifoldReduction :: proc(interface: ^BodyInterface, bodyId: BodyID, value: c.bool) ---
	BodyInterface_GetUseManifoldReduction :: proc(interface: ^BodyInterface, bodyId: BodyID) -> c.bool ---
	BodyInterface_SetUserData :: proc(interface: ^BodyInterface, bodyId: BodyID, inUserData: c.uint64_t) ---
	BodyInterface_GetUserData :: proc(interface: ^BodyInterface, bodyId: BodyID) -> c.uint64_t ---
	BodyInterface_GetMaterial :: proc(interface: ^BodyInterface, bodyId: BodyID, subShapeID: SubShapeID) -> ^PhysicsMaterial ---
	BodyInterface_InvalidateContactCache :: proc(interface: ^BodyInterface, bodyId: BodyID) ---

	//-------------------------------------------------------------------------------------------------
	// BodyLockInterface
	//-------------------------------------------------------------------------------------------------
	BodyLockInterface_LockRead :: proc(lockInterface: ^BodyLockInterface, bodyID: BodyID, outLock: ^BodyLockRead) ---
	BodyLockInterface_UnlockRead :: proc(lockInterface: ^BodyLockInterface, ioLock: ^BodyLockRead) ---

	BodyLockInterface_LockWrite :: proc(lockInterface: ^BodyLockInterface, bodyID: BodyID, outLock: ^BodyLockWrite) ---
	BodyLockInterface_UnlockWrite :: proc(lockInterface: ^BodyLockInterface, ioLock: ^BodyLockWrite) ---

	BodyLockInterface_LockMultiRead :: proc(lockInterface: ^BodyLockInterface, bodyIDs: [^]BodyID, count: c.uint32_t) -> ^BodyLockMultiRead ---
	BodyLockMultiRead_Destroy :: proc(ioLock: ^BodyLockMultiRead) ---
	BodyLockMultiRead_GetBody :: proc(ioLock: ^BodyLockMultiRead, bodyIndex: c.uint32_t) -> ^Body ---

	BodyLockInterface_LockMultiWrite :: proc(lockInterface: ^BodyLockInterface, bodyIDs: [^]BodyID, count: c.uint32_t) -> ^BodyLockMultiWrite ---
	BodyLockMultiWrite_Destroy :: proc(ioLock: ^BodyLockMultiWrite) ---
	BodyLockMultiWrite_GetBody :: proc(ioLock: ^BodyLockMultiWrite, bodyIndex: c.uint32_t) -> ^Body ---

	//-------------------------------------------------------------------------------------------------
	// MotionProperties
	//-------------------------------------------------------------------------------------------------
	MotionProperties_GetAllowedDOFs :: proc(properties: ^MotionProperties) -> AllowedDOFs ---
	MotionProperties_SetLinearDamping :: proc(properties: ^MotionProperties, damping: c.float) ---
	MotionProperties_GetLinearDamping :: proc(properties: ^MotionProperties) -> c.float ---
	MotionProperties_SetAngularDamping :: proc(properties: ^MotionProperties, damping: c.float) ---
	MotionProperties_GetAngularDamping :: proc(properties: ^MotionProperties) -> c.float ---
	MotionProperties_SetMassProperties :: proc(properties: ^MotionProperties, allowedDOFs: AllowedDOFs, massProperties: ^MassProperties) ---
	MotionProperties_GetInverseMassUnchecked :: proc(properties: ^MotionProperties) -> c.float ---
	MotionProperties_SetInverseMass :: proc(properties: ^MotionProperties, inverseMass: c.float) ---
	MotionProperties_GetInverseInertiaDiagonal :: proc(properties: ^MotionProperties, result: ^Vec3) ---
	MotionProperties_GetInertiaRotation :: proc(properties: ^MotionProperties, result: ^Quat) ---
	MotionProperties_SetInverseInertia :: proc(properties: ^MotionProperties, diagonal: ^Vec3, rot: ^Quat) ---
	MotionProperties_ScaleToMass :: proc(properties: ^MotionProperties, mass: c.float) ---

	//-------------------------------------------------------------------------------------------------
	// MassProperties
	//-------------------------------------------------------------------------------------------------
	MassProperties_DecomposePrincipalMomentsOfInertia :: proc(properties: ^MassProperties, rotation: ^Matrix4x4, diagonal: ^Vec3) ---
	MassProperties_ScaleToMass :: proc(properties: ^MassProperties, mass: c.float) ---
	MassProperties_GetEquivalentSolidBoxSize :: proc(mass: c.float, inertiaDiagonal: ^Vec3, result: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// CollideShapeSettings
	//-------------------------------------------------------------------------------------------------
	CollideShapeSettings_Init :: proc(settings: ^CollideShapeSettings) ---

	//-------------------------------------------------------------------------------------------------
	// ShapeCastSettings
	//-------------------------------------------------------------------------------------------------
	ShapeCastSettings_Init :: proc(settings: ^ShapeCastSettings) ---

	//-------------------------------------------------------------------------------------------------
	// BroadPhaseQuery
	//-------------------------------------------------------------------------------------------------
	BroadPhaseQuery_CastRay :: proc(query: ^BroadPhaseQuery, origin: ^Vec3, direction: ^Vec3, callback: RayCastBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> c.bool ---
	BroadPhaseQuery_CastRay2 :: proc(query: ^BroadPhaseQuery, origin: ^Vec3, direction: ^Vec3, collectorType: CollisionCollectorType, callback: RayCastBodyResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> c.bool ---
	BroadPhaseQuery_CollideAABox :: proc(query: ^BroadPhaseQuery, box: ^AABox, callback: CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> c.bool ---
	BroadPhaseQuery_CollideSphere :: proc(query: ^BroadPhaseQuery, center: ^Vec3, radius: c.float, callback: CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> c.bool ---
	BroadPhaseQuery_CollidePoint :: proc(query: ^BroadPhaseQuery, point: ^Vec3, callback: CollideShapeBodyCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter) -> c.bool ---

	//-------------------------------------------------------------------------------------------------
	// NarrowPhaseQuery
	//-------------------------------------------------------------------------------------------------
	NarrowPhaseQuery_CastRay :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, hit: ^RayCastResult, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter) -> c.bool ---
	NarrowPhaseQuery_CastRay2 :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, rayCastSettings: ^RayCastSettings, callback: CastRayCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CastRay3 :: proc(query: ^NarrowPhaseQuery, origin: ^RVec3, direction: ^Vec3, rayCastSettings: ^RayCastSettings, collectorType: CollisionCollectorType, callback: CastRayResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CollidePoint :: proc(query: ^NarrowPhaseQuery, point: ^RVec3, callback: CollidePointCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CollidePoint2 :: proc(query: ^NarrowPhaseQuery, point: ^RVec3, collectorType: CollisionCollectorType, callback: CollidePointResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CollideShape :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, scale: ^Vec3, centerOfMassTransform: ^RMatrix4x4, settings: ^CollideShapeSettings, baseOffset: ^RVec3, callback: CollideShapeCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CollideShape2 :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, scale: ^Vec3, centerOfMassTransform: ^RMatrix4x4, settings: ^CollideShapeSettings, baseOffset: ^RVec3, collectorType: CollisionCollectorType, callback: CollideShapeResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CastShape :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, worldTransform: ^RMatrix4x4, direction: ^Vec3, settings: ^ShapeCastSettings, baseOffset: ^RVec3, callback: CastShapeCollectorCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	NarrowPhaseQuery_CastShape2 :: proc(query: ^NarrowPhaseQuery, shape: ^Shape, worldTransform: ^RMatrix4x4, direction: ^Vec3, settings: ^ShapeCastSettings, baseOffset: ^RVec3, collectorType: CollisionCollectorType, callback: CastShapeResultCallback, userData: rawptr, broadPhaseLayerFilter: ^BroadPhaseLayerFilter, objectLayerFilter: ^ObjectLayerFilter, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---

	//-------------------------------------------------------------------------------------------------
	// Body
	//-------------------------------------------------------------------------------------------------
	Body_GetID :: proc(body: ^Body) -> BodyID ---
	Body_GetBodyType :: proc(body: ^Body) -> BodyType ---
	Body_IsRigidBody :: proc(body: ^Body) -> c.bool ---
	Body_IsSoftBody :: proc(body: ^Body) -> c.bool ---
	Body_IsActive :: proc(body: ^Body) -> c.bool ---
	Body_IsStatic :: proc(body: ^Body) -> c.bool ---
	Body_IsKinematic :: proc(body: ^Body) -> c.bool ---
	Body_IsDynamic :: proc(body: ^Body) -> c.bool ---
	Body_CanBeKinematicOrDynamic :: proc(body: ^Body) -> c.bool ---
	Body_SetIsSensor :: proc(body: ^Body, value: c.bool) ---
	Body_IsSensor :: proc(body: ^Body) -> c.bool ---
	Body_SetCollideKinematicVsNonDynamic :: proc(body: ^Body, value: c.bool) ---
	Body_GetCollideKinematicVsNonDynamic :: proc(body: ^Body) -> c.bool ---
	Body_SetUseManifoldReduction :: proc(body: ^Body, value: c.bool) ---
	Body_GetUseManifoldReduction :: proc(body: ^Body) -> c.bool ---
	Body_GetUseManifoldReductionWithBody :: proc(body: ^Body, other: ^Body) -> c.bool ---
	Body_SetApplyGyroscopicForce :: proc(body: ^Body, value: c.bool) ---
	Body_GetApplyGyroscopicForce :: proc(body: ^Body) -> c.bool ---
	Body_SetEnhancedInternalEdgeRemoval :: proc(body: ^Body, value: c.bool) ---
	Body_GetEnhancedInternalEdgeRemoval :: proc(body: ^Body) -> c.bool ---
	Body_GetEnhancedInternalEdgeRemovalWithBody :: proc(body: ^Body, other: ^Body) -> c.bool ---
	Body_GetMotionType :: proc(body: ^Body) -> MotionType ---
	Body_SetMotionType :: proc(body: ^Body, motionType: MotionType) ---
	Body_GetBroadPhaseLayer :: proc(body: ^Body) -> BroadPhaseLayer ---
	Body_GetObjectLayer :: proc(body: ^Body) -> ObjectLayer ---
	Body_GetAllowSleeping :: proc(body: ^Body) -> c.bool ---
	Body_SetAllowSleeping :: proc(body: ^Body, allowSleeping: c.bool) ---
	Body_ResetSleepTimer :: proc(body: ^Body) ---
	Body_GetFriction :: proc(body: ^Body) -> c.float ---
	Body_SetFriction :: proc(body: ^Body, friction: c.float) ---
	Body_GetRestitution :: proc(body: ^Body) -> c.float ---
	Body_SetRestitution :: proc(body: ^Body, restitution: c.float) ---
	Body_GetLinearVelocity :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetLinearVelocity :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetLinearVelocityClamped :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_GetAngularVelocity :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetAngularVelocity :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_SetAngularVelocityClamped :: proc(body: ^Body, velocity: ^Vec3) ---
	Body_GetPointVelocityCOM :: proc(body: ^Body, pointRelativeToCOM: ^Vec3, velocity: ^Vec3) ---
	Body_GetPointVelocity :: proc(body: ^Body, point: ^RVec3, velocity: ^Vec3) ---
	Body_AddForce :: proc(body: ^Body, force: ^Vec3) ---
	Body_AddForceAtPosition :: proc(body: ^Body, force: ^Vec3, position: ^RVec3) ---
	Body_AddTorque :: proc(body: ^Body, force: ^Vec3) ---
	Body_GetAccumulatedForce :: proc(body: ^Body, force: ^Vec3) ---
	Body_GetAccumulatedTorque :: proc(body: ^Body, force: ^Vec3) ---
	Body_ResetForce :: proc(body: ^Body) ---
	Body_ResetTorque :: proc(body: ^Body) ---
	Body_ResetMotion :: proc(body: ^Body) ---
	Body_GetInverseInertia :: proc(body: ^Body, result: ^Matrix4x4) ---
	Body_AddImpulse :: proc(body: ^Body, impulse: ^Vec3) ---
	Body_AddImpulseAtPosition :: proc(body: ^Body, impulse: ^Vec3, position: ^RVec3) ---
	Body_AddAngularImpulse :: proc(body: ^Body, angularImpulse: ^Vec3) ---
	Body_MoveKinematic :: proc(body: ^Body, targetPosition: ^RVec3, targetRotation: ^Quat, deltaTime: c.float) ---
	Body_ApplyBuoyancyImpulse :: proc(body: ^Body, surfacePosition: ^RVec3, surfaceNormal: ^Vec3, buoyancy: c.float, linearDrag: c.float, angularDrag: c.float, fluidVelocity: ^Vec3, gravity: ^Vec3, deltaTime: c.float) -> c.bool ---
	Body_IsInBroadPhase :: proc(body: ^Body) -> c.bool ---
	Body_IsCollisionCacheInvalid :: proc(body: ^Body) -> c.bool ---
	Body_GetShape :: proc(body: ^Body) -> ^Shape ---
	Body_GetPosition :: proc(body: ^Body, result: ^RVec3) ---
	Body_GetRotation :: proc(body: ^Body, result: ^Quat) ---
	Body_GetWorldTransform :: proc(body: ^Body, result: ^RMatrix4x4) ---
	Body_GetCenterOfMassPosition :: proc(body: ^Body, result: ^RVec3) ---
	Body_GetCenterOfMassTransform :: proc(body: ^Body, result: ^RMatrix4x4) ---
	Body_GetInverseCenterOfMassTransform :: proc(body: ^Body, result: ^RMatrix4x4) ---
	Body_GetWorldSpaceBounds :: proc(body: ^Body, result: ^AABox) ---
	Body_GetWorldSpaceSurfaceNormal :: proc(body: ^Body, subShapeID: SubShapeID, position: ^RVec3, normal: ^Vec3) ---
	Body_GetMotionProperties :: proc(body: ^Body) -> ^MotionProperties ---
	Body_GetMotionPropertiesUnchecked :: proc(body: ^Body) -> ^MotionProperties ---
	Body_SetUserData :: proc(body: ^Body, userData: c.uint64_t) ---
	Body_GetUserData :: proc(body: ^Body) -> c.uint64_t ---
	Body_GetFixedToWorldBody :: proc() -> ^Body ---

	//-------------------------------------------------------------------------------------------------
	// BroadPhaseLayerFilter
	//-------------------------------------------------------------------------------------------------
	BroadPhaseLayerFilter_Create :: proc(procs: BroadPhaseLayerFilter_Procs, userData: rawptr) -> ^BroadPhaseLayerFilter ---
	BroadPhaseLayerFilter_Destroy :: proc(filter: ^BroadPhaseLayerFilter) ---

	//-------------------------------------------------------------------------------------------------
	// ObjectLayerFilter
	//-------------------------------------------------------------------------------------------------
	ObjectLayerFilter_Create :: proc(procs: ObjectLayerFilter_Procs, userData: rawptr) -> ^ObjectLayerFilter ---
	ObjectLayerFilter_Destroy :: proc(filter: ^ObjectLayerFilter) ---

	//-------------------------------------------------------------------------------------------------
	// BodyFilter
	//-------------------------------------------------------------------------------------------------
	BodyFilter_Create :: proc(procs: BodyFilter_Procs, userData: rawptr) -> ^BodyFilter ---
	BodyFilter_Destroy :: proc(filter: ^BodyFilter) ---

	//-------------------------------------------------------------------------------------------------
	// ShapeFilter
	//-------------------------------------------------------------------------------------------------
	ShapeFilter_Create :: proc(procs: ShapeFilter_Procs, userData: rawptr) -> ^ShapeFilter ---
	ShapeFilter_Destroy :: proc(filter: ^ShapeFilter) ---
	ShapeFilter_GetBodyID2 :: proc(filter: ^ShapeFilter) -> BodyID ---
	ShapeFilter_SetBodyID2 :: proc(filter: ^ShapeFilter, id: BodyID) ---

	//-------------------------------------------------------------------------------------------------
	// ContactListener
	//-------------------------------------------------------------------------------------------------
	ContactListener_Create :: proc(procs: ContactListener_Procs, userData: rawptr) -> ^ContactListener ---
	ContactListener_Destroy :: proc(listener: ^ContactListener) ---

	//-------------------------------------------------------------------------------------------------
	// BodyActivationListener
	//-------------------------------------------------------------------------------------------------
	BodyActivationListener_Create :: proc(procs: BodyActivationListener_Procs, userData: rawptr) -> ^BodyActivationListener ---
	BodyActivationListener_Destroy :: proc(listener: ^BodyActivationListener) ---

	//-------------------------------------------------------------------------------------------------
	// BodyDrawFilter
	//-------------------------------------------------------------------------------------------------
	BodyDrawFilter_Create :: proc(procs: BodyDrawFilter_Procs, userData: rawptr) -> ^BodyDrawFilter ---
	BodyDrawFilter_Destroy :: proc(filter: ^BodyDrawFilter) ---

	//-------------------------------------------------------------------------------------------------
	// ContactManifold
	//-------------------------------------------------------------------------------------------------
	ContactManifold_GetWorldSpaceNormal :: proc(manifold: ^ContactManifold, result: ^Vec3) ---
	ContactManifold_GetPenetrationDepth :: proc(manifold: ^ContactManifold) -> c.float ---
	ContactManifold_GetSubShapeID1 :: proc(manifold: ^ContactManifold) -> SubShapeID ---
	ContactManifold_GetSubShapeID2 :: proc(manifold: ^ContactManifold) -> SubShapeID ---
	ContactManifold_GetPointCount :: proc(manifold: ^ContactManifold) -> c.uint32_t ---
	ContactManifold_GetWorldSpaceContactPointOn1 :: proc(manifold: ^ContactManifold, index: c.uint32_t, result: ^RVec3) ---
	ContactManifold_GetWorldSpaceContactPointOn2 :: proc(manifold: ^ContactManifold, index: c.uint32_t, result: ^RVec3) ---

	//-------------------------------------------------------------------------------------------------
	// ContactSettings
	//-------------------------------------------------------------------------------------------------
	ContactSettings_GetFriction :: proc(settings: ^ContactSettings) -> c.float ---
	ContactSettings_SetFriction :: proc(settings: ^ContactSettings, friction: c.float) ---
	ContactSettings_GetRestitution :: proc(settings: ^ContactSettings) -> c.float ---
	ContactSettings_SetRestitution :: proc(settings: ^ContactSettings, restitution: c.float) ---
	ContactSettings_GetInvMassScale1 :: proc(settings: ^ContactSettings) -> c.float ---
	ContactSettings_SetInvMassScale1 :: proc(settings: ^ContactSettings, scale: c.float) ---
	ContactSettings_GetInvInertiaScale1 :: proc(settings: ^ContactSettings) -> c.float ---
	ContactSettings_SetInvInertiaScale1 :: proc(settings: ^ContactSettings, scale: c.float) ---
	ContactSettings_GetInvMassScale2 :: proc(settings: ^ContactSettings) -> c.float ---
	ContactSettings_SetInvMassScale2 :: proc(settings: ^ContactSettings, scale: c.float) ---
	ContactSettings_GetInvInertiaScale2 :: proc(settings: ^ContactSettings) -> c.float ---
	ContactSettings_SetInvInertiaScale2 :: proc(settings: ^ContactSettings, scale: c.float) ---
	ContactSettings_GetIsSensor :: proc(settings: ^ContactSettings) -> c.bool ---
	ContactSettings_SetIsSensor :: proc(settings: ^ContactSettings, sensor: c.bool) ---
	ContactSettings_GetRelativeLinearSurfaceVelocity :: proc(settings: ^ContactSettings, result: ^Vec3) ---
	ContactSettings_SetRelativeLinearSurfaceVelocity :: proc(settings: ^ContactSettings, velocity: ^Vec3) ---
	ContactSettings_GetRelativeAngularSurfaceVelocity :: proc(settings: ^ContactSettings, result: ^Vec3) ---
	ContactSettings_SetRelativeAngularSurfaceVelocity :: proc(settings: ^ContactSettings, velocity: ^Vec3) ---

	//-------------------------------------------------------------------------------------------------
	// CharacterBase
	//-------------------------------------------------------------------------------------------------
	CharacterBase_Destroy :: proc(character: ^CharacterBase) ---
	CharacterBase_GetCosMaxSlopeAngle :: proc(character: ^CharacterBase) -> c.float ---
	CharacterBase_SetMaxSlopeAngle :: proc(character: ^CharacterBase, maxSlopeAngle: c.float) ---
	CharacterBase_GetUp :: proc(character: ^CharacterBase, result: ^Vec3) ---
	CharacterBase_SetUp :: proc(character: ^CharacterBase, value: ^Vec3) ---
	CharacterBase_IsSlopeTooSteep :: proc(character: ^CharacterBase, value: ^Vec3) -> c.bool ---
	CharacterBase_GetShape :: proc(character: ^CharacterBase) -> ^Shape ---
	CharacterBase_GetGroundState :: proc(character: ^CharacterBase) -> GroundState ---
	CharacterBase_IsSupported :: proc(character: ^CharacterBase) -> c.bool ---
	CharacterBase_GetGroundPosition :: proc(character: ^CharacterBase, position: ^RVec3) ---
	CharacterBase_GetGroundNormal :: proc(character: ^CharacterBase, normal: ^Vec3) ---
	CharacterBase_GetGroundVelocity :: proc(character: ^CharacterBase, velocity: ^Vec3) ---
	CharacterBase_GetGroundMaterial :: proc(character: ^CharacterBase) -> ^PhysicsMaterial ---
	CharacterBase_GetGroundBodyId :: proc(character: ^CharacterBase) -> BodyID ---
	CharacterBase_GetGroundSubShapeId :: proc(character: ^CharacterBase) -> SubShapeID ---
	CharacterBase_GetGroundUserData :: proc(character: ^CharacterBase) -> c.uint64_t ---

	//-------------------------------------------------------------------------------------------------
	// CharacterSettings
	//-------------------------------------------------------------------------------------------------
	CharacterSettings_Init :: proc(settings: ^CharacterSettings) ---

	//-------------------------------------------------------------------------------------------------
	// Character
	//-------------------------------------------------------------------------------------------------
	Character_Create :: proc(settings: ^CharacterSettings, position: ^RVec3, rotation: ^Quat, userData: c.uint64_t, system: ^PhysicsSystem) -> ^Character ---
	Character_AddToPhysicsSystem :: proc(character: ^Character, activationMode: Activation, lockBodies: c.bool) ---
	Character_RemoveFromPhysicsSystem :: proc(character: ^Character, lockBodies: c.bool) ---
	Character_Activate :: proc(character: ^Character, lockBodies: c.bool) ---
	Character_PostSimulation :: proc(character: ^Character, maxSeparationDistance: c.float, lockBodies: c.bool) ---
	Character_SetLinearAndAngularVelocity :: proc(character: ^Character, linearVelocity: ^Vec3, angularVelocity: ^Vec3, lockBodies: c.bool) ---
	Character_GetLinearVelocity :: proc(character: ^Character, result: ^Vec3) ---
	Character_SetLinearVelocity :: proc(character: ^Character, value: ^Vec3, lockBodies: c.bool) ---
	Character_AddLinearVelocity :: proc(character: ^Character, value: ^Vec3, lockBodies: c.bool) ---
	Character_AddImpulse :: proc(character: ^Character, value: ^Vec3, lockBodies: c.bool) ---
	Character_GetBodyID :: proc(character: ^Character) -> BodyID ---
	Character_GetPositionAndRotation :: proc(character: ^Character, position: ^RVec3, rotation: ^Quat, lockBodies: c.bool) ---
	Character_SetPositionAndRotation :: proc(character: ^Character, position: ^RVec3, rotation: ^Quat, activationMode: Activation, lockBodies: c.bool) ---
	Character_GetPosition :: proc(character: ^Character, position: ^RVec3, lockBodies: c.bool) ---
	Character_SetPosition :: proc(character: ^Character, position: ^RVec3, activationMode: Activation, lockBodies: c.bool) ---
	Character_GetRotation :: proc(character: ^Character, rotation: ^Quat, lockBodies: c.bool) ---
	Character_SetRotation :: proc(character: ^Character, rotation: ^Quat, activationMode: Activation, lockBodies: c.bool) ---
	Character_GetCenterOfMassPosition :: proc(character: ^Character, result: ^RVec3, lockBodies: c.bool) ---
	Character_GetWorldTransform :: proc(character: ^Character, result: ^RMatrix4x4, lockBodies: c.bool) ---
	Character_GetLayer :: proc(character: ^Character) -> ObjectLayer ---
	Character_SetLayer :: proc(character: ^Character, value: ObjectLayer, lockBodies: c.bool) ---
	Character_SetShape :: proc(character: ^Character, shape: ^Shape, maxPenetrationDepth: c.float, lockBodies: c.bool) ---

	//-------------------------------------------------------------------------------------------------
	// CharacterVirtualSettings
	//-------------------------------------------------------------------------------------------------
	CharacterVirtualSettings_Init :: proc(settings: ^CharacterVirtualSettings) ---

	//-------------------------------------------------------------------------------------------------
	// CharacterVirtual
	//-------------------------------------------------------------------------------------------------
	CharacterVirtual_Create :: proc(settings: ^CharacterVirtualSettings, position: ^RVec3, rotation: ^Quat, userData: c.uint64_t, system: ^PhysicsSystem) -> ^CharacterVirtual ---
	CharacterVirtual_GetID :: proc(character: ^CharacterVirtual) -> CharacterID ---
	CharacterVirtual_SetListener :: proc(character: ^CharacterVirtual, listener: ^CharacterContactListener) ---
	CharacterVirtual_SetCharacterVsCharacterCollision :: proc(character: ^CharacterVirtual, characterVsCharacterCollision: ^CharacterVsCharacterCollision) ---
	CharacterVirtual_GetLinearVelocity :: proc(character: ^CharacterVirtual, velocity: ^Vec3) ---
	CharacterVirtual_SetLinearVelocity :: proc(character: ^CharacterVirtual, velocity: ^Vec3) ---
	CharacterVirtual_GetPosition :: proc(character: ^CharacterVirtual, position: ^RVec3) ---
	CharacterVirtual_SetPosition :: proc(character: ^CharacterVirtual, position: ^RVec3) ---
	CharacterVirtual_GetRotation :: proc(character: ^CharacterVirtual, rotation: ^Quat) ---
	CharacterVirtual_SetRotation :: proc(character: ^CharacterVirtual, rotation: ^Quat) ---
	CharacterVirtual_GetWorldTransform :: proc(character: ^CharacterVirtual, result: ^RMatrix4x4) ---
	CharacterVirtual_GetCenterOfMassTransform :: proc(character: ^CharacterVirtual, result: ^RMatrix4x4) ---
	CharacterVirtual_GetMass :: proc(character: ^CharacterVirtual) -> c.float ---
	CharacterVirtual_SetMass :: proc(character: ^CharacterVirtual, value: c.float) ---
	CharacterVirtual_GetMaxStrength :: proc(character: ^CharacterVirtual) -> c.float ---
	CharacterVirtual_SetMaxStrength :: proc(character: ^CharacterVirtual, value: c.float) ---
	CharacterVirtual_GetPenetrationRecoverySpeed :: proc(character: ^CharacterVirtual) -> c.float ---
	CharacterVirtual_SetPenetrationRecoverySpeed :: proc(character: ^CharacterVirtual, value: c.float) ---
	CharacterVirtual_GetEnhancedInternalEdgeRemoval :: proc(character: ^CharacterVirtual) -> c.bool ---
	CharacterVirtual_SetEnhancedInternalEdgeRemoval :: proc(character: ^CharacterVirtual, value: c.bool) ---
	CharacterVirtual_GetCharacterPadding :: proc(character: ^CharacterVirtual) -> c.float ---
	CharacterVirtual_GetMaxNumHits :: proc(character: ^CharacterVirtual) -> c.uint32_t ---
	CharacterVirtual_SetMaxNumHits :: proc(character: ^CharacterVirtual, value: c.uint32_t) ---
	CharacterVirtual_GetHitReductionCosMaxAngle :: proc(character: ^CharacterVirtual) -> c.float ---
	CharacterVirtual_SetHitReductionCosMaxAngle :: proc(character: ^CharacterVirtual, value: c.float) ---
	CharacterVirtual_GetMaxHitsExceeded :: proc(character: ^CharacterVirtual) -> c.bool ---
	CharacterVirtual_GetShapeOffset :: proc(character: ^CharacterVirtual, result: ^Vec3) ---
	CharacterVirtual_SetShapeOffset :: proc(character: ^CharacterVirtual, value: ^Vec3) ---
	CharacterVirtual_GetUserData :: proc(character: ^CharacterVirtual) -> c.uint64_t ---
	CharacterVirtual_SetUserData :: proc(character: ^CharacterVirtual, value: c.uint64_t) ---
	CharacterVirtual_GetInnerBodyID :: proc(character: ^CharacterVirtual) -> BodyID ---
	CharacterVirtual_CancelVelocityTowardsSteepSlopes :: proc(character: ^CharacterVirtual, desiredVelocity: ^Vec3, velocity: ^Vec3) ---
	CharacterVirtual_StartTrackingContactChanges :: proc(character: ^CharacterVirtual) ---
	CharacterVirtual_FinishTrackingContactChanges :: proc(Character: ^CharacterVirtual) ---
	CharacterVirtual_Update :: proc(character: ^CharacterVirtual, deltaTime: c.float, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---
	CharacterVirtual_ExtendedUpdate :: proc(character: ^CharacterVirtual, deltaTime: c.float, settings: ^ExtendedUpdateSettings, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---
	CharacterVirtual_RefreshContacts :: proc(character: ^CharacterVirtual, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) ---
	CharacterVirtual_CanWalkStairs :: proc(character: ^CharacterVirtual, linearVelocity: ^Vec3) -> c.bool ---
	CharacterVirtual_WalkStairs :: proc(character: ^CharacterVirtual, deltaTime: c.float, stepUp: ^Vec3, stepForward: ^Vec3, stepForwardTest: ^Vec3, stepDownExtra: ^Vec3, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	CharacterVirtual_StickToFloor :: proc(character: ^CharacterVirtual, stepDown: ^Vec3, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	CharacterVirtual_UpdateGroundVelocity :: proc(character: ^CharacterVirtual) ---
	CharacterVirtual_SetShape :: proc(character: ^CharacterVirtual, shape: ^Shape, maxPenetrationDepth: c.float, layer: ObjectLayer, system: ^PhysicsSystem, bodyFilter: ^BodyFilter, shapeFilter: ^ShapeFilter) -> c.bool ---
	CharacterVirtual_SetInnerBodyShape :: proc(character: ^CharacterVirtual, shape: ^Shape) ---
	CharacterVirtual_GetNumContacts :: proc(character: ^CharacterVirtual) -> c.uint32_t ---
	CharacterVirtual_HasCollidedWithBody :: proc(character: ^CharacterVirtual, body: BodyID) -> c.bool ---
	CharacterVirtual_HasCollidedWith :: proc(character: ^CharacterVirtual, other: ^CharacterID) -> c.bool ---
	CharacterVirtual_HasCollidedWithCharacter :: proc(character: ^CharacterVirtual, other: ^CharacterVirtual) -> c.bool ---

	//-------------------------------------------------------------------------------------------------
	// CharacterContactListener
	//-------------------------------------------------------------------------------------------------
	CharacterContactListener_Create :: proc(procs: CharacterContactListener_Procs, userData: rawptr) -> ^CharacterContactListener ---
	CharacterContactListener_Destroy :: proc(listener: ^CharacterContactListener) ---

	//-------------------------------------------------------------------------------------------------
	// CharacterVsCharacterCollision
	//-------------------------------------------------------------------------------JPH_------------------
	CharacterVsCharacterCollision_Create :: proc(procs: CharacterVsCharacterCollision_Procs, userData: rawptr) -> ^CharacterVsCharacterCollision ---
	CharacterVsCharacterCollision_CreateSimple :: proc() -> ^CharacterVsCharacterCollision ---
	CharacterVsCharacterCollisionSimple_AddCharacter :: proc(characterVsCharacter: ^CharacterVsCharacterCollision, character: ^CharacterVirtual) ---
	CharacterVsCharacterCollisionSimple_RemoveCharacter :: proc(characterVsCharacter: ^CharacterVsCharacterCollision, character: ^CharacterVirtual) ---
	CharacterVsCharacterCollision_Destroy :: proc(listener: ^CharacterVsCharacterCollision) ---

	//-------------------------------------------------------------------------------------------------
	// DebugRenderer
	//-------------------------------------------------------------------------------------------------
	DebugRenderer_Create :: proc(procs: DebugRenderer_Procs, userData: rawptr) -> ^DebugRenderer ---
	DebugRenderer_Destroy :: proc(renderer: ^DebugRenderer) ---
	DebugRenderer_NextFrame :: proc(renderer: ^DebugRenderer) ---
	DebugRenderer_DrawLine :: proc(renderer: ^DebugRenderer, from: ^RVec3, to: ^RVec3, color: Color) ---
	DebugRenderer_DrawWireBox :: proc(renderer: ^DebugRenderer, box: ^AABox, color: Color) ---
	DebugRenderer_DrawWireBox2 :: proc(renderer: ^DebugRenderer, mat: ^RMatrix4x4, box: ^AABox, color: Color) ---
	DebugRenderer_DrawMarker :: proc(renderer: ^DebugRenderer, position: ^RVec3, color: Color, size: c.float) ---
	DebugRenderer_DrawArrow :: proc(renderer: ^DebugRenderer, from: ^RVec3, to: ^RVec3, color: Color, size: c.float) ---
	DebugRenderer_DrawCoordinateSystem :: proc(renderer: ^DebugRenderer, mat: ^RMatrix4x4, size: c.float) ---
	DebugRenderer_DrawPlane :: proc(renderer: ^DebugRenderer, point: ^RVec3, normal: ^Vec3, color: Color, size: c.float) ---
	DebugRenderer_DrawWireTriangle :: proc(renderer: ^DebugRenderer, v1: ^RVec3, v2: ^RVec3, v3: ^RVec3, color: Color) ---
	DebugRenderer_DrawWireSphere :: proc(renderer: ^DebugRenderer, center: ^RVec3, radius: c.float, color: Color, level: c.int) ---
	DebugRenderer_DrawWireUnitSphere :: proc(renderer: ^DebugRenderer, mat: ^RMatrix4x4, color: Color, level: c.int) ---
}

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

	my_contact_listener := ContactListener_Create(my_contact_listener_procs, nil)
	defer ContactListener_Destroy(my_contact_listener)

	PhysicsSystem_SetContactListener(physics_system, my_contact_listener)

	my_activation_listener_proc: BodyActivationListener_Procs
	my_activation_listener_proc.OnBodyActivated =
	proc "c" (userData: rawptr, bodyID: BodyID, bodyUserData: c.uint64_t) {
		context = runtime.default_context()
		fmt.println("[BodyActivationListener] A body got activated")
	}
	my_activation_listener_proc.OnBodyDeactivated =
	proc "c" (userData: rawptr, bodyID: BodyID, bodyUserData: c.uint64_t) {
		context = runtime.default_context()
		fmt.println("[BodyActivationListener] A body went to sleep")
	}

	my_activation_listener := BodyActivationListener_Create(my_activation_listener_proc, nil)
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

		sphere_active = BodyInterface_IsActive(body_interface, sphere_id)
	}
	testing.expect_value(t, sphere_active, false)
}
