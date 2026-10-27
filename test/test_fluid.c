// SPDX-License-Identifier: MIT

// MineMogul fork: fluid stage tests. See docs/fluid.md "Tests".

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846
#define WATER_DENSITY 1000.0f

// ENSURE_SMALL that fails on NaN
#define ENSURE_NEAR( C, tol )                                                                      \
	do                                                                                             \
	{                                                                                              \
		double value_ = (double)( C );                                                             \
		if ( !( fabs( value_ ) <= (double)( tol ) ) )                                              \
		{                                                                                          \
			printf( "condition false: abs(" #C ") = %g > %g\n", fabs( value_ ), (double)( tol ) ); \
			return 1;                                                                              \
		}                                                                                          \
	}                                                                                              \
	while ( false )

// A pool whose surface is the plane y = 0
static b3FluidVolumeId CreatePool( b3WorldId worldId, float halfWidth, float depth )
{
	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds =
		(b3AABB){ { -halfWidth, -depth, -halfWidth }, { halfWidth, 0.0f, halfWidth } };
	volumeDef.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.0f };
	return b3CreateFluidVolume( worldId, &volumeDef );
}

// Drag does not move an equilibrium, so the draft tests use heavy drag to settle quickly
static b3FluidVolumeId CreateDampedPool( b3WorldId worldId, float halfWidth, float depth )
{
	b3FluidVolumeId volumeId = CreatePool( worldId, halfWidth, depth );
	b3FluidVolume_SetDrag( volumeId, 10.0f, 10.0f, 1.0f );
	return volumeId;
}

// A basin with a horizontal surface at the given level
static b3FluidVolumeId CreateBasin( b3WorldId worldId, b3AABB bounds, float level,
									b3Vec3 flowVelocity )
{
	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = bounds;
	volumeDef.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, level };
	volumeDef.flowVelocity = flowVelocity;
	return b3CreateFluidVolume( worldId, &volumeDef );
}

// Sets the displaced volume that gives the body a specific gravity in water
static void SetSpecificGravity( b3BodyId bodyId, float specificGravity )
{
	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.displacedVolume =
		b3Body_GetMass( bodyId ) / ( specificGravity * b3DefaultFluidVolumeDef().density );
	b3Body_SetFluidDef( bodyId, &fluidDef );
}

static b3BodyId CreateDynamicBody( b3WorldId worldId, b3Pos position, b3Quat rotation,
								   bool enableSleep )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = position;
	bodyDef.rotation = rotation;
	bodyDef.enableSleep = enableSleep;
	return b3CreateBody( worldId, &bodyDef );
}

static b3BodyId CreateBox( b3WorldId worldId, b3Pos position, b3Quat rotation, b3Vec3 halfExtents,
						   float density, bool enableSleep )
{
	b3BodyId bodyId = CreateDynamicBody( worldId, position, rotation, enableSleep );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = density;
	b3BoxHull box = b3MakeBoxHull( halfExtents.x, halfExtents.y, halfExtents.z );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
	return bodyId;
}

static b3BodyId CreateBall( b3WorldId worldId, b3Pos position, float radius, float density,
							bool enableSleep )
{
	b3BodyId bodyId = CreateDynamicBody( worldId, position, b3Quat_identity, enableSleep );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = density;
	b3Sphere sphere = { b3Vec3_zero, radius };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	return bodyId;
}

// Capsule along the local axis
static b3BodyId CreateRod( b3WorldId worldId, b3Pos position, b3Quat rotation, b3Vec3 axis,
						   float halfLength, float radius, float density, bool enableSleep )
{
	b3BodyId bodyId = CreateDynamicBody( worldId, position, rotation, enableSleep );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = density;
	b3Capsule capsule = { b3MulSV( -halfLength, axis ), b3MulSV( halfLength, axis ), radius };
	b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
	return bodyId;
}

static void CreateSlab( b3WorldId worldId, b3Vec3 center, b3Vec3 halfExtents )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = b3ToPos( center );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( halfExtents.x, halfExtents.y, halfExtents.z );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
}

static void LockRotation( b3BodyId bodyId )
{
	b3MotionLocks locks = { 0 };
	locks.angularX = true;
	locks.angularY = true;
	locks.angularZ = true;
	b3Body_SetMotionLocks( bodyId, locks );
}

static void Simulate( b3WorldId worldId, float hertz, int subStepCount, float seconds )
{
	int stepCount = (int)( seconds * hertz + 0.5f );
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, subStepCount );
	}
}

static float GetHeight( b3BodyId bodyId )
{
	return (float)b3Body_GetPosition( bodyId ).y;
}

static b3Vec3 GetUpAxis( b3BodyId bodyId )
{
	return b3RotateVector( b3Body_GetRotation( bodyId ), b3Vec3_axisY );
}

// Sine of the angle between the body up axis and the world up axis
static float GetTiltSine( b3BodyId bodyId )
{
	b3Vec3 up = GetUpAxis( bodyId );
	return sqrtf( up.x * up.x + up.z * up.z );
}

static bool IsFiniteBody( b3BodyId bodyId )
{
	return b3IsValidPosition( b3Body_GetPosition( bodyId ) ) &&
		   b3IsValidVec3( b3Body_GetLinearVelocity( bodyId ) ) &&
		   b3IsValidVec3( b3Body_GetAngularVelocity( bodyId ) );
}

static bool IsDefEqual( b3BodyFluidDef a, b3BodyFluidDef b )
{
	return a.displacedVolume == b.displacedVolume && a.dragScale == b.dragScale &&
		   a.enabled == b.enabled;
}

static void CountEvents( b3WorldId worldId, int* beginCount, int* endCount )
{
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	*beginCount += events.beginCount;
	*endCount += events.endCount;
}

static uint32_t HashBeginEvent( uint32_t hash, const b3FluidBeginTouchEvent* event )
{
	hash = b3Hash( hash, (const uint8_t*)&event->bodyId, (int)sizeof( b3BodyId ) );
	hash = b3Hash( hash, (const uint8_t*)&event->volumeId, (int)sizeof( b3FluidVolumeId ) );
	hash = b3Hash( hash, (const uint8_t*)&event->point, (int)sizeof( b3Pos ) );
	hash = b3Hash( hash, (const uint8_t*)&event->velocity, (int)sizeof( b3Vec3 ) );
	return hash;
}

static uint32_t HashEndEvent( uint32_t hash, const b3FluidEndTouchEvent* event )
{
	hash = b3Hash( hash, (const uint8_t*)&event->bodyId, (int)sizeof( b3BodyId ) );
	hash = b3Hash( hash, (const uint8_t*)&event->volumeId, (int)sizeof( b3FluidVolumeId ) );
	return hash;
}

static uint32_t HashBody( uint32_t hash, b3BodyId bodyId )
{
	b3WorldTransform transform = b3Body_GetTransform( bodyId );
	b3Vec3 linearVelocity = b3Body_GetLinearVelocity( bodyId );
	b3Vec3 angularVelocity = b3Body_GetAngularVelocity( bodyId );
	float fraction = b3Body_GetSubmergedFraction( bodyId );
	b3FluidVolumeId volumeId = b3Body_GetFluidVolume( bodyId );
	hash = b3Hash( hash, (const uint8_t*)&transform.p, (int)sizeof( b3Pos ) );
	hash = b3Hash( hash, (const uint8_t*)&transform.q, (int)sizeof( b3Quat ) );
	hash = b3Hash( hash, (const uint8_t*)&linearVelocity, (int)sizeof( b3Vec3 ) );
	hash = b3Hash( hash, (const uint8_t*)&angularVelocity, (int)sizeof( b3Vec3 ) );
	hash = b3Hash( hash, (const uint8_t*)&fraction, (int)sizeof( float ) );
	hash = b3Hash( hash, (const uint8_t*)&volumeId, (int)sizeof( b3FluidVolumeId ) );
	return hash;
}

// Analytic references in double precision

static double SphereCapVolume( double radius, double height )
{
	return TEST_PI * height * height * ( 3.0 * radius - height ) / 3.0;
}

static double SphereVolume( double radius )
{
	return 4.0 / 3.0 * TEST_PI * radius * radius * radius;
}

static double CapsuleVolume( double radius, double length )
{
	return TEST_PI * radius * radius * length + SphereVolume( radius );
}

// Area of a disc below a chord at the given offset toward the surface
static double DiscAreaBelow( double radius, double offset )
{
	if ( offset >= radius )
	{
		return TEST_PI * radius * radius;
	}

	if ( offset <= -radius )
	{
		return 0.0;
	}

	double root = sqrt( fmax( radius * radius - offset * offset, 0.0 ) );
	return TEST_PI * radius * radius - radius * radius * acos( offset / radius ) + offset * root;
}

typedef double SubmergedVolumeFcn( const double* shape, double depth );

// Volume of a sphere {radius} submerged to the given height
static double SubmergedSphere( const double* shape, double depth )
{
	return SphereCapVolume( shape[0], depth );
}

// Volume of a level capsule {radius, length} submerged to the given height
static double SubmergedLevelCapsule( const double* shape, double depth )
{
	double radius = shape[0];
	return shape[1] * DiscAreaBelow( radius, depth - radius ) + SphereCapVolume( radius, depth );
}

// Volume of an upright capsule {radius, length} submerged to the given height
static double SubmergedUprightCapsule( const double* shape, double depth )
{
	double radius = shape[0];
	double length = shape[1];
	if ( depth <= radius )
	{
		return SphereCapVolume( radius, depth );
	}

	double halfSphere = SphereCapVolume( radius, radius );
	if ( depth <= radius + length )
	{
		return halfSphere + TEST_PI * radius * radius * ( depth - radius );
	}

	double t = depth - radius - length;
	return halfSphere + TEST_PI * radius * radius * length +
		   TEST_PI * ( radius * radius * t - t * t * t / 3.0 );
}

// Depth in [0, maxDepth] where the submerged volume reaches the target
static double SolveDraft( SubmergedVolumeFcn* fcn, const double* shape, double target,
						  double maxDepth )
{
	double lower = 0.0;
	double upper = maxDepth;
	for ( int i = 0; i < 100; ++i )
	{
		double mid = 0.5 * ( lower + upper );
		if ( fcn( shape, mid ) < target )
		{
			lower = mid;
		}
		else
		{
			upper = mid;
		}
	}

	return 0.5 * ( lower + upper );
}

// Volume of an origin-centered capsule below y = level, by fine slices along the axis
static double TiltedCapsuleVolume( double cosine, double sine, double halfLength, double radius,
								   double level )
{
	if ( sine < 1e-6 )
	{
		double shape[2] = { radius, 2.0 * halfLength };
		double depth =
			fmin( fmax( level + halfLength + radius, 0.0 ), 2.0 * ( halfLength + radius ) );
		return SubmergedUprightCapsule( shape, depth );
	}

	int sliceCount = 8000;
	double extent = halfLength + radius;
	double ds = 2.0 * extent / sliceCount;
	double volume = 0.0;
	for ( int i = 0; i < sliceCount; ++i )
	{
		double s = -extent + ( i + 0.5 ) * ds;
		double t = fabs( s ) - halfLength;
		double rho = t > 0.0 ? sqrt( fmax( radius * radius - t * t, 0.0 ) ) : radius;
		volume += ds * DiscAreaBelow( rho, ( level - s * cosine ) / sine );
	}

	return volume;
}

// The vector with the given value on one world axis and zero on the others
static b3Vec3 MakeAxisVector( int axis, float value )
{
	return (b3Vec3){ axis == 0 ? value : 0.0f, axis == 1 ? value : 0.0f, axis == 2 ? value : 0.0f };
}

static float GetAxisComponent( b3Vec3 v, int axis )
{
	return axis == 0 ? v.x : ( axis == 1 ? v.y : v.z );
}

// Gravity, the surface normal and the bounds axis lie on the given axis, up being +1 or -1. The
// volume is turned about that axis.
static int BoxDraft( float hertz, int subStepCount, int axis, float up, float gravityScale,
					 float rotation, float* height )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = MakeAxisVector( axis, -10.0f * up );
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// The fluid side spans 5 m below the surface
	b3Vec3 lower = { -5.0f, -5.0f, -5.0f };
	b3Vec3 upper = { 5.0f, 5.0f, 5.0f };
	if ( up > 0.0f )
	{
		upper = b3Sub( upper, MakeAxisVector( axis, 5.0f ) );
	}
	else
	{
		lower = b3Add( lower, MakeAxisVector( axis, 5.0f ) );
	}

	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = (b3AABB){ lower, upper };
	volumeDef.rotation = rotation;
	volumeDef.surface = (b3Plane){ MakeAxisVector( axis, up ), 0.0f };
	b3CreateFluidVolume( worldId, &volumeDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = b3ToPos( MakeAxisVector( axis, 0.1f * up ) );
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.gravityScale = gravityScale;
	bodyDef.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	SetSpecificGravity( bodyId, 0.5f );

	float timeStep = 1.0f / hertz;
	int stepCount = (int)( 20.0f * hertz );
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Half draft puts the center on the surface, with no drift off the axis
	b3Vec3 position = b3ToVec3( b3Body_GetPosition( bodyId ) );
	b3Vec3 velocity = b3Body_GetLinearVelocity( bodyId );
	*height = up * GetAxisComponent( position, axis );
	ENSURE_NEAR( *height, 0.002f );
	ENSURE_NEAR( GetAxisComponent( velocity, axis ), 0.001f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( bodyId ) - 0.5f, 0.005f );
	ENSURE_NEAR( b3Length( b3Sub( position, MakeAxisVector( axis, up * *height ) ) ), 1e-6f );

	b3FluidStats stats = b3World_GetFluidStats( worldId );
	ENSURE( stats.volumeCount == 1 );
	ENSURE( stats.bodyCount == 1 );
	ENSURE( stats.clippedShapeCount == 1 );

	b3DestroyWorld( worldId );
	return 0;
}

// A free cube is upright-stable below specific gravity 0.21 and above 0.79
static int FreeBoxDraft( float specificGravity )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	float halfHeight = 0.25f;
	float height = halfHeight - specificGravity * 2.0f * halfHeight;
	b3Quat rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.1f );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, height + 0.02f, 0.0f }, rotation,
								 (b3Vec3){ halfHeight, halfHeight, halfHeight },
								 specificGravity * WATER_DENSITY, false );

	Simulate( worldId, 50.0f, 2, 30.0f );

	ENSURE( GetUpAxis( bodyId ).y > 0.0f );
	ENSURE( GetTiltSine( bodyId ) < 1e-3f );
	ENSURE_NEAR( GetHeight( bodyId ) - height, 1e-4f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( bodyId ) - specificGravity, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// A 0.5 m box settles at the draft of its specific gravity
static int BoxEquilibrium( void )
{
	float height;
	ENSURE( BoxDraft( 60.0f, 4, 1, 1.0f, 1.0f, 0.0f, &height ) == 0 );
	ENSURE( BoxDraft( 50.0f, 2, 1, 1.0f, 1.0f, 0.0f, &height ) == 0 );
	ENSURE( FreeBoxDraft( 0.15f ) == 0 );
	ENSURE( FreeBoxDraft( 0.85f ) == 0 );
	return 0;
}

// Gravity and the surface along x, then z, then down y give the draft found along y
static int AxisFrames( void )
{
	float reference;
	ENSURE( BoxDraft( 50.0f, 2, 1, 1.0f, 1.0f, 0.0f, &reference ) == 0 );

	int axes[3] = { 0, 2, 1 };
	float ups[3] = { 1.0f, 1.0f, -1.0f };
	for ( int i = 0; i < 3; ++i )
	{
		float height;
		ENSURE( BoxDraft( 50.0f, 2, axes[i], ups[i], 1.0f, 0.0f, &height ) == 0 );
		ENSURE_NEAR( height - reference, 1e-5f );
	}

	return 0;
}

// An oblique plank of specific gravity 0.5 centered at the given height
static b3BodyId CreateObliquePlank( b3WorldId worldId, float height, float gravityScale )
{
	b3Quat rotation = b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 2.0f, 0.5f } ), 0.7f );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.5f, height, -0.5f }, rotation,
								 (b3Vec3){ 0.4f, 0.02f, 0.15f }, 500.0f, false );
	b3Body_SetGravityScale( bodyId, gravityScale );
	return bodyId;
}

// Angular velocity of a plank released across the surface without drag
static b3Vec3 PlankSpinUp( float gravity, float gravityScale )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, gravity, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );
	b3FluidVolume_SetDrag( volumeId, 0.0f, 0.0f, 0.0f );

	b3BodyId bodyId = CreateObliquePlank( worldId, 0.0f, gravityScale );
	Simulate( worldId, 60.0f, 4, 0.1f );

	b3Vec3 angularVelocity = b3Body_GetAngularVelocity( bodyId );
	b3DestroyWorld( worldId );
	return angularVelocity;
}

// Buoyancy scales with gravityScale, so the draft does not change and zero leaves a body at rest
static int GravityScale( void )
{
	float reference;
	float height;
	ENSURE( BoxDraft( 50.0f, 2, 1, 1.0f, 1.0f, 0.0f, &reference ) == 0 );
	ENSURE( BoxDraft( 50.0f, 2, 1, 1.0f, 2.0f, 0.0f, &height ) == 0 );
	ENSURE_NEAR( height - reference, 5e-4f );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );

	// Free planks fully submerged and across the surface, where the buoyancy has an arm
	b3BodyId bodyIds[2] = { CreateObliquePlank( worldId, -1.0f, 0.0f ),
							CreateObliquePlank( worldId, 0.0f, 0.0f ) };
	b3WorldTransform starts[2] = { b3Body_GetTransform( bodyIds[0] ),
								   b3Body_GetTransform( bodyIds[1] ) };

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyIds[0] ), volumeId ) );
		ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyIds[1] ), volumeId ) );
		ENSURE( b3Body_GetSubmergedFraction( bodyIds[0] ) == 1.0f );

		float fraction = b3Body_GetSubmergedFraction( bodyIds[1] );
		ENSURE( 0.0f < fraction && fraction < 1.0f );
	}

	for ( int i = 0; i < 2; ++i )
	{
		b3WorldTransform transform = b3Body_GetTransform( bodyIds[i] );
		ENSURE_NEAR( b3Length( b3SubPos( transform.p, starts[i].p ) ), 1e-6f );
		ENSURE_NEAR( b3Length( b3InvMulQuat( starts[i].q, transform.q ).v ), 1e-6f );
		ENSURE_NEAR( b3Length( b3Body_GetLinearVelocity( bodyIds[i] ) ), 1e-6f );
		ENSURE_NEAR( b3Length( b3Body_GetAngularVelocity( bodyIds[i] ) ), 1e-6f );
	}

	b3DestroyWorld( worldId );

	// Doubling the gravity scale matches doubling the gravity, buoyancy torque included
	b3Vec3 scaled = PlankSpinUp( -10.0f, 2.0f );
	b3Vec3 strong = PlankSpinUp( -20.0f, 1.0f );
	ENSURE( b3Length( strong ) > 0.1f );
	ENSURE_NEAR( b3Length( b3Sub( scaled, strong ) ) / b3Length( strong ), 1e-6f );

	return 0;
}

static int SphereDraft( float specificGravity, float hertz, int subStepCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	float radius = 0.25f;
	double shape[1] = { radius };
	double draft = SolveDraft( SubmergedSphere, shape, specificGravity * SphereVolume( radius ),
							   2.0 * radius );
	float height = (float)( radius - draft );

	b3BodyId bodyId = CreateBall( worldId, (b3Pos){ 0.0f, height + 0.05f, 0.0f }, radius,
								  specificGravity * WATER_DENSITY, false );
	Simulate( worldId, hertz, subStepCount, 30.0f );

	ENSURE_NEAR( GetHeight( bodyId ) - height, 1e-4f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( bodyId ) - specificGravity, 1e-4f );
	ENSURE_NEAR( b3Length( b3Body_GetLinearVelocity( bodyId ) ), 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int SphereEquilibrium( void )
{
	float specificGravities[3] = { 0.1f, 0.5f, 0.9f };
	for ( int i = 0; i < 3; ++i )
	{
		ENSURE( SphereDraft( specificGravities[i], 60.0f, 4 ) == 0 );
		ENSURE( SphereDraft( specificGravities[i], 50.0f, 2 ) == 0 );
	}

	return 0;
}

// A free capsule lies level at the draft of its specific gravity
static int LevelCapsuleDraft( float specificGravity )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	float radius = 0.1f;
	float halfLength = 0.25f;
	double shape[2] = { radius, 2.0 * halfLength };
	double target = specificGravity * CapsuleVolume( radius, 2.0 * halfLength );
	float height =
		(float)( radius - SolveDraft( SubmergedLevelCapsule, shape, target, 2.0 * radius ) );

	b3Quat rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.2f );
	b3Vec3 axis = { 1.0f, 0.0f, 0.0f };
	b3BodyId bodyId = CreateRod( worldId, (b3Pos){ 0.0f, height + 0.03f, 0.0f }, rotation, axis,
								 halfLength, radius, specificGravity * WATER_DENSITY, false );

	Simulate( worldId, 50.0f, 2, 30.0f );

	b3Vec3 worldAxis = b3RotateVector( b3Body_GetRotation( bodyId ), axis );
	ENSURE( fabsf( worldAxis.y ) < 1e-3f );
	ENSURE_NEAR( GetHeight( bodyId ) - height, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// Fixed rotation: the waterline crosses the lower cap, the cylinder, then the upper cap
static int UprightCapsuleDraft( float specificGravity )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	float radius = 0.1f;
	float halfLength = 0.3f;
	double shape[2] = { radius, 2.0 * halfLength };
	double target = specificGravity * CapsuleVolume( radius, 2.0 * halfLength );
	double draft =
		SolveDraft( SubmergedUprightCapsule, shape, target, 2.0 * ( radius + halfLength ) );
	float height = (float)( halfLength + radius - draft );

	b3BodyId bodyId =
		CreateRod( worldId, (b3Pos){ 0.0f, height + 0.03f, 0.0f }, b3Quat_identity, b3Vec3_axisY,
				   halfLength, radius, specificGravity * WATER_DENSITY, false );
	LockRotation( bodyId );

	Simulate( worldId, 50.0f, 2, 30.0f );

	ENSURE_NEAR( GetHeight( bodyId ) - height, 1e-4f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( bodyId ) - specificGravity, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

static int CapsuleEquilibrium( void )
{
	ENSURE( LevelCapsuleDraft( 0.3f ) == 0 );
	ENSURE( LevelCapsuleDraft( 0.7f ) == 0 );
	ENSURE( UprightCapsuleDraft( 0.05f ) == 0 );
	ENSURE( UprightCapsuleDraft( 0.5f ) == 0 );
	ENSURE( UprightCapsuleDraft( 0.97f ) == 0 );
	return 0;
}

// Buoyancy uses the def's displaced volume, never the mass
static int DisplacedVolumeOverride( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	b3Vec3 halfExtents = { 0.25f, 0.25f, 0.25f };
	float volume = 0.125f;

	// Geometric volume, specific gravity 0.5
	b3BodyId geometricId = CreateBox( worldId, (b3Pos){ -2.0f, 0.1f, 0.0f }, b3Quat_identity,
									  halfExtents, 500.0f, false );

	// Sinks by its geometry, floats at half draft with four times the volume
	b3BodyId heavyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.1f, 0.0f }, b3Quat_identity,
								  halfExtents, 2000.0f, false );

	// Neutral by its geometry, floats at 0.4 draft
	b3BodyId neutralId = CreateBox( worldId, (b3Pos){ 2.0f, 0.1f, 0.0f }, b3Quat_identity,
									halfExtents, WATER_DENSITY, false );

	LockRotation( geometricId );
	LockRotation( heavyId );
	LockRotation( neutralId );

	b3BodyFluidDef heavyDef = b3DefaultBodyFluidDef();
	heavyDef.displacedVolume = 4.0f * volume;
	b3Body_SetFluidDef( heavyId, &heavyDef );

	b3BodyFluidDef neutralDef = b3DefaultBodyFluidDef();
	neutralDef.displacedVolume = 2.5f * volume;
	b3Body_SetFluidDef( neutralId, &neutralDef );

	ENSURE( IsDefEqual( b3Body_GetFluidDef( heavyId ), heavyDef ) );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( neutralId ), neutralDef ) );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( geometricId ), b3DefaultBodyFluidDef() ) );

	Simulate( worldId, 50.0f, 2, 30.0f );

	ENSURE_NEAR( GetHeight( geometricId ), 1e-4f );
	ENSURE_NEAR( GetHeight( heavyId ), 1e-4f );
	ENSURE_NEAR( GetHeight( neutralId ) - 0.05f, 1e-4f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( neutralId ) - 0.4f, 1e-4f );

	// Changing the def on a floater moves its equilibrium to 0.2 draft
	neutralDef.displacedVolume = 5.0f * volume;
	b3Body_SetFluidDef( neutralId, &neutralDef );
	Simulate( worldId, 50.0f, 2, 30.0f );
	ENSURE_NEAR( GetHeight( neutralId ) - 0.15f, 1e-4f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( neutralId ) - 0.2f, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// A tilted plank settles flat
static int PlankFloatsFlat( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	b3Quat roll = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.15f );
	b3Quat pitch = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.1f );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.02f, 0.0f }, b3MulQuat( pitch, roll ),
								 (b3Vec3){ 0.4f, 0.02f, 0.15f }, 500.0f, false );

	Simulate( worldId, 50.0f, 2, 30.0f );

	ENSURE( GetUpAxis( bodyId ).y > 0.0f );
	ENSURE( GetTiltSine( bodyId ) < 1e-3f );
	ENSURE_NEAR( GetHeight( bodyId ), 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// A float with heavy ballast below it rights itself from nearly upside down
static int BottomHeavyRights( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateDampedPool( worldId, 5.0f, 5.0f );

	b3Quat rotation =
		b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 160.0f * B3_PI / 180.0f );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, rotation,
								 (b3Vec3){ 0.2f, 0.1f, 0.2f }, 300.0f, false );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 20000.0f;
	b3Sphere ballast = { { 0.0f, -0.25f, 0.0f }, 0.05f };
	b3CreateSphereShape( bodyId, &shapeDef, &ballast );

	ENSURE( GetUpAxis( bodyId ).y < -0.9f );

	Simulate( worldId, 50.0f, 2, 30.0f );

	ENSURE( GetUpAxis( bodyId ).y > 0.0f );
	ENSURE( GetTiltSine( bodyId ) < 1e-3f );
	ENSURE( IsFiniteBody( bodyId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// Sweeps the surface through a fixed capsule against a fine reference integral
static int CapsuleSweep( float tiltDegrees )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3AABB bounds = { { -5.0f, -5.0f, -5.0f }, { 5.0f, 1.0f, 5.0f } };
	b3FluidVolumeId volumeId = CreateBasin( worldId, bounds, 0.0f, b3Vec3_zero );

	// Zero gravity scale removes buoyancy and every lock holds the pose
	float radius = 0.1f;
	float halfLength = 0.3f;
	b3Quat rotation =
		b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, tiltDegrees * B3_PI / 180.0f );
	b3BodyId bodyId = CreateRod( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, rotation, b3Vec3_axisY,
								 halfLength, radius, WATER_DENSITY, false );
	b3Body_SetGravityScale( bodyId, 0.0f );
	b3MotionLocks locks = { true, true, true, true, true, true };
	b3Body_SetMotionLocks( bodyId, locks );

	b3Vec3 axis = b3RotateVector( b3Body_GetRotation( bodyId ), b3Vec3_axisY );
	double cosine = fabs( (double)axis.y );
	double sine = sqrt( (double)axis.x * axis.x + (double)axis.z * axis.z );
	double total = CapsuleVolume( radius, 2.0 * halfLength );
	double extent = halfLength * cosine + radius + 0.01;

	int sampleCount = 400;
	float previousFraction = 0.0f;
	double previousReference = 0.0;
	for ( int i = 0; i <= sampleCount; ++i )
	{
		float level = (float)( -extent + 2.0 * extent * i / sampleCount );
		b3FluidVolume_SetSurface( volumeId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, level } );
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		float fraction = b3Body_GetSubmergedFraction( bodyId );
		double reference = TiltedCapsuleVolume( cosine, sine, halfLength, radius, level ) / total;

		// The band quadrature is within 3.5e-5 of the volume against a double precision reference
		ENSURE_NEAR( fraction - reference, 5e-5 );

		if ( i > 0 )
		{
			// Monotone, and no jump above 1e-5 of the volume between depths 2 mm apart
			ENSURE( fraction >= previousFraction - 1e-6f );
			ENSURE_NEAR( ( fraction - previousFraction ) - ( reference - previousReference ),
						 1e-5 );
		}

		previousFraction = fraction;
		previousReference = reference;
	}

	ENSURE( previousFraction == 1.0f );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 1.0f );

	b3FluidVolume_SetSurface( volumeId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, (float)-extent } );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 0.0f );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyId ) ) );

	b3DestroyWorld( worldId );
	return 0;
}

static int CapsuleVolumeContinuity( void )
{
	ENSURE( CapsuleSweep( 0.0f ) == 0 );
	ENSURE( CapsuleSweep( 30.0f ) == 0 );
	ENSURE( CapsuleSweep( 60.0f ) == 0 );
	ENSURE( CapsuleSweep( 90.0f ) == 0 );
	return 0;
}

// A 5 cm box of specific gravity 0.5 bobbing without drag keeps its heave amplitude
static int HeaveWithoutDrag( float hertz, int subStepCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );
	b3FluidVolume_SetDrag( volumeId, 0.0f, 0.0f, 0.0f );

	// Locked rotation because a cube at this specific gravity is upright-unstable
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.01f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.025f, 0.025f, 0.025f }, 500.0f, false );
	LockRotation( bodyId );

	int stepCount = (int)( 10.0f * hertz );
	int windowCount = (int)hertz;
	float first = 0.0f;
	float last = 0.0f;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, subStepCount );
		float amplitude = fabsf( GetHeight( bodyId ) );
		if ( i < windowCount )
		{
			first = b3MaxFloat( first, amplitude );
		}

		if ( i >= stepCount - windowCount )
		{
			last = b3MaxFloat( last, amplitude );
		}
	}

	// Energy-neutral map; sampled peaks lie within 1 - cos(w * dt / 2) < 2% of the true peak
	ENSURE( last <= 1.03f * first );
	ENSURE( last >= 0.95f * first );

	b3DestroyWorld( worldId );
	return 0;
}

static int HeaveStability( void )
{
	ENSURE( HeaveWithoutDrag( 50.0f, 2 ) == 0 );
	ENSURE( HeaveWithoutDrag( 60.0f, 4 ) == 0 );
	return 0;
}

static int PlankRoll( float hertz, int subStepCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );
	b3FluidVolume_SetDrag( volumeId, 0.0f, 0.0f, 0.0f );

	b3Quat rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.1f );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, rotation,
								 (b3Vec3){ 0.4f, 0.02f, 0.15f }, 500.0f, false );
	float initial = GetTiltSine( bodyId );

	int stepCount = (int)( 10.0f * hertz );
	int windowCount = (int)hertz;
	float last = 0.0f;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, subStepCount );
		if ( i >= stepCount - windowCount )
		{
			last = b3MaxFloat( last, GetTiltSine( bodyId ) );
		}
	}

	// Energy caps the tilt of the softest mode; discrete energy wobbles by (w dt)^2 / 8 = 2.3%
	ENSURE( IsFiniteBody( bodyId ) );
	ENSURE( last <= 1.05f * initial );

	b3DestroyWorld( worldId );
	return 0;
}

// A plank rolled 0.1 rad about its long axis, under the 0.133 rad edge emergence, gains no roll
static int RollStability( void )
{
	ENSURE( PlankRoll( 50.0f, 2 ) == 0 );
	ENSURE( PlankRoll( 60.0f, 4 ) == 0 );
	return 0;
}

// Kinetic, gravitational and hydrostatic energy of a box, with default gravity and surface y = 0
static double BoxEnergy( b3BodyId bodyId, b3Vec3 halfExtents )
{
	double mass = b3Body_GetMass( bodyId );
	b3WorldTransform transform = b3Body_GetTransform( bodyId );
	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	b3Vec3 w = b3InvRotateVector( transform.q, b3Body_GetAngularVelocity( bodyId ) );
	b3Vec3 iw = b3MulMV( b3Body_GetLocalRotationalInertia( bodyId ), w );
	double energy = 0.5 * mass * b3Dot( v, v ) + 0.5 * b3Dot( w, iw ) + 10.0 * mass * transform.p.y;

	// The fluid's potential: rho * g times the integral of depth over the submerged part, on a grid
	int counts[3] = { 200, 8, 75 };
	double cell[3] = { 2.0 * halfExtents.x / counts[0], 2.0 * halfExtents.y / counts[1],
					   2.0 * halfExtents.z / counts[2] };
	b3Vec3 ex = b3RotateVector( transform.q, b3Vec3_axisX );
	b3Vec3 ey = b3RotateVector( transform.q, b3Vec3_axisY );
	b3Vec3 ez = b3RotateVector( transform.q, b3Vec3_axisZ );
	double depthSum = 0.0;
	for ( int i = 0; i < counts[0]; ++i )
	{
		double x = -halfExtents.x + ( i + 0.5 ) * cell[0];
		for ( int j = 0; j < counts[1]; ++j )
		{
			double y = -halfExtents.y + ( j + 0.5 ) * cell[1];
			double base = transform.p.y + x * ex.y + y * ey.y;
			for ( int k = 0; k < counts[2]; ++k )
			{
				double z = -halfExtents.z + ( k + 0.5 ) * cell[2];
				depthSum += fmax( -( base + z * ez.y ), 0.0 );
			}
		}
	}

	return energy + WATER_DENSITY * 10.0 * depthSum * cell[0] * cell[1] * cell[2];
}

// Without drag a thin plank pitched off its principal axes must not gain energy
static int PlankPitch( float hertz, int subStepCount, float isotropicInertiaTolerance )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.isotropicInertiaTolerance = isotropicInertiaTolerance;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );
	b3FluidVolume_SetDrag( volumeId, 0.0f, 0.0f, 0.0f );

	b3Vec3 halfExtents = { 0.4f, 0.02f, 0.15f };
	b3Quat pitch = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.15f );
	b3Quat offAxis = b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 0.3f, 0.0f } ), 0.05f );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, b3MulQuat( pitch, offAxis ),
								 halfExtents, 500.0f, false );
	double initial = BoxEnergy( bodyId, halfExtents );

	Simulate( worldId, hertz, subStepCount, 20.0f );

	// The lagging substep inertia used to pump it from 1.5 J to over 8 J in these 20 s
	double energy = BoxEnergy( bodyId, halfExtents );
	ENSURE( IsFiniteBody( bodyId ) );
	ENSURE( energy <= 1.2 * initial );

	b3DestroyWorld( worldId );
	return 0;
}

static int PitchStability( void )
{
	float tolerance = b3DefaultWorldDef().isotropicInertiaTolerance;
	ENSURE( PlankPitch( 50.0f, 2, tolerance ) == 0 );
	ENSURE( PlankPitch( 60.0f, 8, tolerance ) == 0 );

	// A tolerance of 1 turns the plank's gyroscopic torque off, which must not matter
	ENSURE( PlankPitch( 30.0f, 2, 1.0f ) == 0 );
	ENSURE( PlankPitch( 50.0f, 2, 1.0f ) == 0 );
	return 0;
}

// With the default drag the bobbing box and the rolling plank both fall asleep
static int DefaultDragSleeps( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );

	b3BodyId boxId = CreateBox( worldId, (b3Pos){ -1.0f, 0.01f, 0.0f }, b3Quat_identity,
								(b3Vec3){ 0.025f, 0.025f, 0.025f }, 500.0f, true );
	LockRotation( boxId );

	b3Quat rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.1f );
	b3BodyId plankId = CreateBox( worldId, (b3Pos){ 1.0f, 0.0f, 0.0f }, rotation,
								  (b3Vec3){ 0.4f, 0.02f, 0.15f }, 500.0f, true );

	float hertz = 50.0f;
	int stepCount = (int)( 20.0f * hertz );
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, 2 );
		if ( b3World_GetAwakeBodyCount( worldId ) == 0 )
		{
			break;
		}
	}

	ENSURE( b3Body_IsAwake( boxId ) == false );
	ENSURE( b3Body_IsAwake( plankId ) == false );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( boxId ), volumeId ) );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( plankId ), volumeId ) );

	// Asleep under 0.05 m/s means up to 0.05 / w off equilibrium: 2.5 mm box, 2.3 mm plank
	ENSURE_NEAR( GetHeight( boxId ), 2.5e-3f );
	ENSURE_NEAR( GetHeight( plankId ), 2.3e-3f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( boxId ) - 0.5f, 0.05f );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( plankId ) - 0.5f, 0.06f );

	b3DestroyWorld( worldId );
	return 0;
}

// A dense box sinks to the floor and sleeps there, fully submerged
static int Sinking( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 3.0f );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -3.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );

	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.3f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.1f, 0.1f, 0.1f }, 2500.0f, true );

	float minVelocity = 0.0f;
	for ( int i = 0; i < 600 && b3Body_IsAwake( bodyId ); ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		minVelocity = b3MinFloat( minVelocity, b3Body_GetLinearVelocity( bodyId ).y );
	}

	// The drag holds it well below the 7.7 m/s of a free fall to the floor
	ENSURE( minVelocity < -0.5f && minVelocity > -5.0f );

	// Resting contact sits within the linear slop of the floor
	ENSURE( b3Body_IsAwake( bodyId ) == false );
	ENSURE_NEAR( GetHeight( bodyId ) + 2.9f, 0.01f );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 1.0f );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyId ), volumeId ) );
	ENSURE( b3Body_GetLinearDamping( bodyId ) == 0.0f );
	ENSURE( b3Body_GetAngularDamping( bodyId ) == 0.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// A neutral sphere is carried at the current's speed with the exact backward Euler transient
static int CurrentSpeed( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3Vec3 flow = { 0.8f, 0.0f, -0.4f };
	b3AABB bounds = { { -50.0f, -5.0f, -50.0f }, { 50.0f, 0.0f, 50.0f } };
	b3FluidVolumeId volumeId = CreateBasin( worldId, bounds, 0.0f, flow );
	b3FluidVolume_SetDrag( volumeId, 1.5f, 1.5f, 0.0f );

	b3BodyId bodyId =
		CreateBall( worldId, (b3Pos){ 0.0f, -2.0f, 0.0f }, 0.2f, WATER_DENSITY, false );

	// Fully submerged, so the rate is the linear drag: v - u shrinks by 1 / (1 + h * k) per substep
	float hertz = 50.0f;
	int subStepCount = 2;
	int stepCount = 25;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, subStepCount );
	}

	double h = 1.0 / ( hertz * subStepCount );
	float decay = (float)pow( 1.0 + 1.5 * h, -(double)( subStepCount * stepCount ) );
	b3Vec3 expected = b3MulSV( 1.0f - decay, flow );
	b3Vec3 velocity = b3Body_GetLinearVelocity( bodyId );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 1.0f );
	ENSURE_NEAR( velocity.x - expected.x, 1e-5f );
	ENSURE_NEAR( velocity.y, 1e-5f );
	ENSURE_NEAR( velocity.z - expected.z, 1e-5f );

	// Quadratic drag converges to the same current
	b3FluidVolume_SetDrag( volumeId, 1.5f, 1.5f, 1.0f );
	Simulate( worldId, hertz, subStepCount, 10.0f );

	velocity = b3Body_GetLinearVelocity( bodyId );
	ENSURE_NEAR( velocity.x - flow.x, 1e-4f );
	ENSURE_NEAR( velocity.y, 1e-4f );
	ENSURE_NEAR( velocity.z - flow.z, 1e-4f );
	ENSURE_NEAR( GetHeight( bodyId ) + 2.0f, 1e-3f );

	b3DestroyWorld( worldId );
	return 0;
}

// Terminal speed solving (d + s * (linear + q * v)) * v = a; body damping d is not drag scaled
static float GetTerminalSpeed( float acceleration, float linearDrag, float quadraticRate,
							   float dragScale, float bodyDamping )
{
	float b = bodyDamping + dragScale * linearDrag;
	float c = dragScale * quadraticRate;
	return 2.0f * acceleration / ( b + sqrtf( b * b + 4.0f * c * acceleration ) );
}

static int TerminalSpeed( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 10.0f, 300.0f );

	// Specific gravity 2 sinks at a = g / 2 per gravity scale. A = V^(2/3) = 0.04 m^2, m = 16 kg
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, -1.0f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.1f, 0.1f, 0.1f }, 2000.0f, false );
	float acceleration = 5.0f;
	float quadraticRate = 0.5f * WATER_DENSITY * 0.04f / 16.0f;

	// The last three reach the quadratic substep prediction through gravity scale and body damping
	struct
	{
		float linearDrag;
		float quadraticDrag;
		float dragScale;
		float gravityScale;
		float linearDamping;
		int subStepCount;
	} cases[] = {
		{ 1.5f, 0.0f, 1.0f, 1.0f, 0.0f, 2 }, { 1.5f, 0.0f, 2.0f, 1.0f, 0.0f, 2 },
		{ 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 2 }, { 1.5f, 1.0f, 1.0f, 1.0f, 0.0f, 2 },
		{ 0.0f, 1.0f, 1.0f, 2.0f, 0.0f, 2 }, { 0.0f, 1.0f, 1.0f, 1.0f, 0.5f, 2 },
		{ 1.5f, 1.0f, 2.0f, 1.0f, 0.5f, 4 },
	};

	for ( int i = 0; i < ARRAY_COUNT( cases ); ++i )
	{
		b3FluidVolume_SetDrag( volumeId, cases[i].linearDrag, 1.5f, cases[i].quadraticDrag );
		b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
		fluidDef.dragScale = cases[i].dragScale;
		b3Body_SetFluidDef( bodyId, &fluidDef );
		b3Body_SetGravityScale( bodyId, cases[i].gravityScale );
		b3Body_SetLinearDamping( bodyId, cases[i].linearDamping );

		Simulate( worldId, 50.0f, cases[i].subStepCount, 10.0f );

		float expected = GetTerminalSpeed(
			cases[i].gravityScale * acceleration, cases[i].linearDrag,
			cases[i].quadraticDrag * quadraticRate, cases[i].dragScale, cases[i].linearDamping );
		float speed = -b3Body_GetLinearVelocity( bodyId ).y;

		// Exact fixed point; rounding of 1 / (1 + h * k) is amplified by 1 / (h * k)
		ENSURE_NEAR( speed / expected - 1.0f, 1e-4f );
		ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 1.0f );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// Pure quadratic drag takes the speed relative to the current from v0 to v0 / (1 + q * v0 * t)
static int QuadraticDecayRun( int subStepCount, b3Vec3 flow )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3AABB bounds = { { -10.0f, -10.0f, -10.0f }, { 10.0f, 0.0f, 10.0f } };
	b3FluidVolumeId volumeId = CreateBasin( worldId, bounds, 0.0f, flow );
	b3FluidVolume_SetDrag( volumeId, 0.0f, 0.0f, 1.0f );

	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, -2.0f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.025f, 0.025f, 0.025f }, 500.0f, false );
	b3Vec3 launch = { 6.0f, 0.0f, 8.0f };
	b3Body_SetLinearVelocity( bodyId, b3Add( flow, launch ) );

	// A current leaves the relative velocity as the small difference of two larger ones
	bool still = b3Length( flow ) == 0.0f;
	double speedTolerance = still ? 1e-5 : 1e-4;
	double directionTolerance = still ? 1e-6 : 1e-4;

	// q = 0.5 * rho * Cd * A / m with A = V^(2/3), so q * v0 * dt = 4 in the first step
	double initialSpeed = b3Length( launch );
	double q = 0.5 * WATER_DENSITY * 0.05 * 0.05 / b3Body_GetMass( bodyId );
	float hertz = 50.0f;
	for ( int i = 1; i <= 25; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, subStepCount );

		double expected = initialSpeed / ( 1.0 + q * initialSpeed * i / hertz );
		b3Vec3 relative = b3Sub( b3Body_GetLinearVelocity( bodyId ), flow );
		double speed = b3Length( relative );
		ENSURE_NEAR( speed / expected - 1.0, speedTolerance );
		ENSURE_NEAR( b3Length( b3Cross( relative, launch ) ) / ( speed * initialSpeed ),
					 directionTolerance );
		ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 1.0f );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// The substeps hold one rate, chosen so the decay is exact at the step boundary for any count
static int QuadraticDecay( void )
{
	b3Vec3 flow = { 1.0f, 0.0f, -2.0f };
	int subStepCounts[4] = { 1, 2, 4, 8 };
	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( QuadraticDecayRun( subStepCounts[i], b3Vec3_zero ) == 0 );
		ENSURE( QuadraticDecayRun( subStepCounts[i], flow ) == 0 );
	}

	return 0;
}

// Angular drag decays spin by 1 / (1 + h * ka) per substep; drag scale zero removes it
static int AngularDrag( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreatePool( worldId, 10.0f, 10.0f );

	b3Vec3 spin = { 0.0f, 4.0f, 3.0f };
	b3BodyId bodyId =
		CreateBall( worldId, (b3Pos){ 0.0f, -2.0f, 0.0f }, 0.2f, WATER_DENSITY, false );
	b3Body_SetAngularVelocity( bodyId, spin );

	b3BodyId freeId =
		CreateBall( worldId, (b3Pos){ 2.0f, -2.0f, 0.0f }, 0.2f, WATER_DENSITY, false );
	b3Body_SetAngularVelocity( freeId, spin );
	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.dragScale = 0.0f;
	b3Body_SetFluidDef( freeId, &fluidDef );

	float hertz = 60.0f;
	int subStepCount = 2;
	int stepCount = 60;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, 1.0f / hertz, subStepCount );
	}

	double h = 1.0 / ( hertz * subStepCount );
	float decay = (float)pow( 1.0 + 1.5 * h, -(double)( subStepCount * stepCount ) );
	b3Vec3 angularVelocity = b3Body_GetAngularVelocity( bodyId );
	ENSURE_NEAR( b3Length( b3Sub( angularVelocity, b3MulSV( decay, spin ) ) ), 1e-4f );

	angularVelocity = b3Body_GetAngularVelocity( freeId );
	ENSURE_NEAR( b3Length( b3Sub( angularVelocity, spin ) ), 1e-5f );
	ENSURE( b3Body_GetSubmergedFraction( freeId ) == 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// Ineligible bodies inside a moving volume get no fluid write and no events
static int IneligibleBodies( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = (b3AABB){ { -10.0f, -10.0f, -10.0f }, { 10.0f, 0.0f, 10.0f } };
	volumeDef.flowVelocity = (b3Vec3){ 2.0f, 0.0f, 0.5f };
	volumeDef.maskBits = 0x2;
	b3FluidVolumeId volumeId = b3CreateFluidVolume( worldId, &volumeDef );

	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 500.0f;
	shapeDef.filter.categoryBits = 0x2;

	// Every shape fails the mask, with a twin outside the volume
	b3BodyId maskedId =
		CreateDynamicBody( worldId, (b3Pos){ -4.0f, -1.0f, 0.0f }, b3Quat_identity, false );
	b3BodyId twinId =
		CreateDynamicBody( worldId, (b3Pos){ 100.0f, -1.0f, 0.0f }, b3Quat_identity, false );
	b3ShapeDef maskedDef = shapeDef;
	maskedDef.filter.categoryBits = 0x1;
	b3CreateSphereShape( maskedId, &maskedDef, &sphere );
	b3CreateSphereShape( twinId, &maskedDef, &sphere );

	b3BodyId sensorId =
		CreateDynamicBody( worldId, (b3Pos){ -2.0f, -1.0f, 0.0f }, b3Quat_identity, false );
	b3ShapeDef sensorDef = shapeDef;
	sensorDef.isSensor = true;
	b3CreateSphereShape( sensorId, &sensorDef, &sphere );

	// Shapeless, with a displaced volume that cannot make it eligible
	b3BodyId shapelessId =
		CreateDynamicBody( worldId, (b3Pos){ 0.0f, -1.0f, 0.0f }, b3Quat_identity, false );
	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.displacedVolume = 1.0f;
	b3Body_SetFluidDef( shapelessId, &fluidDef );

	b3BodyId massId =
		CreateDynamicBody( worldId, (b3Pos){ 2.0f, -1.0f, 0.0f }, b3Quat_identity, false );
	b3ShapeDef zeroDef = shapeDef;
	zeroDef.density = 0.0f;
	b3CreateSphereShape( massId, &zeroDef, &sphere );
	b3Body_SetFluidDef( massId, &fluidDef );

	b3BodyId controlId =
		CreateDynamicBody( worldId, (b3Pos){ 4.0f, 0.0f, 0.0f }, b3Quat_identity, false );
	b3CreateSphereShape( controlId, &shapeDef, &sphere );

	ENSURE( b3Body_GetMass( massId ) == 0.0f );

	b3BodyId ineligibleIds[4] = { maskedId, sensorId, shapelessId, massId };
	int beginCount = 0;
	int endCount = 0;
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		for ( int j = 0; j < events.beginCount; ++j )
		{
			ENSURE( B3_ID_EQUALS( events.beginEvents[j].bodyId, controlId ) );
		}

		beginCount += events.beginCount;
		endCount += events.endCount;
	}

	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( IsFiniteBody( ineligibleIds[i] ) );
		ENSURE( b3Body_GetSubmergedFraction( ineligibleIds[i] ) == 0.0f );
		ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( ineligibleIds[i] ) ) );
	}

	// The masked body falls exactly like its dry twin
	b3Vec3 maskedVelocity = b3Body_GetLinearVelocity( maskedId );
	b3Vec3 twinVelocity = b3Body_GetLinearVelocity( twinId );
	ENSURE( memcmp( &maskedVelocity, &twinVelocity, sizeof( b3Vec3 ) ) == 0 );
	ENSURE( b3Body_GetPosition( maskedId ).y == b3Body_GetPosition( twinId ).y );

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( controlId ), volumeId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// A cube with one corner 1e-6 below the surface, and one with its corner 1e-6 above
static int HullCornerGuard( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreatePool( worldId, 5.0f, 5.0f );

	// Turn the (-1, -1, -1) corner straight down
	float half = 0.25f;
	b3Vec3 corner = { -half, -half, -half };
	b3Vec3 axis = b3Normalize( (b3Vec3){ -1.0f, 0.0f, 1.0f } );
	b3Quat rotation = b3MakeQuatFromAxisAngle( axis, (float)acos( 1.0 / sqrt( 3.0 ) ) );

	float offsets[2] = { -1e-6f, 1e-6f };
	b3BodyId bodyIds[2];
	for ( int i = 0; i < 2; ++i )
	{
		bodyIds[i] = CreateBox( worldId, (b3Pos){ 2.0f * i, 1.0f, 0.0f }, rotation,
								(b3Vec3){ half, half, half }, 500.0f, false );
		float cornerHeight = (float)b3Body_GetWorldPoint( bodyIds[i], corner ).y;
		b3Pos position = b3Body_GetPosition( bodyIds[i] );
		position.y += offsets[i] - cornerHeight;
		b3Body_SetTransform( bodyIds[i], position, rotation );
	}

	float lowest = (float)b3Body_GetWorldPoint( bodyIds[0], corner ).y;
	ENSURE( lowest < 0.0f && lowest > -2e-6f );

	b3World_Step( worldId, 1.0f / 50.0f, 2 );

	// The corner tetrahedron is about 1e-18 m^3: the fraction is tiny or zero, events agree
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	float fraction = b3Body_GetSubmergedFraction( bodyIds[0] );
	bool inFluid = B3_IS_NON_NULL( b3Body_GetFluidVolume( bodyIds[0] ) );
	ENSURE( fraction >= 0.0f && fraction < 1e-9f );
	ENSURE( inFluid == ( fraction > 0.0f ) );
	ENSURE( events.beginCount == ( inFluid ? 1 : 0 ) );
	ENSURE( events.endCount == 0 );
	ENSURE( IsFiniteBody( bodyIds[0] ) );
	ENSURE( b3Length( b3Body_GetAngularVelocity( bodyIds[0] ) ) < 1e-3f );

	ENSURE( b3Body_GetSubmergedFraction( bodyIds[1] ) == 0.0f );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyIds[1] ) ) );

	for ( int i = 0; i < 100; ++i )
	{
		b3World_Step( worldId, 1.0f / 50.0f, 2 );
	}

	for ( int i = 0; i < 2; ++i )
	{
		ENSURE( IsFiniteBody( bodyIds[i] ) );
		float f = b3Body_GetSubmergedFraction( bodyIds[i] );
		ENSURE( 0.0f < f && f <= 1.0f );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// Two spheres on one body, the upper one fully above the surface
static int TwoSpheresOneDry( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreatePool( worldId, 5.0f, 5.0f );

	b3BodyId bodyId =
		CreateDynamicBody( worldId, (b3Pos){ 1.0f, 0.0f, -1.0f }, b3Quat_identity, false );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 500.0f;
	b3Sphere lower = { { 0.0f, -0.1f, 0.0f }, 0.2f };
	b3Sphere upper = { { 0.0f, 0.5f, 0.0f }, 0.2f };
	b3CreateSphereShape( bodyId, &shapeDef, &lower );
	b3CreateSphereShape( bodyId, &shapeDef, &upper );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	// A cap of height 0.3 on the lower sphere over both sphere volumes
	double expected = SphereCapVolume( 0.2, 0.3 ) / ( 2.0 * SphereVolume( 0.2 ) );
	ENSURE_NEAR( b3Body_GetSubmergedFraction( bodyId ) - (float)expected, 1e-5f );

	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 1 );
	ENSURE_NEAR( (float)events.beginEvents[0].point.x - 1.0f, 1e-5f );
	ENSURE_NEAR( (float)events.beginEvents[0].point.y, 1e-5f );
	ENSURE_NEAR( (float)events.beginEvents[0].point.z + 1.0f, 1e-5f );
	ENSURE( b3Length( b3Body_GetAngularVelocity( bodyId ) ) < 1e-5f );

	Simulate( worldId, 60.0f, 4, 10.0f );

	float fraction = b3Body_GetSubmergedFraction( bodyId );
	ENSURE( IsFiniteBody( bodyId ) );
	ENSURE( 0.0f < fraction && fraction <= 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

#define LEDGER_CAPACITY 16

// Tracks the volume of each body from the event stream alone
typedef struct EventLedger
{
	b3BodyId bodyIds[LEDGER_CAPACITY];
	b3FluidVolumeId volumeIds[LEDGER_CAPACITY];
	int count;
	int beginCount;
	int endCount;
	int stepBeginCount;
	int stepEndCount;
} EventLedger;

static void AddLedgerBody( EventLedger* ledger, b3BodyId bodyId )
{
	ledger->bodyIds[ledger->count] = bodyId;
	ledger->volumeIds[ledger->count] = b3_nullFluidVolumeId;
	ledger->count += 1;
}

static int FindLedgerIndex( const EventLedger* ledger, b3BodyId bodyId )
{
	for ( int i = 0; i < ledger->count; ++i )
	{
		if ( B3_ID_EQUALS( ledger->bodyIds[i], bodyId ) )
		{
			return i;
		}
	}

	return -1;
}

// Applies a step's events, ends before begins, and checks them against b3Body_GetFluidVolume
static int ApplyFluidEvents( EventLedger* ledger, b3WorldId worldId )
{
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	for ( int i = 0; i < events.endCount; ++i )
	{
		b3FluidEndTouchEvent event = events.endEvents[i];
		int index = FindLedgerIndex( ledger, event.bodyId );
		ENSURE( index >= 0 );
		ENSURE( B3_ID_EQUALS( ledger->volumeIds[index], event.volumeId ) );
		ledger->volumeIds[index] = b3_nullFluidVolumeId;
	}

	for ( int i = 0; i < events.beginCount; ++i )
	{
		b3FluidBeginTouchEvent event = events.beginEvents[i];
		int index = FindLedgerIndex( ledger, event.bodyId );
		ENSURE( index >= 0 );
		ENSURE( B3_IS_NULL( ledger->volumeIds[index] ) );
		ENSURE( b3FluidVolume_IsValid( event.volumeId ) );
		ledger->volumeIds[index] = event.volumeId;
	}

	ledger->stepBeginCount = events.beginCount;
	ledger->stepEndCount = events.endCount;
	ledger->beginCount += events.beginCount;
	ledger->endCount += events.endCount;

	for ( int i = 0; i < ledger->count; ++i )
	{
		if ( b3Body_IsValid( ledger->bodyIds[i] ) )
		{
			ENSURE(
				B3_ID_EQUALS( ledger->volumeIds[i], b3Body_GetFluidVolume( ledger->bodyIds[i] ) ) );
		}
		else
		{
			ENSURE( B3_IS_NULL( ledger->volumeIds[i] ) );
		}
	}

	return 0;
}

static int StepLedger( b3WorldId worldId, EventLedger* ledger, float timeStep, int subStepCount )
{
	b3World_Step( worldId, timeStep, subStepCount );
	return ApplyFluidEvents( ledger, worldId );
}

static int EnterExitEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 1.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { b3Vec3_zero, 0.25f };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	SetSpecificGravity( bodyId, 0.5f );

	float timeStep = 1.0f / 60.0f;
	int beginCount = 0;
	int endCount = 0;
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		endCount += events.endCount;
		if ( events.beginCount == 0 )
		{
			continue;
		}

		beginCount += events.beginCount;
		b3FluidBeginTouchEvent event = events.beginEvents[0];
		ENSURE( B3_ID_EQUALS( event.bodyId, bodyId ) );
		ENSURE( B3_ID_EQUALS( event.volumeId, volumeId ) );
		ENSURE_NEAR( (float)event.point.y, 0.001f );
		ENSURE( event.velocity.y < 0.0f );

		// A dt == 0 step must not repeat the begin event
		b3World_Step( worldId, 0.0f, 4 );
		ENSURE( b3World_GetFluidEvents( worldId ).beginCount == 0 );
	}

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyId ), volumeId ) );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) > 0.0f );

	// Lift out of the water
	b3Body_SetTransform( bodyId, (b3Pos){ 0.0f, 3.0f, 0.0f }, b3Quat_identity );
	b3Body_SetAwake( bodyId, true );
	b3World_Step( worldId, timeStep, 4 );
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 0 );
	ENSURE( events.endCount == 1 );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].bodyId, bodyId ) );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].volumeId, volumeId ) );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyId ) ) );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 0.0f );

	// Back in, then destroyed
	b3Body_SetTransform( bodyId, (b3Pos){ 0.0f, -1.0f, 0.0f }, b3Quat_identity );
	b3World_Step( worldId, timeStep, 4 );
	events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 1 );
	ENSURE( events.endCount == 0 );

	b3DestroyBody( bodyId );
	b3World_Step( worldId, timeStep, 4 );
	events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 0 );
	ENSURE( events.endCount == 1 );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].bodyId, bodyId ) );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].volumeId, volumeId ) );

	b3World_Step( worldId, timeStep, 4 );
	events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 0 );
	ENSURE( events.endCount == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// A current carries a floater across two abutting volumes at one level and then out of the second
static int VolumeSwitchEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3Vec3 flow = { 1.0f, 0.0f, 0.0f };
	b3FluidVolumeId firstId = CreateBasin(
		worldId, (b3AABB){ { -4.0f, -2.0f, -2.0f }, { 0.0f, 0.0f, 2.0f } }, 0.0f, flow );
	b3FluidVolumeId secondId = CreateBasin(
		worldId, (b3AABB){ { 0.0f, -2.0f, -2.0f }, { 2.0f, 0.0f, 2.0f } }, 0.0f, flow );

	b3BodyId bodyId = CreateBall( worldId, (b3Pos){ -1.0f, 0.0f, 0.0f }, 0.15f, 500.0f, false );

	EventLedger ledger = { 0 };
	AddLedgerBody( &ledger, bodyId );

	int switchStep = -1;
	int exitStep = -1;
	for ( int i = 0; i < 360; ++i )
	{
		ENSURE( StepLedger( worldId, &ledger, 1.0f / 60.0f, 4 ) == 0 );
		b3FluidEvents events = b3World_GetFluidEvents( worldId );

		if ( i == 0 )
		{
			ENSURE( events.beginCount == 1 && events.endCount == 0 );
			ENSURE( B3_ID_EQUALS( events.beginEvents[0].volumeId, firstId ) );
		}
		else if ( events.endCount > 0 && B3_ID_EQUALS( events.endEvents[0].volumeId, firstId ) )
		{
			// The switch ends the first volume and begins the second in the same step
			ENSURE( switchStep == -1 );
			ENSURE( events.beginCount == 1 && events.endCount == 1 );
			ENSURE( B3_ID_EQUALS( events.beginEvents[0].volumeId, secondId ) );
			ENSURE( (float)b3Body_GetPosition( bodyId ).x > -0.1f );
			switchStep = i;
		}
		else if ( events.endCount > 0 )
		{
			ENSURE( exitStep == -1 );
			ENSURE( events.beginCount == 0 && events.endCount == 1 );
			ENSURE( B3_ID_EQUALS( events.endEvents[0].volumeId, secondId ) );
			exitStep = i;
		}
		else
		{
			ENSURE( events.beginCount == 0 );
		}
	}

	ENSURE( switchStep > 0 && exitStep > switchStep );
	ENSURE( ledger.beginCount == 2 && ledger.endCount == 2 );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyId ) ) );

	b3DestroyWorld( worldId );
	return 0;
}

#define LIFECYCLE_GROUP_SIZE 6

// Destroy, disable, kinematic, static and enabled = false on awake and sleeping floaters
static int LifecycleEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 8.0f, 2.0f );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 8.0f, 0.5f, 8.0f } );

	// Flat floaters are upright-stable. The first group never sleeps.
	EventLedger ledger = { 0 };
	b3BodyId bodyIds[2 * LIFECYCLE_GROUP_SIZE];
	for ( int i = 0; i < 2 * LIFECYCLE_GROUP_SIZE; ++i )
	{
		bool enableSleep = i >= LIFECYCLE_GROUP_SIZE;
		b3Pos position = { -5.5f + i, 0.0f, 0.0f };
		bodyIds[i] = CreateBox( worldId, position, b3Quat_identity, (b3Vec3){ 0.15f, 0.05f, 0.15f },
								500.0f, enableSleep );
		AddLedgerBody( &ledger, bodyIds[i] );
	}

	float timeStep = 1.0f / 60.0f;
	ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
	ENSURE( ledger.stepBeginCount == 2 * LIFECYCLE_GROUP_SIZE );

	for ( int i = 0; i < 300; ++i )
	{
		ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
	}

	ENSURE( ledger.beginCount == 2 * LIFECYCLE_GROUP_SIZE && ledger.endCount == 0 );

	b3BodyFluidDef disabledDef = b3DefaultBodyFluidDef();
	disabledDef.enabled = false;

	for ( int group = 0; group < 2; ++group )
	{
		b3BodyId* ids = bodyIds + group * LIFECYCLE_GROUP_SIZE;
		bool sleeping = group == 1;
		for ( int i = 0; i < LIFECYCLE_GROUP_SIZE; ++i )
		{
			ENSURE( b3Body_IsAwake( ids[i] ) != sleeping );
		}

		b3DestroyBody( ids[0] );

		b3Body_Disable( ids[1] );
		b3Body_SetType( ids[2], b3_kinematicBody );
		b3Body_SetType( ids[3], b3_staticBody );
		b3Body_SetFluidDef( ids[4], &disabledDef );

		for ( int i = 1; i < 5; ++i )
		{
			ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( ids[i] ) ) );
			ENSURE( b3Body_GetSubmergedFraction( ids[i] ) == 0.0f );
		}

		// Losing buoyancy wakes a sleeping floater
		ENSURE( b3Body_IsAwake( ids[4] ) );
		ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( ids[5] ), volumeId ) );
	}

	ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
	ENSURE( ledger.stepEndCount == 10 && ledger.stepBeginCount == 0 );

	ENSURE( StepLedger( worldId, &ledger, 0.0f, 4 ) == 0 );
	ENSURE( ledger.stepEndCount == 0 && ledger.stepBeginCount == 0 );

	for ( int i = 0; i < 30; ++i )
	{
		ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
		ENSURE( ledger.stepEndCount == 0 && ledger.stepBeginCount == 0 );
	}

	// Restore: each comes back with one begin
	b3BodyFluidDef enabledDef = b3DefaultBodyFluidDef();
	for ( int group = 0; group < 2; ++group )
	{
		b3BodyId* ids = bodyIds + group * LIFECYCLE_GROUP_SIZE;
		b3Body_Enable( ids[1] );
		b3Body_SetType( ids[2], b3_dynamicBody );
		b3Body_SetType( ids[3], b3_dynamicBody );
		b3Body_SetFluidDef( ids[4], &enabledDef );
		ENSURE( b3Body_IsAwake( ids[4] ) );
	}

	ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
	ENSURE( ledger.stepBeginCount == 8 && ledger.stepEndCount == 0 );

	// Destroying the volume right after a step with begins ends every member and repeats no begin
	b3DestroyFluidVolume( volumeId );
	ENSURE( b3FluidVolume_IsValid( volumeId ) == false );
	for ( int i = 0; i < 2 * LIFECYCLE_GROUP_SIZE; ++i )
	{
		if ( b3Body_IsValid( bodyIds[i] ) )
		{
			ENSURE( b3Body_IsAwake( bodyIds[i] ) );
			ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyIds[i] ) ) );
		}
	}

	ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
	ENSURE( ledger.stepBeginCount == 0 && ledger.stepEndCount == 10 );

	ENSURE( StepLedger( worldId, &ledger, timeStep, 4 ) == 0 );
	ENSURE( ledger.stepBeginCount == 0 && ledger.stepEndCount == 0 );

	ENSURE( StepLedger( worldId, &ledger, 0.0f, 4 ) == 0 );
	ENSURE( ledger.stepBeginCount == 0 && ledger.stepEndCount == 0 );

	ENSURE( ledger.beginCount == ledger.endCount );
	ENSURE( b3World_GetFluidStats( worldId ).volumeCount == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// No stale begins after a dt == 0 step or after the last volume is destroyed
static int NoRepeatedBegins( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 5.0f );

	b3BodyId firstId = CreateBall( worldId, (b3Pos){ -1.0f, 0.0f, 0.0f }, 0.2f, 500.0f, true );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3World_GetFluidEvents( worldId ).beginCount == 1 );

	b3World_Step( worldId, 0.0f, 4 );
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 0 && events.endCount == 0 );

	b3BodyId secondId = CreateBall( worldId, (b3Pos){ 1.0f, 0.0f, 0.0f }, 0.2f, 500.0f, true );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 1 && B3_ID_EQUALS( events.beginEvents[0].bodyId, secondId ) );

	b3DestroyFluidVolume( volumeId );

	// Ends in body id order, delivered once
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 0 && events.endCount == 2 );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].bodyId, firstId ) );
	ENSURE( B3_ID_EQUALS( events.endEvents[1].bodyId, secondId ) );

	for ( int i = 0; i < 3; ++i )
	{
		b3World_Step( worldId, i == 1 ? 0.0f : 1.0f / 60.0f, 4 );
		events = b3World_GetFluidEvents( worldId );
		ENSURE( events.beginCount == 0 && events.endCount == 0 );
	}

	b3FluidStats stats = b3World_GetFluidStats( worldId );
	ENSURE( stats.volumeCount == 0 && stats.bodyCount == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// Begin velocity is the pre-step velocity, before drag, and the point is on the surface
static int BeginPayload( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Dense fluid with heavy drag
	b3FluidVolumeId volumeId = CreatePool( worldId, 10.0f, 10.0f );
	b3FluidVolume_SetDensity( volumeId, 3000.0f );
	b3FluidVolume_SetDrag( volumeId, 40.0f, 40.0f, 1.0f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.5f, 0.6f, -0.25f };
	bodyDef.linearVelocity = (b3Vec3){ 0.7f, -1.0f, -0.3f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 500.0f;
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	float hertz = 50.0f;
	int subStepCount = 2;
	float lead = 0.25f / hertz;
	bool entered = false;
	for ( int i = 0; i < 100 && entered == false; ++i )
	{
		b3Vec3 velocity = b3Body_GetLinearVelocity( bodyId );
		b3Pos position = b3Body_GetPosition( bodyId );

		b3World_Step( worldId, 1.0f / hertz, subStepCount );
		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		if ( events.beginCount == 0 )
		{
			continue;
		}

		entered = true;
		ENSURE( events.beginCount == 1 && i > 0 );
		b3FluidBeginTouchEvent event = events.beginEvents[0];
		ENSURE( memcmp( &event.velocity, &velocity, sizeof( b3Vec3 ) ) == 0 );

		// The sphere's centroid lies under its predicted center, center + dt * v / 4 at 2 substeps
		ENSURE_NEAR( (float)event.point.y, 1e-5f );
		ENSURE_NEAR( (float)( event.point.x - position.x ) - lead * velocity.x, 1e-5f );
		ENSURE_NEAR( (float)( event.point.z - position.z ) - lead * velocity.z, 1e-5f );
	}

	ENSURE( entered );

	// Entering fully submerged through the side of the bounds, so drag acts in the entry step
	b3BodyId sideId = CreateBall( worldId, (b3Pos){ -10.4f, -2.0f, 1.0f }, 0.2f, 500.0f, false );
	b3Body_SetLinearVelocity( sideId, (b3Vec3){ 2.0f, 0.0f, 0.5f } );
	entered = false;
	for ( int i = 0; i < 100 && entered == false; ++i )
	{
		b3Vec3 velocity = b3Body_GetLinearVelocity( sideId );
		b3Pos position = b3Body_GetPosition( sideId );

		b3World_Step( worldId, 1.0f / hertz, subStepCount );
		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		if ( events.beginCount == 0 )
		{
			continue;
		}

		entered = true;
		ENSURE( events.beginCount == 1 && i > 0 );
		b3FluidBeginTouchEvent event = events.beginEvents[0];
		ENSURE( B3_ID_EQUALS( event.bodyId, sideId ) );
		ENSURE( b3Body_GetSubmergedFraction( sideId ) == 1.0f );
		ENSURE( memcmp( &event.velocity, &velocity, sizeof( b3Vec3 ) ) == 0 );

		// The drag rate of 40/s takes about half the horizontal speed in one 50 Hz step
		b3Vec3 after = b3Body_GetLinearVelocity( sideId );
		ENSURE( after.x < 0.6f * velocity.x );

		ENSURE_NEAR( (float)event.point.y, 1e-5f );
		ENSURE_NEAR( (float)( event.point.x - position.x ) - lead * velocity.x, 1e-5f );
		ENSURE_NEAR( (float)( event.point.z - position.z ) - lead * velocity.z, 1e-5f );
	}

	ENSURE( entered );

	b3DestroyWorld( worldId );
	return 0;
}

// A def set before any volume exists takes effect when one is created
static int DefsBeforeFirstVolume( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();

	// Defs without a volume leave the simulation bit-identical
	b3WorldTransform transforms[2];
	for ( int pass = 0; pass < 2; ++pass )
	{
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.5f, 0.0f }, b3Quat_identity,
									 (b3Vec3){ 0.25f, 0.25f, 0.25f }, WATER_DENSITY, false );
		b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 1.0f, 2.0f, 3.0f } );
		if ( pass == 1 )
		{
			SetSpecificGravity( bodyId, 0.4f );
			b3FluidEvents events = b3World_GetFluidEvents( worldId );
			ENSURE( events.beginCount == 0 && events.endCount == 0 );
		}

		Simulate( worldId, 60.0f, 4, 1.0f );
		transforms[pass] = b3Body_GetTransform( bodyId );
		b3DestroyWorld( worldId );
	}

	ENSURE( memcmp( transforms + 0, transforms + 1, sizeof( b3WorldTransform ) ) == 0 );

	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.5f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.25f, 0.25f, 0.25f }, WATER_DENSITY, false );
	LockRotation( bodyId );

	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.displacedVolume = 0.125f / 0.4f;
	b3Body_SetFluidDef( bodyId, &fluidDef );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( bodyId ), fluidDef ) );
	ENSURE( b3World_GetFluidStats( worldId ).volumeCount == 0 );

	CreateDampedPool( worldId, 5.0f, 5.0f );
	Simulate( worldId, 50.0f, 2, 30.0f );

	// Specific gravity 0.4 puts the center 0.05 above the surface
	ENSURE_NEAR( GetHeight( bodyId ) - 0.05f, 1e-4f );

	b3DestroyWorld( worldId );
	return 0;
}

// Defs survive destroying the last volume, and volume ids never repeat
static int DefsSurviveVolumeDestroy( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.1f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.25f, 0.25f, 0.25f }, WATER_DENSITY, false );
	LockRotation( bodyId );

	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.displacedVolume = 0.125f / 0.4f;
	fluidDef.dragScale = 1.5f;
	b3Body_SetFluidDef( bodyId, &fluidDef );

	b3FluidVolumeId firstId = CreateDampedPool( worldId, 5.0f, 5.0f );
	Simulate( worldId, 50.0f, 2, 30.0f );
	ENSURE_NEAR( GetHeight( bodyId ) - 0.05f, 1e-4f );

	b3DestroyFluidVolume( firstId );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( bodyId ), fluidDef ) );

	b3World_Step( worldId, 1.0f / 50.0f, 2 );
	ENSURE( b3World_GetFluidEvents( worldId ).endCount == 1 );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( bodyId ), fluidDef ) );

	b3FluidVolumeId secondId = CreateDampedPool( worldId, 5.0f, 5.0f );
	ENSURE( B3_ID_EQUALS( firstId, secondId ) == false );
	ENSURE( b3FluidVolume_IsValid( firstId ) == false );
	ENSURE( b3FluidVolume_IsValid( secondId ) );

	b3Body_SetTransform( bodyId, (b3Pos){ 0.0f, 0.1f, 0.0f }, b3Quat_identity );
	b3Body_SetLinearVelocity( bodyId, b3Vec3_zero );
	Simulate( worldId, 50.0f, 2, 30.0f );
	ENSURE_NEAR( GetHeight( bodyId ) - 0.05f, 1e-4f );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyId ), secondId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// The array setter skips stale ids, including one whose slot was reused
static int BatchSetter( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodyIds[4];
	for ( int i = 0; i < 4; ++i )
	{
		bodyIds[i] = CreateBox( worldId, (b3Pos){ 2.0f * i, 0.1f, 0.0f }, b3Quat_identity,
								(b3Vec3){ 0.25f, 0.25f, 0.25f }, WATER_DENSITY, false );
		LockRotation( bodyIds[i] );
	}

	// A stored def that destroying the body must reset, so the slot's next body starts at default
	b3BodyId staleId = bodyIds[1];
	b3BodyFluidDef staleDef = b3DefaultBodyFluidDef();
	staleDef.displacedVolume = 0.125f / 0.3f;
	staleDef.dragScale = 3.0f;
	b3Body_SetFluidDef( staleId, &staleDef );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( staleId ), staleDef ) );

	b3DestroyBody( staleId );
	b3BodyId reusedId = CreateBox( worldId, (b3Pos){ 2.0f, 0.1f, 0.0f }, b3Quat_identity,
								   (b3Vec3){ 0.25f, 0.25f, 0.25f }, WATER_DENSITY, false );
	LockRotation( reusedId );
	ENSURE( reusedId.index1 == staleId.index1 );
	ENSURE( B3_ID_EQUALS( reusedId, staleId ) == false );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( reusedId ), b3DefaultBodyFluidDef() ) );

	b3BodyId ids[4] = { bodyIds[0], staleId, bodyIds[2], bodyIds[3] };
	b3BodyFluidDef defs[4];
	for ( int i = 0; i < 4; ++i )
	{
		defs[i] = b3DefaultBodyFluidDef();
		defs[i].displacedVolume = 0.125f / ( 0.2f + 0.2f * i );
	}

	defs[3].enabled = false;

	// Before any volume exists
	b3World_SetBodyFluidDefs( worldId, NULL, NULL, 0 );
	b3World_SetBodyFluidDefs( worldId, ids, defs, 4 );

	ENSURE( IsDefEqual( b3Body_GetFluidDef( bodyIds[0] ), defs[0] ) );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( reusedId ), b3DefaultBodyFluidDef() ) );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( bodyIds[2] ), defs[2] ) );
	ENSURE( IsDefEqual( b3Body_GetFluidDef( bodyIds[3] ), defs[3] ) );

	CreateDampedPool( worldId, 10.0f, 5.0f );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -5.5f, 0.0f }, (b3Vec3){ 10.0f, 0.5f, 10.0f } );
	Simulate( worldId, 50.0f, 2, 30.0f );

	// Specific gravity 0.2 and 0.6, and the disabled body sinks to the floor
	ENSURE_NEAR( GetHeight( bodyIds[0] ) - 0.15f, 1e-4f );
	ENSURE_NEAR( GetHeight( bodyIds[2] ) + 0.05f, 1e-4f );
	ENSURE( GetHeight( bodyIds[3] ) < -4.5f );

	// The reused slot is neutral and creeps under, instead of floating at the stale def's 0.4 draft
	ENSURE( b3Body_GetSubmergedFraction( reusedId ) > 0.9f );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyIds[3] ) ) );

	b3DestroyWorld( worldId );
	return 0;
}

// A sleeping floater wakes and sinks when the density is halved
static int WakeOnDensity( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 2.0f );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );

	// A dry body asleep on ground outside the pool
	CreateSlab( worldId, (b3Vec3){ 20.0f, -0.5f, 0.0f }, (b3Vec3){ 2.0f, 0.5f, 2.0f } );
	b3BodyId dryId = CreateBox( worldId, (b3Pos){ 20.0f, 0.1f, 0.0f }, b3Quat_identity,
								(b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );

	b3BodyId floaterId = CreateBox( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, b3Quat_identity,
									(b3Vec3){ 0.15f, 0.05f, 0.15f }, 600.0f, true );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3Body_IsAwake( floaterId ) == false );
	ENSURE( b3Body_IsAwake( dryId ) == false );
	float height = GetHeight( floaterId );
	ENSURE( height > -0.05f );

	b3FluidVolume_SetDensity( volumeId, 500.0f );
	ENSURE( b3Body_IsAwake( floaterId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	// Specific gravity 1.2 now
	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( GetHeight( floaterId ) < -1.9f );
	ENSURE( b3Body_GetSubmergedFraction( floaterId ) == 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// A submerged sleeper on the bed stays asleep through surface changes that keep it submerged
static int SubmergedSleeperStaysAsleep( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 2.0f );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );

	// A dry body on a ledge above the water inside the footprint
	CreateSlab( worldId, (b3Vec3){ 3.0f, 0.1f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.5f } );
	b3BodyId dryId = CreateBox( worldId, (b3Pos){ 3.0f, 0.3f, 0.0f }, b3Quat_identity,
								(b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );

	b3BodyId sinkerId = CreateBox( worldId, (b3Pos){ -1.0f, -1.9f, 0.0f }, b3Quat_identity,
								   (b3Vec3){ 0.1f, 0.1f, 0.1f }, 3000.0f, true );
	b3BodyId floaterId = CreateBox( worldId, (b3Pos){ 1.0f, 0.0f, 0.0f }, b3Quat_identity,
									(b3Vec3){ 0.15f, 0.05f, 0.15f }, 500.0f, true );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( b3Body_GetSubmergedFraction( sinkerId ) == 1.0f );

	b3FluidVolume_SetSurface( volumeId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.05f } );
	ENSURE( b3Body_IsAwake( sinkerId ) == false );
	ENSURE( b3Body_IsAwake( floaterId ) );

	// It follows the surface, resting within the sleep speed over w = 14 rad/s, 3.5 mm
	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE_NEAR( GetHeight( floaterId ) - 0.05f, 3.5e-3f );

	b3FluidVolume_SetSurface( volumeId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, -0.05f } );
	ENSURE( b3Body_IsAwake( sinkerId ) == false );
	ENSURE( b3Body_IsAwake( floaterId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE_NEAR( GetHeight( floaterId ) + 0.05f, 3.5e-3f );

	// A current wakes what it touches, not the dry body
	b3FluidVolume_SetFlowVelocity( volumeId, (b3Vec3){ 0.5f, 0.0f, 0.0f } );
	ENSURE( b3Body_IsAwake( sinkerId ) );
	ENSURE( b3Body_IsAwake( floaterId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// A pool bed at y = -2 and a sinker asleep on it at x = -1
static b3BodyId CreateSinkerScene( b3WorldId worldId )
{
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );
	return CreateBox( worldId, (b3Pos){ -1.0f, -1.9f, 0.0f }, b3Quat_identity,
					  (b3Vec3){ 0.1f, 0.1f, 0.1f }, 3000.0f, true );
}

// A SetSurface that keeps a sunk sleeper below both planes wakes it when another volume may hold it
static int OverlapSleeperWake( void )
{
	// Pool alone, then with a binned, an unbinned, and a distant binned volume over the sinker
	for ( int kind = 0; kind < 4; ++kind )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 2.0f );
		b3BodyId sinkerId = CreateSinkerScene( worldId );

		if ( kind > 0 )
		{
			b3AABB bounds = { { -2.0f, -2.0f, -1.0f }, { 0.0f, -1.0f, 1.0f } };
			if ( kind == 2 )
			{
				bounds.lowerBound.x = -B3_HUGE;
				bounds.upperBound.x = B3_HUGE;
			}
			else if ( kind == 3 )
			{
				bounds.lowerBound.x += 30.0f;
				bounds.upperBound.x += 30.0f;
			}

			CreateBasin( worldId, bounds, -1.0f, b3Vec3_zero );
		}

		Simulate( worldId, 60.0f, 4, 5.0f );
		ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
		ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( sinkerId ), volumeId ) );

		b3FluidVolume_SetSurface( volumeId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.05f } );
		bool expected = kind == 1 || kind == 2;
		if ( b3Body_IsAwake( sinkerId ) != expected )
		{
			printf( "  overlap kind %d\n", kind );
		}

		ENSURE( b3Body_IsAwake( sinkerId ) == expected );
		b3DestroyWorld( worldId );
	}

	// Lowering the pool under the basin's surface hands the sinker over
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId poolId = CreatePool( worldId, 5.0f, 2.0f );
	b3FluidVolumeId basinId = CreateBasin(
		worldId, (b3AABB){ { -2.0f, -2.0f, -1.0f }, { 0.0f, -0.5f, 1.0f } }, -0.5f, b3Vec3_zero );
	b3BodyId sinkerId = CreateSinkerScene( worldId );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( sinkerId ), poolId ) );

	b3FluidVolume_SetSurface( poolId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, -0.8f } );
	ENSURE( b3Body_IsAwake( sinkerId ) );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( sinkerId ), basinId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// A pool bed at y = -2, a dry box on ground far away, and a flat box of specific gravity 0.5
static b3BodyId CreateWakeScene( b3WorldId worldId, b3Pos position, b3BodyId* dryId )
{
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );
	CreateSlab( worldId, (b3Vec3){ 20.0f, -0.5f, 0.0f }, (b3Vec3){ 2.0f, 0.5f, 2.0f } );
	*dryId = CreateBox( worldId, (b3Pos){ 20.0f, 0.1f, 0.0f }, b3Quat_identity,
						(b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );
	return CreateBox( worldId, position, b3Quat_identity, (b3Vec3){ 0.15f, 0.05f, 0.15f }, 500.0f,
					  true );
}

// Level in the volume, and within the sleep speed over w = 14 rad/s, 3.5 mm, of the surface
static int EnsureFloating( b3BodyId bodyId, b3FluidVolumeId volumeId )
{
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyId ), volumeId ) );
	ENSURE( GetTiltSine( bodyId ) < 1e-3f );
	ENSURE_NEAR( GetHeight( bodyId ), 3.5e-3f );
	return 0;
}

// A volume created over a box asleep on the bed wakes it, and it floats up
static int WakeOnCreate( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyId dryId;
	b3BodyId bodyId = CreateWakeScene( worldId, (b3Pos){ 0.0f, -1.9f, 0.0f }, &dryId );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE_NEAR( GetHeight( bodyId ) + 1.95f, 0.01f );

	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 2.0f );
	ENSURE( b3Body_IsAwake( bodyId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 1 && events.endCount == 0 );
	ENSURE( B3_ID_EQUALS( events.beginEvents[0].bodyId, bodyId ) );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( EnsureFloating( bodyId, volumeId ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// Bounds that shrink off a sleeping floater wake it, and its touch ends in the next step
static int WakeOnBoundsShrink( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 2.0f );
	b3BodyId dryId;
	b3BodyId bodyId = CreateWakeScene( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &dryId );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( EnsureFloating( bodyId, volumeId ) == 0 );

	b3FluidVolume_SetBounds( volumeId, (b3AABB){ { 2.0f, -2.0f, -5.0f }, { 5.0f, 0.0f, 5.0f } } );
	ENSURE( b3Body_IsAwake( bodyId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 0 && events.endCount == 1 );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].bodyId, bodyId ) );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].volumeId, volumeId ) );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyId ) ) );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 0.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// Bounds that grow over a dry box asleep on the floor wake it, and its touch begins next step
static int WakeOnBoundsGrow( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreateBasin(
		worldId, (b3AABB){ { 2.0f, -2.0f, -5.0f }, { 5.0f, 0.0f, 5.0f } }, 0.0f, b3Vec3_zero );
	b3BodyId dryId;
	b3BodyId bodyId = CreateWakeScene( worldId, (b3Pos){ 0.0f, -1.9f, 0.0f }, &dryId );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyId ) ) );

	b3FluidVolume_SetBounds( volumeId, (b3AABB){ { -5.0f, -2.0f, -5.0f }, { 5.0f, 0.0f, 5.0f } } );
	ENSURE( b3Body_IsAwake( bodyId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 1 && events.endCount == 0 );
	ENSURE( B3_ID_EQUALS( events.beginEvents[0].bodyId, bodyId ) );
	ENSURE( B3_ID_EQUALS( events.beginEvents[0].volumeId, volumeId ) );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( EnsureFloating( bodyId, volumeId ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// A displacedVolume change on a sleeping floater wakes it, and at specific gravity 2 it sinks
static int WakeOnDisplacedVolume( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreatePool( worldId, 5.0f, 2.0f );
	b3BodyId dryId;
	b3BodyId bodyId = CreateWakeScene( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &dryId );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( EnsureFloating( bodyId, volumeId ) == 0 );

	SetSpecificGravity( bodyId, 2.0f );
	ENSURE( b3Body_IsAwake( bodyId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE_NEAR( GetHeight( bodyId ) + 1.95f, 0.01f );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// Turning fluids back on wakes a sleeper on the bed, which floats up, but not the dry sleepers
static int WakeOnReenable( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Bounds 1 m above the surface, so a box on a ledge at y = 0.3 is inside them but dry
	b3AABB bounds = { { -5.0f, -2.0f, -5.0f }, { 5.0f, 1.0f, 5.0f } };
	b3FluidVolumeId volumeId = CreateBasin( worldId, bounds, 0.0f, b3Vec3_zero );
	b3BodyId dryId;
	b3BodyId bodyId = CreateWakeScene( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f }, &dryId );
	CreateSlab( worldId, (b3Vec3){ 3.0f, 0.2f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.5f } );
	b3BodyId ledgeId = CreateBox( worldId, (b3Pos){ 3.0f, 0.4f, 0.0f }, b3Quat_identity,
								  (b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );

	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.enabled = false;
	b3Body_SetFluidDef( bodyId, &fluidDef );
	b3Body_SetFluidDef( dryId, &fluidDef );
	b3Body_SetFluidDef( ledgeId, &fluidDef );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE_NEAR( GetHeight( bodyId ) + 1.95f, 0.01f );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bodyId ) ) );
	ENSURE_NEAR( GetHeight( ledgeId ) - 0.4f, 0.01f );

	fluidDef.enabled = true;
	b3Body_SetFluidDef( bodyId, &fluidDef );
	b3Body_SetFluidDef( dryId, &fluidDef );
	b3Body_SetFluidDef( ledgeId, &fluidDef );
	ENSURE( b3Body_IsAwake( bodyId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );
	ENSURE( b3Body_IsAwake( ledgeId ) == false );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( EnsureFloating( bodyId, volumeId ) == 0 );
	ENSURE( b3Body_IsAwake( dryId ) == false );
	ENSURE( b3Body_IsAwake( ledgeId ) == false );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( ledgeId ) ) );

	b3DestroyWorld( worldId );
	return 0;
}

// A floater resting outside a thin tank wall, below the water level, stays dry
static int OutsideTankWall( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -1.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );
	CreateSlab( worldId, (b3Vec3){ 1.025f, -0.4f, 0.0f }, (b3Vec3){ 0.025f, 0.6f, 1.05f } );
	b3FluidVolumeId volumeId = CreateBasin(
		worldId, (b3AABB){ { -1.0f, -1.0f, -1.0f }, { 1.0f, 0.0f, 1.0f } }, 0.0f, b3Vec3_zero );

	b3Vec3 halfExtents = { 0.1f, 0.1f, 0.1f };
	b3BodyId outsideId = CreateBox( worldId, (b3Pos){ 1.151f, -0.9f, 0.0f }, b3Quat_identity,
									halfExtents, 500.0f, true );
	b3BodyId insideId = CreateBox( worldId, (b3Pos){ 0.85f, 0.0f, 0.0f }, b3Quat_identity,
								   halfExtents, 500.0f, true );

	int beginCount = 0;
	int endCount = 0;
	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		CountEvents( worldId, &beginCount, &endCount );
	}

	ENSURE( b3Body_GetSubmergedFraction( outsideId ) == 0.0f );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( outsideId ) ) );
	ENSURE_NEAR( GetHeight( outsideId ) + 0.9f, 0.01f );

	// At specific gravity 0.5 a symmetric body's center stays on the surface at any orientation
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( insideId ), volumeId ) );
	ENSURE_NEAR( GetHeight( insideId ), 0.02f );
	ENSURE( beginCount == 1 && endCount == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// Abutting volumes at different levels: each floater keeps its own level and current
static int AbuttingVolumes( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3Vec3 lowFlow = { 0.0f, 0.0f, 0.4f };
	b3Vec3 highFlow = { 0.0f, 0.0f, -0.4f };
	b3FluidVolumeId lowId = CreateBasin(
		worldId, (b3AABB){ { -3.0f, -2.0f, -10.0f }, { 0.0f, 0.0f, 10.0f } }, 0.0f, lowFlow );
	b3FluidVolumeId highId = CreateBasin(
		worldId, (b3AABB){ { 0.0f, -2.0f, -10.0f }, { 3.0f, 0.5f, 10.0f } }, 0.5f, highFlow );

	b3BodyId lowBodyId = CreateBall( worldId, (b3Pos){ -0.15f, 0.0f, 0.0f }, 0.1f, 500.0f, false );
	b3BodyId highBodyId = CreateBall( worldId, (b3Pos){ 0.15f, 0.5f, 0.0f }, 0.1f, 500.0f, false );

	// Long enough for the linear drag alone to bring the drift within 1e-4 of the current
	int beginCount = 0;
	int endCount = 0;
	for ( int i = 0; i < 900; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		CountEvents( worldId, &beginCount, &endCount );
	}

	ENSURE( beginCount == 2 && endCount == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( lowBodyId ), lowId ) );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( highBodyId ), highId ) );

	// Specific gravity 0.5 puts the centers on their own surfaces
	ENSURE_NEAR( GetHeight( lowBodyId ), 1e-3f );
	ENSURE_NEAR( GetHeight( highBodyId ) - 0.5f, 1e-3f );

	b3Vec3 lowVelocity = b3Body_GetLinearVelocity( lowBodyId );
	b3Vec3 highVelocity = b3Body_GetLinearVelocity( highBodyId );
	ENSURE_NEAR( b3Length( b3Sub( lowVelocity, lowFlow ) ), 1e-3f );
	ENSURE_NEAR( b3Length( b3Sub( highVelocity, highFlow ) ), 1e-3f );

	// Point query: the higher surface wins on the shared face, air over the lower volume is dry
	float depth = -1.0f;
	ENSURE( B3_ID_EQUALS(
		b3World_FindFluidVolume( worldId, (b3Pos){ -0.01f, -0.1f, 0.0f }, &depth ), lowId ) );
	ENSURE_NEAR( depth - 0.1f, 1e-6f );
	ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( worldId, (b3Pos){ 0.01f, 0.3f, 0.0f }, &depth ),
						  highId ) );
	ENSURE_NEAR( depth - 0.2f, 1e-6f );
	ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( worldId, (b3Pos){ 0.0f, -0.1f, 0.0f }, &depth ),
						  highId ) );
	ENSURE_NEAR( depth - 0.6f, 1e-6f );
	ENSURE(
		B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ -0.01f, 0.3f, 0.0f }, &depth ) ) );
	ENSURE( depth == 0.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// A floater below the bounds floor, partly overlapping the bounds, stays dry
static int UnderTheFloor( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeId volumeId = CreateBasin(
		worldId, (b3AABB){ { -2.0f, -1.0f, -2.0f }, { 2.0f, 0.0f, 2.0f } }, 0.0f, b3Vec3_zero );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -1.65f, 0.0f }, (b3Vec3){ 3.0f, 0.5f, 3.0f } );

	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, -1.05f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );

	int beginCount = 0;
	int endCount = 0;
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		CountEvents( worldId, &beginCount, &endCount );
	}

	ENSURE( beginCount == 0 && endCount == 0 );
	ENSURE( b3Body_GetSubmergedFraction( bodyId ) == 0.0f );
	ENSURE_NEAR( GetHeight( bodyId ) + 1.05f, 0.01f );
	ENSURE( b3Body_IsAwake( bodyId ) == false );

	// Raising the floater's center over the floor puts it in the fluid
	b3Body_SetTransform( bodyId, (b3Pos){ 0.0f, -0.9f, 0.0f }, b3Quat_identity );
	b3Body_SetAwake( bodyId, true );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyId ), volumeId ) );

	b3DestroyWorld( worldId );
	return 0;
}

static int PointQuery( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	float depth = -1.0f;
	ENSURE(
		B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ 0.0f, -1.0f, 0.0f }, &depth ) ) );
	ENSURE( depth == 0.0f );

	b3FluidVolumeId volumeId = CreateBasin(
		worldId, (b3AABB){ { -2.0f, -3.0f, -2.0f }, { 2.0f, 1.0f, 2.0f } }, 0.5f, b3Vec3_zero );

	ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( worldId, (b3Pos){ 1.0f, -1.0f, -1.0f }, &depth ),
						  volumeId ) );
	ENSURE_NEAR( depth - 1.5f, 1e-6f );
	ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( worldId, (b3Pos){ 1.0f, -1.0f, -1.0f }, NULL ),
						  volumeId ) );

	// Above the surface, outside the footprint, under the floor, and on the surface itself
	ENSURE(
		B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ 0.0f, 0.75f, 0.0f }, &depth ) ) );
	ENSURE(
		B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ 2.5f, -1.0f, 0.0f }, &depth ) ) );
	ENSURE(
		B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ 0.0f, -3.5f, 0.0f }, &depth ) ) );
	ENSURE( B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ 0.0f, 0.5f, 0.0f }, &depth ) ) );
	ENSURE( depth == 0.0f );

	b3DestroyFluidVolume( volumeId );
	ENSURE(
		B3_IS_NULL( b3World_FindFluidVolume( worldId, (b3Pos){ 1.0f, -1.0f, -1.0f }, &depth ) ) );

	b3DestroyWorld( worldId );
	return 0;
}

// Volume definition round trip, setters, user data, ids and stats
static int VolumeApi( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	int userValue = 7;
	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = (b3AABB){ { -1.0f, -2.0f, -3.0f }, { 1.0f, 2.0f, 3.0f } };
	volumeDef.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.5f };
	volumeDef.density = 800.0f;
	volumeDef.linearDrag = 0.5f;
	volumeDef.angularDrag = 0.25f;
	volumeDef.quadraticDrag = 2.0f;
	volumeDef.flowVelocity = (b3Vec3){ 1.0f, 0.0f, -1.0f };
	volumeDef.maskBits = 0x5;
	volumeDef.userData = &userValue;
	b3FluidVolumeId volumeId = b3CreateFluidVolume( worldId, &volumeDef );
	ENSURE( b3FluidVolume_IsValid( volumeId ) );
	ENSURE( b3FluidVolume_IsValid( b3_nullFluidVolumeId ) == false );

	b3FluidVolumeDef def = b3FluidVolume_GetDef( volumeId );
	ENSURE( memcmp( &def.bounds, &volumeDef.bounds, sizeof( b3AABB ) ) == 0 );
	ENSURE( memcmp( &def.surface, &volumeDef.surface, sizeof( b3Plane ) ) == 0 );
	ENSURE( def.density == 800.0f && def.linearDrag == 0.5f && def.angularDrag == 0.25f &&
			def.quadraticDrag == 2.0f );
	ENSURE( memcmp( &def.flowVelocity, &volumeDef.flowVelocity, sizeof( b3Vec3 ) ) == 0 );
	ENSURE( def.maskBits == 0x5 && def.userData == &userValue );
	ENSURE( b3FluidVolume_GetUserData( volumeId ) == &userValue );

	b3AABB bounds = { { -4.0f, -4.0f, -4.0f }, { 4.0f, 0.0f, 4.0f } };
	b3FluidVolume_SetBounds( volumeId, bounds );
	b3FluidVolume_SetSurface( volumeId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, -0.5f } );
	b3FluidVolume_SetFlowVelocity( volumeId, (b3Vec3){ 0.0f, 0.0f, 2.0f } );
	b3FluidVolume_SetDrag( volumeId, 3.0f, 4.0f, 5.0f );
	b3FluidVolume_SetDensity( volumeId, 1200.0f );
	b3FluidVolume_SetUserData( volumeId, NULL );

	def = b3FluidVolume_GetDef( volumeId );
	ENSURE( memcmp( &def.bounds, &bounds, sizeof( b3AABB ) ) == 0 );
	ENSURE( def.surface.offset == -0.5f && def.flowVelocity.z == 2.0f );
	ENSURE( def.linearDrag == 3.0f && def.angularDrag == 4.0f && def.quadraticDrag == 5.0f &&
			def.density == 1200.0f );
	ENSURE( b3FluidVolume_GetUserData( volumeId ) == NULL );

	// A def read back is valid for creation
	b3FluidVolumeId copyId = b3CreateFluidVolume( worldId, &def );
	ENSURE( b3FluidVolume_IsValid( copyId ) );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3World_GetFluidStats( worldId ).volumeCount == 2 );

	b3DestroyFluidVolume( volumeId );
	ENSURE( b3FluidVolume_IsValid( volumeId ) == false );
	ENSURE( b3FluidVolume_IsValid( copyId ) );

	b3FluidVolumeId reusedId = b3CreateFluidVolume( worldId, &def );
	ENSURE( reusedId.index1 == volumeId.index1 );
	ENSURE( reusedId.generation != volumeId.generation );
	ENSURE( b3FluidVolume_IsValid( volumeId ) == false );

	b3DestroyWorld( worldId );
	ENSURE( b3FluidVolume_IsValid( reusedId ) == false );
	return 0;
}

// User damping is restored after every fluid step, a falling-asleep step and a dt == 0 step
static int DampingRestored( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreatePool( worldId, 5.0f, 5.0f );

	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.05f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.15f, 0.05f, 0.15f }, 500.0f, true );
	b3Body_SetLinearDamping( bodyId, 0.25f );
	b3Body_SetAngularDamping( bodyId, 0.5f );

	for ( int i = 0; i < 600 && b3Body_IsAwake( bodyId ); ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( b3Body_GetLinearDamping( bodyId ) == 0.25f );
		ENSURE( b3Body_GetAngularDamping( bodyId ) == 0.5f );
	}

	ENSURE( b3Body_IsAwake( bodyId ) == false );
	ENSURE( B3_IS_NON_NULL( b3Body_GetFluidVolume( bodyId ) ) );

	b3Body_SetAwake( bodyId, true );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3Body_SetLinearDamping( bodyId, 0.75f );
	b3World_Step( worldId, 0.0f, 4 );
	ENSURE( b3Body_GetLinearDamping( bodyId ) == 0.75f );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Body_GetLinearDamping( bodyId ) == 0.75f );
	ENSURE( b3Body_GetAngularDamping( bodyId ) == 0.5f );

	b3DestroyWorld( worldId );
	return 0;
}

#define PILE_COUNT 12

// Bodies that touch no fluid see no fluid write, so a distant volume changes nothing
static uint32_t RunPile( bool withFluid, b3WorldTransform* transforms, int stepCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	if ( withFluid )
	{
		b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
		volumeDef.bounds = (b3AABB){ { 1000.0f, -10.0f, 1000.0f }, { 1010.0f, 0.0f, 1010.0f } };
		volumeDef.flowVelocity = (b3Vec3){ 1.0f, 0.0f, 0.0f };
		b3CreateFluidVolume( worldId, &volumeDef );
	}

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 10.0f, 1.0f, 10.0f );
	b3CreateHullShape( groundId, &shapeDef, &groundBox.base );

	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.2f, 0.2f );
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3Capsule capsule = { { -0.2f, 0.0f, 0.0f }, { 0.2f, 0.0f, 0.0f }, 0.15f };

	b3BodyId bodyIds[PILE_COUNT];
	for ( int i = 0; i < PILE_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.1f * ( i % 3 ), 0.5f + 0.5f * i, 0.07f * ( i % 2 ) };
		bodyDef.rotation =
			b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 0.5f, 0.25f } ), 0.3f * i );
		bodyIds[i] = b3CreateBody( worldId, &bodyDef );

		if ( i % 3 == 0 )
		{
			b3CreateHullShape( bodyIds[i], &shapeDef, &box.base );
		}
		else if ( i % 3 == 1 )
		{
			b3CreateSphereShape( bodyIds[i], &shapeDef, &sphere );
		}
		else
		{
			b3CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );
		}
	}

	uint32_t hash = B3_HASH_INIT;
	for ( int step = 0; step < stepCount; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		for ( int i = 0; i < PILE_COUNT; ++i )
		{
			b3WorldTransform transform = b3Body_GetTransform( bodyIds[i] );
			b3Vec3 linearVelocity = b3Body_GetLinearVelocity( bodyIds[i] );
			b3Vec3 angularVelocity = b3Body_GetAngularVelocity( bodyIds[i] );
			hash = b3Hash( hash, (const uint8_t*)&transform, sizeof( transform ) );
			hash = b3Hash( hash, (const uint8_t*)&linearVelocity, sizeof( linearVelocity ) );
			hash = b3Hash( hash, (const uint8_t*)&angularVelocity, sizeof( angularVelocity ) );
		}
	}

	for ( int i = 0; i < PILE_COUNT; ++i )
	{
		transforms[i] = b3Body_GetTransform( bodyIds[i] );
	}

	b3DestroyWorld( worldId );
	return hash;
}

static int DistantVolumeBitIdentical( void )
{
	b3WorldTransform plain[PILE_COUNT];
	b3WorldTransform fluid[PILE_COUNT];
	int stepCount = 180;

	uint32_t plainHash = RunPile( false, plain, stepCount );
	uint32_t fluidHash = RunPile( true, fluid, stepCount );

	ENSURE( plainHash == fluidHash );
	ENSURE( memcmp( plain, fluid, sizeof( plain ) ) == 0 );
	return 0;
}

#define FLOAT_COUNT 9

// Mixed shapes in a current; hashes transforms, velocities and the ordered event stream
static uint32_t RunFloatingScene( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = (b3AABB){ { -3.0f, -3.0f, -3.0f }, { 3.0f, 0.0f, 3.0f } };
	volumeDef.flowVelocity = (b3Vec3){ 0.6f, 0.0f, 0.2f };
	b3CreateFluidVolume( worldId, &volumeDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.15f, 0.25f );
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3Capsule capsule = { { -0.25f, 0.0f, 0.0f }, { 0.25f, 0.0f, 0.0f }, 0.12f };
	float specificGravities[3] = { 0.3f, 0.7f, 1.5f };

	b3BodyId bodyIds[FLOAT_COUNT];
	for ( int i = 0; i < FLOAT_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position =
			(b3Pos){ -1.5f + 0.9f * ( i % 3 ), 0.4f + 0.2f * ( i / 3 ), -1.0f + 0.8f * ( i / 3 ) };
		bodyDef.rotation =
			b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 0.3f, 1.0f, 0.7f } ), 0.4f * i );
		bodyDef.angularVelocity = (b3Vec3){ 0.5f, -0.2f * i, 0.1f };
		bodyIds[i] = b3CreateBody( worldId, &bodyDef );

		if ( i % 3 == 0 )
		{
			b3CreateHullShape( bodyIds[i], &shapeDef, &box.base );
		}
		else if ( i % 3 == 1 )
		{
			b3CreateSphereShape( bodyIds[i], &shapeDef, &sphere );
		}
		else
		{
			b3CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );
		}

		SetSpecificGravity( bodyIds[i], specificGravities[i / 3] );
	}

	uint32_t hash = B3_HASH_INIT;
	for ( int step = 0; step < 240; ++step )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			hash = HashBeginEvent( hash, events.beginEvents + i );
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			hash = HashEndEvent( hash, events.endEvents + i );
		}
	}

	for ( int i = 0; i < FLOAT_COUNT; ++i )
	{
		hash = HashBody( hash, bodyIds[i] );
	}

	b3DestroyWorld( worldId );
	return hash;
}

static int WorkerDeterminism( void )
{
	uint32_t reference = RunFloatingScene( 1 );
	for ( int workerCount = 2; workerCount <= 4; ++workerCount )
	{
		uint32_t hash = RunFloatingScene( workerCount );
		if ( hash != reference )
		{
			printf( "  workers=%d hash=0x%08X reference=0x%08X\n", workerCount, hash, reference );
		}
		ENSURE( hash == reference );
	}

	return 0;
}

// Enough awake bodies for several 32-body blocks of the parallel pass
#define SCENE_BODY_COUNT 240
#define SCENE_STEP_COUNT 300

typedef struct SceneResult
{
	uint32_t hash;
	int beginCount;
	int endCount;
} SceneResult;

// Two basins with currents, lifecycle calls and a volume recreated mid-run; hashes the events
static SceneResult RunEventScene( int workerCount, int switchStep, int switchWorkerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	CreateSlab( worldId, (b3Vec3){ -2.0f, -3.5f, 0.0f }, (b3Vec3){ 14.0f, 0.5f, 6.0f } );

	b3AABB lowBounds = { { -6.0f, -3.0f, -4.0f }, { 0.0f, 0.0f, 4.0f } };
	b3AABB highBounds = { { 0.0f, -3.0f, -4.0f }, { 6.0f, 0.5f, 4.0f } };
	b3Vec3 lowFlow = { -1.2f, 0.0f, 0.2f };
	b3FluidVolumeId lowId = CreateBasin( worldId, lowBounds, 0.0f, lowFlow );
	b3FluidVolumeId highId =
		CreateBasin( worldId, highBounds, 0.5f, (b3Vec3){ -1.5f, 0.0f, -0.1f } );

	b3BoxHull box = b3MakeBoxHull( 0.15f, 0.1f, 0.2f );
	b3Sphere sphere = { b3Vec3_zero, 0.15f };
	b3Capsule capsule = { { -0.2f, 0.0f, 0.0f }, { 0.2f, 0.0f, 0.0f }, 0.1f };
	float densities[3] = { 300.0f, 700.0f, 1500.0f };

	b3BodyId bodyIds[SCENE_BODY_COUNT];
	for ( int i = 0; i < SCENE_BODY_COUNT; ++i )
	{
		int ix = i % 12;
		int iz = ( i / 12 ) % 5;
		int iy = i / 60;
		b3Pos position = { -5.5f + 0.95f * ix, 0.8f + 0.6f * iy, -2.4f + 1.2f * iz };
		b3Quat rotation =
			b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 0.3f, 1.0f, 0.7f } ), 0.37f * i );
		bodyIds[i] = CreateDynamicBody( worldId, position, rotation, true );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = densities[( i / 3 ) % 3];
		if ( i % 3 == 0 )
		{
			b3CreateHullShape( bodyIds[i], &shapeDef, &box.base );
		}
		else if ( i % 3 == 1 )
		{
			b3CreateSphereShape( bodyIds[i], &shapeDef, &sphere );
		}
		else
		{
			b3CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );
		}
	}

	b3BodyFluidDef disabledDef = b3DefaultBodyFluidDef();
	disabledDef.enabled = false;
	b3BodyFluidDef enabledDef = b3DefaultBodyFluidDef();
	enabledDef.dragScale = 1.5f;

	SceneResult result = { B3_HASH_INIT, 0, 0 };
	for ( int step = 0; step < SCENE_STEP_COUNT; ++step )
	{
		if ( step == switchStep )
		{
			b3World_SetWorkerCount( worldId, switchWorkerCount );
		}

		for ( int i = 0; i < SCENE_BODY_COUNT; ++i )
		{
			if ( b3Body_IsValid( bodyIds[i] ) == false )
			{
				continue;
			}

			if ( i % 37 == 0 && step == 60 )
			{
				b3Body_Disable( bodyIds[i] );
			}
			else if ( i % 37 == 0 && step == 90 )
			{
				b3Body_Enable( bodyIds[i] );
			}
			else if ( i % 41 == 1 && step == 70 )
			{
				b3Body_SetType( bodyIds[i], b3_kinematicBody );
			}
			else if ( i % 41 == 1 && step == 110 )
			{
				b3Body_SetType( bodyIds[i], b3_dynamicBody );
			}
			else if ( i % 53 == 2 && step == 100 )
			{
				b3DestroyBody( bodyIds[i] );
			}
			else if ( i % 29 == 3 && step == 80 )
			{
				b3Body_SetFluidDef( bodyIds[i], &disabledDef );
			}
			else if ( i % 29 == 3 && step == 130 )
			{
				b3Body_SetFluidDef( bodyIds[i], &enabledDef );
			}
		}

		if ( step == 150 )
		{
			b3FluidVolume_SetSurface( highId, (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.3f } );
		}
		else if ( step == 200 )
		{
			b3DestroyFluidVolume( lowId );
		}
		else if ( step == 210 )
		{
			lowId = CreateBasin( worldId, lowBounds, 0.0f, lowFlow );
		}

		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			result.hash = HashBeginEvent( result.hash, events.beginEvents + i );
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			result.hash = HashEndEvent( result.hash, events.endEvents + i );
		}

		result.beginCount += events.beginCount;
		result.endCount += events.endCount;
	}

	for ( int i = 0; i < SCENE_BODY_COUNT; ++i )
	{
		if ( b3Body_IsValid( bodyIds[i] ) )
		{
			result.hash = HashBody( result.hash, bodyIds[i] );
		}
	}

	b3DestroyWorld( worldId );
	return result;
}

static int EventStreamDeterminism( void )
{
	SceneResult reference = RunEventScene( 1, -1, 1 );

	// Every body enters, many switch basins or leave, and the destroyed basin ends its members
	ENSURE( reference.beginCount > SCENE_BODY_COUNT );
	ENSURE( reference.endCount > SCENE_BODY_COUNT / 2 );

	for ( int workerCount = 2; workerCount <= 4; ++workerCount )
	{
		SceneResult result = RunEventScene( workerCount, -1, workerCount );
		if ( result.hash != reference.hash )
		{
			printf( "  workers=%d hash=0x%08X reference=0x%08X\n", workerCount, result.hash,
					reference.hash );
		}

		ENSURE( result.hash == reference.hash );
		ENSURE( result.beginCount == reference.beginCount &&
				result.endCount == reference.endCount );
	}

	// Changing the worker count mid-run resizes the fluid task contexts
	SceneResult result = RunEventScene( 1, 120, 3 );
	ENSURE( result.hash == reference.hash );
	result = RunEventScene( 4, 120, 2 );
	ENSURE( result.hash == reference.hash );

	return 0;
}

// Volume lookup tests: a scan over every volume, written from docs/fluid.md "Classification"

#define LOOKUP_CAPACITY 512
#define LOOKUP_BODY_COUNT 96
#define LOOKUP_RANDOM_POINT_COUNT 2000

#if defined( BOX3D_DOUBLE_PRECISION )
typedef double LookupScalar;
#else
typedef float LookupScalar;
#endif

typedef struct LookupVolume
{
	b3FluidVolumeId id;
	b3AABB bounds;
	float rotation;
	b3Plane surface;

	// Bounds by axis, the vertical axis and the turn, refreshed from the fields above
	float lower[3];
	float upper[3];
	int axis;
	b3CosSin cosSin;
} LookupVolume;

// Fluid volumes mirrored by slot, plus sphere bodies that touch nothing
typedef struct LookupScene
{
	b3WorldId worldId;
	LookupVolume volumes[LOOKUP_CAPACITY];
	int count;
	bool overflow;
	uint32_t seed;
	double center[3];
	float scale;
	b3BodyId bodyIds[LOOKUP_BODY_COUNT];
	float bodyRadii[LOOKUP_BODY_COUNT];
	float bodyExtents[LOOKUP_BODY_COUNT];

	// Some bodies aim at the bands of these volumes
	int edgeIndices[4];
	int edgeCount;

	// Volumes are created and changed with turns only in scenes that ask for them
	bool rotate;

	// Coverage
	int tieCount;
	int wetBodyCount;
	int bandBodyCount;
	int turnedPointCount;
	int turnedBodyCount;
} LookupScene;

static LookupScene s_lookupScene;

// xorshift32
static uint32_t NextRandom( uint32_t* state )
{
	uint32_t x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return x;
}

static double RandomRange( uint32_t* state, double lower, double upper )
{
	double unit = (double)( NextRandom( state ) >> 8 ) / 16777216.0;
	return lower + ( upper - lower ) * unit;
}

static int RandomInt( uint32_t* state, int count )
{
	return (int)( NextRandom( state ) % (uint32_t)count );
}

static void SetAxisComponent( b3Vec3* v, int axis, float value )
{
	if ( axis == 0 )
	{
		v->x = value;
	}
	else if ( axis == 1 )
	{
		v->y = value;
	}
	else
	{
		v->z = value;
	}
}

static double GetPosComponent( b3Pos p, int axis )
{
	return axis == 0 ? p.x : ( axis == 1 ? p.y : p.z );
}

static void SetPosComponent( b3Pos* p, int axis, double value )
{
	LookupScalar scalar = (LookupScalar)value;
	if ( axis == 0 )
	{
		p->x = scalar;
	}
	else if ( axis == 1 )
	{
		p->y = scalar;
	}
	else
	{
		p->z = scalar;
	}
}

// The bounds axis most aligned with the normal, ties going to x, then y
static int GetReferenceAxis( b3Vec3 normal )
{
	float ax = fabsf( normal.x );
	float ay = fabsf( normal.y );
	float az = fabsf( normal.z );
	if ( ax >= ay && ax >= az )
	{
		return 0;
	}

	return ay >= az ? 1 : 2;
}

static void RefreshLookupVolume( LookupVolume* volume )
{
	for ( int i = 0; i < 3; ++i )
	{
		volume->lower[i] = GetAxisComponent( volume->bounds.lowerBound, i );
		volume->upper[i] = GetAxisComponent( volume->bounds.upperBound, i );
	}

	volume->axis = GetReferenceAxis( volume->surface.normal );
	volume->cosSin =
		volume->rotation == 0.0f ? (b3CosSin){ 1.0f, 0.0f } : b3ComputeCosSin( volume->rotation );
}

// Inside the footprint, see docs/fluid.md "Rotation": axes a = (k + 1) % 3 and b = (k + 2) % 3,
// inclusive bounds when unturned, else the point turned back about the bounds center, all in double
static bool IsInReferenceFootprint( const LookupVolume* volume, const double* p )
{
	const float* lower = volume->lower;
	const float* upper = volume->upper;
	int a = ( volume->axis + 1 ) % 3;
	int b = ( volume->axis + 2 ) % 3;
	if ( volume->rotation == 0.0f )
	{
		return ( p[a] < lower[a] || upper[a] < p[a] || p[b] < lower[b] || upper[b] < p[b] ) == false;
	}

	double centerA = 0.5 * ( (double)lower[a] + (double)upper[a] );
	double centerB = 0.5 * ( (double)lower[b] + (double)upper[b] );
	double halfA = 0.5 * ( (double)upper[a] - (double)lower[a] );
	double halfB = 0.5 * ( (double)upper[b] - (double)lower[b] );
	double c = volume->cosSin.cosine;
	double s = volume->cosSin.sine;
	double da = p[a] - centerA;
	double db = p[b] - centerB;
	double la = c * da + s * db;
	double lb = c * db - s * da;
	return -halfA <= la && la <= halfA && -halfB <= lb && lb <= halfB;
}

// A point in the turned footprint frame, about the bounds center, as world coordinates on a and b
static void TurnFootprintPoint( const LookupVolume* volume, double localA, double localB,
								double* worldA, double* worldB )
{
	int a = ( volume->axis + 1 ) % 3;
	int b = ( volume->axis + 2 ) % 3;
	double centerA = 0.5 * ( (double)volume->lower[a] + (double)volume->upper[a] );
	double centerB = 0.5 * ( (double)volume->lower[b] + (double)volume->upper[b] );
	double c = volume->cosSin.cosine;
	double s = volume->cosSin.sine;
	*worldA = centerA + c * localA - s * localB;
	*worldB = centerB + s * localA + c * localB;
}

// Classification by a scan in index order: containing bounds, lowest separation, lowest index
static int FindReferenceVolume( LookupScene* scene, b3Pos point, float radius, float* separation,
								bool* contains )
{
	int bestIndex = -1;
	bool bestContains = false;
	float bestSeparation = 0.0f;
	double p[3] = { point.x, point.y, point.z };

	for ( int index = 0; index < scene->count; ++index )
	{
		const LookupVolume* volume = scene->volumes + index;
		if ( B3_IS_NULL( volume->id ) )
		{
			continue;
		}

		if ( IsInReferenceFootprint( volume, p ) == false )
		{
			continue;
		}

		const float* lower = volume->lower;
		const float* upper = volume->upper;
		int axis = volume->axis;

		// The radius extends the bounds on the side the normal points to
		b3Vec3 normal = volume->surface.normal;
		float bottom = lower[axis];
		float top = upper[axis];
		bool up = GetAxisComponent( normal, axis ) >= 0.0f;
		float reachBottom = up ? bottom : bottom - radius;
		float reachTop = up ? top + radius : top;
		if ( p[axis] < reachBottom || reachTop < p[axis] )
		{
			continue;
		}

		bool inside = bottom <= p[axis] && p[axis] <= top;
		float s = (float)( normal.x * point.x + normal.y * point.y + normal.z * point.z -
						   volume->surface.offset );

		if ( bestIndex >= 0 && inside == bestContains && s == bestSeparation && s < 0.0f )
		{
			scene->tieCount += 1;
		}

		if ( bestIndex < 0 || ( inside && bestContains == false ) ||
			 ( inside == bestContains && s < bestSeparation ) )
		{
			bestIndex = index;
			bestContains = inside;
			bestSeparation = s;
		}
	}

	*separation = bestSeparation;
	*contains = bestContains;
	return bestIndex;
}

// b3World_FindFluidVolume against the reference with r = 0
static int CheckLookupPoint( LookupScene* scene, b3Pos point )
{
	float separation;
	bool contains;
	int index = FindReferenceVolume( scene, point, 0.0f, &separation, &contains );

	b3FluidVolumeId expectedId = b3_nullFluidVolumeId;
	float expectedDepth = 0.0f;
	if ( index >= 0 && separation < 0.0f )
	{
		expectedId = scene->volumes[index].id;
		expectedDepth = -separation;
	}

	float depth = -1.0f;
	b3FluidVolumeId volumeId = b3World_FindFluidVolume( scene->worldId, point, &depth );
	if ( B3_ID_EQUALS( volumeId, expectedId ) && depth == expectedDepth )
	{
		scene->turnedPointCount +=
			B3_IS_NON_NULL( expectedId ) && scene->volumes[index].rotation != 0.0f ? 1 : 0;
		return 0;
	}

	printf( "  point (%.17g, %.17g, %.17g): volume %d depth %.9g, reference %d depth %.9g\n",
			(double)point.x, (double)point.y, (double)point.z, volumeId.index1 - 1, depth,
			expectedId.index1 - 1, expectedDepth );
	return 1;
}

// The point and its float neighbours on each axis, plus its double neighbours in double builds
static int CheckLookupNeighbours( LookupScene* scene, b3Pos point )
{
	if ( CheckLookupPoint( scene, point ) )
	{
		return 1;
	}

	for ( int axis = 0; axis < 3; ++axis )
	{
		double value = GetPosComponent( point, axis );
		for ( int side = -1; side <= 1; side += 2 )
		{
			b3Pos neighbour = point;
			SetPosComponent( &neighbour, axis, nextafterf( (float)value, side * INFINITY ) );
			if ( CheckLookupPoint( scene, neighbour ) )
			{
				return 1;
			}

#if defined( BOX3D_DOUBLE_PRECISION )
			SetPosComponent( &neighbour, axis, nextafter( value, side * INFINITY ) );
			if ( CheckLookupPoint( scene, neighbour ) )
			{
				return 1;
			}
#endif
		}
	}

	return 0;
}

// Each turned footprint's corners, edge midpoints and center with their neighbours, the boundary
// points nudged in and out, and the corners of and random points in the box around the footprint,
// which it partly misses, at the bottom, middle and top
static int CheckTurnedFootprintPoints( LookupScene* scene )
{
	for ( int index = 0; index < scene->count; ++index )
	{
		const LookupVolume* volume = scene->volumes + index;
		if ( B3_IS_NULL( volume->id ) || volume->rotation == 0.0f )
		{
			continue;
		}

		int axis = volume->axis;
		int a = ( axis + 1 ) % 3;
		int b = ( axis + 2 ) % 3;
		double halfA = 0.5 * ( (double)volume->upper[a] - (double)volume->lower[a] );
		double halfB = 0.5 * ( (double)volume->upper[b] - (double)volume->lower[b] );
		double c = fabs( (double)volume->cosSin.cosine );
		double s = fabs( (double)volume->cosSin.sine );
		double boxA = c * fabs( halfA ) + s * fabs( halfB );
		double boxB = s * fabs( halfA ) + c * fabs( halfB );
		float bottom = volume->lower[axis];
		float top = volume->upper[axis];
		float levels[3] = { bottom, 0.5f * bottom + 0.5f * top, top };

		for ( int level = 0; level < 3; ++level )
		{
			b3Pos point = { 0 };
			SetPosComponent( &point, axis, levels[level] );

			for ( int k = 0; k < 9; ++k )
			{
				double localA = ( k % 3 - 1 ) * halfA;
				double localB = ( k / 3 - 1 ) * halfB;
				double worldA, worldB;
				TurnFootprintPoint( volume, localA, localB, &worldA, &worldB );
				SetPosComponent( &point, a, worldA );
				SetPosComponent( &point, b, worldB );
				if ( CheckLookupNeighbours( scene, point ) )
				{
					return 1;
				}

				for ( int side = -1; side <= 1 && k != 4; side += 2 )
				{
					double scale = 1.0 + side / 1048576.0;
					TurnFootprintPoint( volume, scale * localA, scale * localB, &worldA, &worldB );
					SetPosComponent( &point, a, worldA );
					SetPosComponent( &point, b, worldB );
					if ( CheckLookupPoint( scene, point ) )
					{
						return 1;
					}
				}
			}

			double centerA, centerB;
			TurnFootprintPoint( volume, 0.0, 0.0, &centerA, &centerB );
			for ( int k = 0; k < 4; ++k )
			{
				SetPosComponent( &point, a, centerA + ( k % 2 == 0 ? -boxA : boxA ) );
				SetPosComponent( &point, b, centerB + ( k / 2 == 0 ? -boxB : boxB ) );
				if ( CheckLookupNeighbours( scene, point ) )
				{
					return 1;
				}
			}

			for ( int k = 0; k < 8; ++k )
			{
				double offsetA = boxA * RandomRange( &scene->seed, -1.0, 1.0 );
				double offsetB = boxB * RandomRange( &scene->seed, -1.0, 1.0 );
				SetPosComponent( &point, a, centerA + offsetA );
				SetPosComponent( &point, b, centerB + offsetB );
				if ( CheckLookupPoint( scene, point ) )
				{
					return 1;
				}
			}
		}
	}

	return 0;
}

static b3Pos RandomScenePoint( LookupScene* scene, double spread )
{
	b3Pos point = { 0 };
	for ( int axis = 0; axis < 3; ++axis )
	{
		double offset = spread * scene->scale * RandomRange( &scene->seed, -1.0, 1.0 );
		SetPosComponent( &point, axis, scene->center[axis] + offset );
	}

	return point;
}

static int CheckLookupPoints( LookupScene* scene )
{
	for ( int i = 0; i < LOOKUP_RANDOM_POINT_COUNT; ++i )
	{
		if ( CheckLookupPoint( scene, RandomScenePoint( scene, 1.25 ) ) )
		{
			return 1;
		}
	}

	// The unbounded limit, just past it, and far beyond
	float huge = B3_HUGE;
	double farValues[3] = { huge, nextafterf( huge, INFINITY ), 1.0e15 };
	for ( int i = 0; i < 3; ++i )
	{
		for ( int axis = 0; axis < 3; ++axis )
		{
			for ( int side = -1; side <= 1; side += 2 )
			{
				b3Pos point = RandomScenePoint( scene, 0.0 );
				SetPosComponent( &point, axis, side * farValues[i] );
				if ( CheckLookupPoint( scene, point ) )
				{
					return 1;
				}
			}
		}
	}

	// Each volume's corners, edge midpoints, face centers and center, with their neighbours
	for ( int index = 0; index < scene->count; ++index )
	{
		const LookupVolume* volume = scene->volumes + index;
		if ( B3_IS_NULL( volume->id ) )
		{
			continue;
		}

		for ( int k = 0; k < 27; ++k )
		{
			b3Pos point = { 0 };
			int code = k;
			for ( int axis = 0; axis < 3; ++axis )
			{
				float lower = GetAxisComponent( volume->bounds.lowerBound, axis );
				float upper = GetAxisComponent( volume->bounds.upperBound, axis );
				int choice = code % 3;
				code /= 3;
				float value =
					choice == 0 ? lower : ( choice == 1 ? 0.5f * lower + 0.5f * upper : upper );
				SetPosComponent( &point, axis, value );
			}

			if ( CheckLookupNeighbours( scene, point ) )
			{
				return 1;
			}
		}
	}

	return CheckTurnedFootprintPoints( scene );
}

static void CreateLookupVolume( LookupScene* scene, const b3FluidVolumeDef* def )
{
	b3FluidVolumeId volumeId = b3CreateFluidVolume( scene->worldId, def );
	int index = volumeId.index1 - 1;
	if ( index < 0 || LOOKUP_CAPACITY <= index )
	{
		scene->overflow = true;
		return;
	}

	scene->volumes[index] = (LookupVolume){ volumeId, def->bounds, def->rotation, def->surface };
	RefreshLookupVolume( scene->volumes + index );
	scene->count = b3MaxInt( scene->count, index + 1 );
}

static void DestroyLookupVolume( LookupScene* scene, int index )
{
	if ( index >= 0 )
	{
		b3DestroyFluidVolume( scene->volumes[index].id );
		scene->volumes[index].id = b3_nullFluidVolumeId;
	}
}

static void SetLookupBounds( LookupScene* scene, int index, b3AABB bounds )
{
	b3FluidVolume_SetBounds( scene->volumes[index].id, bounds );
	scene->volumes[index].bounds = bounds;
	RefreshLookupVolume( scene->volumes + index );
}

static void SetLookupSurface( LookupScene* scene, int index, b3Plane surface )
{
	b3FluidVolume_SetSurface( scene->volumes[index].id, surface );
	scene->volumes[index].surface = surface;
	RefreshLookupVolume( scene->volumes + index );
}

static void SetLookupRotation( LookupScene* scene, int index, float rotation )
{
	b3FluidVolume_SetRotation( scene->volumes[index].id, rotation );
	scene->volumes[index].rotation = rotation;
	RefreshLookupVolume( scene->volumes + index );
}

// Both zeros, near-zero turns, eighth, quarter and half turns, one far past a full turn, and random
static float RandomLookupRotation( LookupScene* scene )
{
	float rotations[10] = { 0.0f,		   -0.0f,		  1.0e-7f, -1.0e-7f, 0.25f * B3_PI,
							-0.25f * B3_PI, 0.5f * B3_PI, -0.5f * B3_PI, B3_PI,	  1000.0f };
	int kind = RandomInt( &scene->seed, 13 );
	return kind < 10 ? rotations[kind] : (float)RandomRange( &scene->seed, -B3_PI, B3_PI );
}

static int CountLiveVolumes( const LookupScene* scene )
{
	int liveCount = 0;
	for ( int index = 0; index < scene->count; ++index )
	{
		liveCount += B3_IS_NON_NULL( scene->volumes[index].id ) ? 1 : 0;
	}

	return liveCount;
}

// Much larger than the scene: unbounded, or widened by a bounds change
static bool IsHugeVolume( const LookupScene* scene, const LookupVolume* volume )
{
	b3Vec3 extent = b3Sub( volume->bounds.upperBound, volume->bounds.lowerBound );
	float limit = 10.0f * scene->scale;
	return extent.x > limit || extent.y > limit || extent.z > limit;
}

// Standing along y with a footprint narrower than half of B3_HUGE, see docs/fluid.md
static bool IsBinnedVolume( const LookupVolume* volume )
{
	b3Vec3 extent = b3Sub( volume->bounds.upperBound, volume->bounds.lowerBound );
	float limit = 0.5f * B3_HUGE;
	return GetReferenceAxis( volume->surface.normal ) == 1 && extent.x < limit && extent.z < limit;
}

// A random live slot, -1 when there is none
static int RandomLiveVolume( LookupScene* scene, bool ordinaryOnly, bool binnedOnly )
{
	int live[LOOKUP_CAPACITY];
	int liveCount = 0;
	for ( int index = 0; index < scene->count; ++index )
	{
		const LookupVolume* volume = scene->volumes + index;
		if ( B3_IS_NULL( volume->id ) || ( ordinaryOnly && IsHugeVolume( scene, volume ) ) ||
			 ( binnedOnly && IsBinnedVolume( volume ) == false ) )
		{
			continue;
		}

		live[liveCount++] = index;
	}

	return liveCount == 0 ? -1 : live[RandomInt( &scene->seed, liveCount )];
}

// Sizes from a ten-thousandth of the scene up to the whole scene
static b3AABB RandomLookupBounds( LookupScene* scene )
{
	b3AABB bounds;
	for ( int axis = 0; axis < 3; ++axis )
	{
		double size = scene->scale * pow( 10.0, -4.0 * RandomRange( &scene->seed, 0.0, 1.0 ) );
		double lower = scene->center[axis] + scene->scale * RandomRange( &scene->seed, -1.0, 1.0 );
		SetAxisComponent( &bounds.lowerBound, axis, (float)lower );
		SetAxisComponent( &bounds.upperBound, axis, (float)( lower + size ) );
	}

	return bounds;
}

// Mostly +y, and -y, x, z, tilted and 45 degree normals, at a level below, in or above the bounds
static b3Plane RandomLookupSurface( LookupScene* scene, b3AABB bounds )
{
	uint32_t* seed = &scene->seed;
	float side = RandomInt( seed, 2 ) == 0 ? -1.0f : 1.0f;
	float tiltX = (float)RandomRange( seed, -0.4, 0.4 );
	float tiltZ = (float)RandomRange( seed, -0.4, 0.4 );
	b3Vec3 normal = b3Vec3_axisY;

	int kind = RandomInt( seed, 12 );
	if ( kind == 6 )
	{
		normal = (b3Vec3){ 0.0f, -1.0f, 0.0f };
	}
	else if ( kind == 7 )
	{
		normal = (b3Vec3){ side, 0.0f, 0.0f };
	}
	else if ( kind == 8 )
	{
		normal = (b3Vec3){ 0.0f, 0.0f, side };
	}
	else if ( kind == 9 )
	{
		normal = b3Normalize( (b3Vec3){ tiltX, 1.0f, tiltZ } );
	}
	else if ( kind == 10 )
	{
		normal = b3Normalize( (b3Vec3){ side, 1.0f, 0.0f } );
	}
	else if ( kind == 11 )
	{
		normal = b3Normalize( (b3Vec3){ side, tiltX, 1.5f * tiltZ } );
	}

	b3Vec3 point;
	for ( int axis = 0; axis < 3; ++axis )
	{
		double lower = GetAxisComponent( bounds.lowerBound, axis );
		double upper = GetAxisComponent( bounds.upperBound, axis );
		if ( upper - lower > 10.0 * scene->scale )
		{
			lower = scene->center[axis] - scene->scale;
			upper = scene->center[axis] + scene->scale;
		}

		double t = RandomRange( seed, -0.3, 1.3 );
		SetAxisComponent( &point, axis, (float)( lower + t * ( upper - lower ) ) );
	}

	return (b3Plane){ normal, b3Dot( normal, point ) };
}

static b3FluidVolumeDef MakeLookupVolumeDef( LookupScene* scene )
{
	uint32_t* seed = &scene->seed;
	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	int sourceIndex = RandomLiveVolume( scene, true, false );
	int kind = RandomInt( seed, 10 );

	if ( sourceIndex >= 0 && kind == 0 )
	{
		// An identical duplicate
		def.bounds = scene->volumes[sourceIndex].bounds;
		def.rotation = scene->volumes[sourceIndex].rotation;
		def.surface = scene->volumes[sourceIndex].surface;
		return def;
	}

	b3AABB bounds = RandomLookupBounds( scene );
	if ( sourceIndex >= 0 && kind <= 3 )
	{
		// Shares a face with the source, flush with it on the other axes half the time
		b3AABB source = scene->volumes[sourceIndex].bounds;
		int axis = RandomInt( seed, 3 );
		bool flush = RandomInt( seed, 2 ) == 0;
		for ( int i = 0; i < 3; ++i )
		{
			float lower = GetAxisComponent( bounds.lowerBound, i );
			float upper = GetAxisComponent( bounds.upperBound, i );
			if ( i == axis )
			{
				float extent = upper - lower;
				if ( RandomInt( seed, 2 ) == 0 )
				{
					lower = GetAxisComponent( source.upperBound, i );
					upper = lower + extent;
				}
				else
				{
					upper = GetAxisComponent( source.lowerBound, i );
					lower = upper - extent;
				}
			}
			else if ( flush )
			{
				lower = GetAxisComponent( source.lowerBound, i );
				upper = GetAxisComponent( source.upperBound, i );
			}

			SetAxisComponent( &bounds.lowerBound, i, lower );
			SetAxisComponent( &bounds.upperBound, i, upper );
		}
	}

	// Zero width footprints and zero height
	int flat = RandomInt( seed, 12 );
	if ( flat == 0 || flat == 2 )
	{
		bounds.upperBound.x = bounds.lowerBound.x;
	}

	if ( flat == 1 || flat == 2 )
	{
		bounds.upperBound.z = bounds.lowerBound.z;
	}

	if ( flat == 3 )
	{
		bounds.upperBound.y = bounds.lowerBound.y;
	}

	def.bounds = bounds;
	def.surface = RandomLookupSurface( scene, bounds );
	if ( scene->rotate )
	{
		def.rotation = RandomLookupRotation( scene );
	}

	return def;
}

// The binned volume with the highest level surface, else any binned volume, else -1
static int FindHighestVolume( LookupScene* scene )
{
	int highestIndex = -1;
	for ( int index = 0; index < scene->count; ++index )
	{
		const LookupVolume* volume = scene->volumes + index;
		if ( B3_IS_NON_NULL( volume->id ) && IsBinnedVolume( volume ) &&
			 volume->surface.normal.y == 1.0f &&
			 ( highestIndex < 0 ||
			   volume->surface.offset > scene->volumes[highestIndex].surface.offset ) )
		{
			highestIndex = index;
		}
	}

	return highestIndex >= 0 ? highestIndex : RandomLiveVolume( scene, false, true );
}

// Never binned, with a binned volume's plane so they tie: an x strip over the highest, or unbounded
static b3FluidVolumeDef MakeUnboundedVolumeDef( LookupScene* scene, int ordinal )
{
	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	b3AABB unbounded = def.bounds;
	int sourceIndex =
		ordinal % 2 == 0 ? FindHighestVolume( scene ) : RandomLiveVolume( scene, false, true );
	if ( sourceIndex < 0 )
	{
		def.surface = (b3Plane){ b3Vec3_axisY, (float)scene->center[1] };
		return def;
	}

	def.surface = scene->volumes[sourceIndex].surface;
	if ( ordinal % 2 == 0 )
	{
		def.bounds = scene->volumes[sourceIndex].bounds;
		def.bounds.lowerBound.x = unbounded.lowerBound.x;
		def.bounds.upperBound.x = unbounded.upperBound.x;
	}

	return def;
}

static void PopulateLookupScene( LookupScene* scene, int volumeCount, int unboundedCount )
{
	for ( int i = 0; i < volumeCount; ++i )
	{
		b3FluidVolumeDef def = MakeLookupVolumeDef( scene );
		CreateLookupVolume( scene, &def );
	}

	for ( int i = 0; i < unboundedCount; ++i )
	{
		b3FluidVolumeDef def = MakeUnboundedVolumeDef( scene, i );
		CreateLookupVolume( scene, &def );
	}
}

static b3AABB MutateLookupBounds( LookupScene* scene, int index )
{
	uint32_t* seed = &scene->seed;
	b3AABB bounds = scene->volumes[index].bounds;
	if ( IsHugeVolume( scene, scene->volumes + index ) )
	{
		return RandomLookupBounds( scene );
	}

	b3Vec3 center = b3MulSV( 0.5f, b3Add( bounds.lowerBound, bounds.upperBound ) );
	b3Vec3 extent = b3Sub( bounds.upperBound, bounds.lowerBound );
	float huge = B3_HUGE;
	float offsetX = scene->scale * (float)RandomRange( seed, -1.0, 1.0 );
	float offsetY = scene->scale * (float)RandomRange( seed, -1.0, 1.0 );
	float offsetZ = scene->scale * (float)RandomRange( seed, -1.0, 1.0 );
	b3Vec3 offset = { offsetX, offsetY, offsetZ };

	switch ( RandomInt( seed, 8 ) )
	{
		case 0:
			// Move within the scene
			bounds.lowerBound = b3Add( bounds.lowerBound, offset );
			bounds.upperBound = b3Add( bounds.upperBound, offset );
			break;

		case 1:
			// Move past the scene so the grid has to grow
			offset = MakeAxisVector( RandomInt( seed, 2 ) == 0 ? 0 : 2,
									 ( offsetX < 0.0f ? -3.0f : 3.0f ) * scene->scale );
			bounds.lowerBound = b3Add( bounds.lowerBound, offset );
			bounds.upperBound = b3Add( bounds.upperBound, offset );
			break;

		case 2:
			bounds.lowerBound = b3MulSub( center, 0.25f, extent );
			bounds.upperBound = b3MulAdd( center, 0.25f, extent );
			break;

		case 3:
			bounds.lowerBound = b3MulSub( center, 2.0f, extent );
			bounds.upperBound = b3MulAdd( center, 2.0f, extent );
			break;

		case 4:
			bounds.upperBound.x = bounds.lowerBound.x;
			break;

		case 5:
			bounds = b3DefaultFluidVolumeDef().bounds;
			break;

		case 6:
			// Binned but nearly as wide as binning allows, so a few cells span everything
			bounds.lowerBound.x = center.x - 0.2f * huge;
			bounds.upperBound.x = center.x + 0.2f * huge;
			break;

		default:
			// Too wide to bin
			bounds.lowerBound.x = center.x - 0.3f * huge;
			bounds.upperBound.x = center.x + 0.3f * huge;
			break;
	}

	return bounds;
}

// A surface on another axis than the volume's current one
static b3Plane SwingLookupSurface( LookupScene* scene, int index )
{
	const LookupVolume* volume = scene->volumes + index;
	int axis = GetReferenceAxis( volume->surface.normal );
	b3Plane surface = RandomLookupSurface( scene, volume->bounds );
	while ( GetReferenceAxis( surface.normal ) == axis )
	{
		surface = RandomLookupSurface( scene, volume->bounds );
	}

	return surface;
}

static bool IsEdgeVolume( const LookupScene* scene, int index )
{
	for ( int i = 0; i < scene->edgeCount; ++i )
	{
		if ( scene->edgeIndices[i] == index )
		{
			return true;
		}
	}

	return false;
}

// The binned volume reaching furthest along an x or z direction, so it bounds the lookup grid there
static int FindEdgeVolume( const LookupScene* scene, int axis, float sign )
{
	int edgeIndex = -1;
	float edgeFace = 0.0f;
	for ( int index = 0; index < scene->count; ++index )
	{
		const LookupVolume* volume = scene->volumes + index;
		float face = sign * ( sign > 0.0f ? volume->upper[axis] : volume->lower[axis] );
		if ( B3_IS_NON_NULL( volume->id ) && IsBinnedVolume( volume ) &&
			 IsEdgeVolume( scene, index ) == false && ( edgeIndex < 0 || face > edgeFace ) )
		{
			edgeIndex = index;
			edgeFace = face;
		}
	}

	return edgeIndex;
}

static void CreateLookupBodies( LookupScene* scene )
{
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.maskBits = 0;

	for ( int i = 0; i < LOOKUP_BODY_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.enableSleep = false;
		b3BodyId bodyId = b3CreateBody( scene->worldId, &bodyDef );

		float radius = (float)RandomRange( &scene->seed, 0.02, 1.5 );
		b3Sphere sphere = { b3Vec3_zero, radius };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );

		scene->bodyIds[i] = bodyId;
		scene->bodyRadii[i] = radius;
		scene->bodyExtents[i] = b3Length( b3Body_GetMaxExtent( bodyId ) );
	}
}

// Random points, and points on and around the band beyond a volume's top
static b3Pos PlaceLookupBody( LookupScene* scene, int bodyIndex )
{
	uint32_t* seed = &scene->seed;
	int index = RandomLiveVolume( scene, true, false );
	if ( bodyIndex % 4 == 3 && scene->edgeCount > 0 )
	{
		index = scene->edgeIndices[( bodyIndex / 4 ) % scene->edgeCount];
	}

	if ( bodyIndex % 4 == 0 || index < 0 )
	{
		return RandomScenePoint( scene, 1.25 );
	}

	const LookupVolume* volume = scene->volumes + index;
	b3Vec3 normal = volume->surface.normal;
	int axis = GetReferenceAxis( normal );
	float sign = GetAxisComponent( normal, axis ) >= 0.0f ? 1.0f : -1.0f;
	float radius = scene->bodyExtents[bodyIndex];

	b3Pos point = { 0 };
	for ( int i = 0; i < 3; ++i )
	{
		float lower = GetAxisComponent( volume->bounds.lowerBound, i );
		float upper = GetAxisComponent( volume->bounds.upperBound, i );
		int choice = RandomInt( seed, 8 );
		double value = RandomRange( seed, lower, upper );

		if ( i == axis )
		{
			float face = sign > 0.0f ? upper : lower;
			float reach = sign > 0.0f ? upper + radius : lower - radius;
			if ( choice == 0 )
			{
				value = reach;
			}
			else if ( choice == 1 )
			{
				value = nextafterf( reach, sign * INFINITY );
			}
			else if ( choice <= 5 )
			{
				value = face + sign * 1.25 * radius * RandomRange( seed, 0.0, 1.0 );
			}
		}
		else if ( choice == 0 )
		{
			value = lower;
		}
		else if ( choice == 1 )
		{
			value = upper;
		}
		else if ( choice == 2 )
		{
			value = nextafterf( upper, INFINITY );
		}

		SetPosComponent( &point, i, value );
	}

	// A turned footprint carries the choices off the vertical axis with it
	if ( volume->rotation != 0.0f )
	{
		int a = ( axis + 1 ) % 3;
		int b = ( axis + 2 ) % 3;
		double centerA = 0.5 * ( (double)volume->lower[a] + (double)volume->upper[a] );
		double centerB = 0.5 * ( (double)volume->lower[b] + (double)volume->upper[b] );
		double worldA, worldB;
		TurnFootprintPoint( volume, GetPosComponent( point, a ) - centerA,
							GetPosComponent( point, b ) - centerB, &worldA, &worldB );
		SetPosComponent( &point, a, worldA );
		SetPosComponent( &point, b, worldB );
	}

	return point;
}

// b3Body_GetFluidVolume against the reference with r = |maxExtent| at the predicted center
static int CheckLookupBodies( LookupScene* scene, int subStepCount )
{
	float timeStep = 1.0f / 60.0f;
	float beta = (float)( subStepCount - 1 ) / (float)( 2 * subStepCount );
	float lead = beta * timeStep;
	int expected[LOOKUP_BODY_COUNT];
	float separations[LOOKUP_BODY_COUNT];
	bool contains[LOOKUP_BODY_COUNT];

	for ( int i = 0; i < LOOKUP_BODY_COUNT; ++i )
	{
		b3BodyId bodyId = scene->bodyIds[i];
		b3Vec3 velocity = b3Vec3_zero;
		if ( i % 2 == 1 )
		{
			velocity.x = (float)RandomRange( &scene->seed, -2.0, 2.0 );
			velocity.y = (float)RandomRange( &scene->seed, -2.0, 2.0 );
			velocity.z = (float)RandomRange( &scene->seed, -2.0, 2.0 );
		}

		b3Body_SetTransform( bodyId, PlaceLookupBody( scene, i ), b3Quat_identity );
		b3Body_SetLinearVelocity( bodyId, velocity );
		b3Body_SetAngularVelocity( bodyId, b3Vec3_zero );

		b3Vec3 shift = b3MulSV( lead, b3Body_GetLinearVelocity( bodyId ) );
		b3Pos center = b3OffsetPos( b3Body_GetWorldCenter( bodyId ), shift );
		expected[i] = FindReferenceVolume( scene, center, scene->bodyExtents[i], separations + i,
										   contains + i );
	}

	b3World_Step( scene->worldId, timeStep, subStepCount );

	for ( int i = 0; i < LOOKUP_BODY_COUNT; ++i )
	{
		// A centered sphere is wet exactly when its center is less than a radius above the surface
		bool wet = expected[i] >= 0 && separations[i] < scene->bodyRadii[i];
		b3FluidVolumeId expectedId = wet ? scene->volumes[expected[i]].id : b3_nullFluidVolumeId;
		b3FluidVolumeId volumeId = b3Body_GetFluidVolume( scene->bodyIds[i] );
		if ( B3_ID_EQUALS( volumeId, expectedId ) == false )
		{
			b3Pos position = b3Body_GetPosition( scene->bodyIds[i] );
			printf( "  body %d near (%.9g, %.9g, %.9g): volume %d, reference %d separation %.9g "
					"radius %.9g reach %.9g\n",
					i, (double)position.x, (double)position.y, (double)position.z,
					volumeId.index1 - 1, expectedId.index1 - 1, separations[i], scene->bodyRadii[i],
					scene->bodyExtents[i] );
			return 1;
		}

		scene->wetBodyCount += wet ? 1 : 0;
		scene->bandBodyCount += wet && contains[i] == false ? 1 : 0;
		scene->turnedBodyCount += wet && scene->volumes[expected[i]].rotation != 0.0f ? 1 : 0;
	}

	return 0;
}

static int CheckLookupScene( LookupScene* scene, const char* stage, int subStepCount )
{
	if ( scene->overflow || CheckLookupPoints( scene ) || CheckLookupBodies( scene, subStepCount ) )
	{
		printf( "  lookup stage: %s, seed 0x%08X\n", stage, scene->seed );
		return 1;
	}

	return 0;
}

typedef struct LookupConfig
{
	uint32_t seed;
	int volumeCount;
	int unboundedCount;
	double center[3];
	float scale;
	bool rotate;
} LookupConfig;

static int RunLookupScene( const LookupConfig* config )
{
	LookupScene* scene = &s_lookupScene;
	memset( scene, 0, sizeof( LookupScene ) );
	scene->seed = config->seed;
	scene->center[0] = config->center[0];
	scene->center[1] = config->center[1];
	scene->center[2] = config->center[2];
	scene->scale = config->scale;
	scene->rotate = config->rotate;

	b3WorldDef worldDef = b3DefaultWorldDef();
	scene->worldId = b3CreateWorld( &worldDef );
	CreateLookupBodies( scene );

	PopulateLookupScene( scene, config->volumeCount, config->unboundedCount );
	ENSURE( CheckLookupScene( scene, "create", 2 ) == 0 );

	// Destroy a third, then create volumes in the freed slots
	int destroyCount = CountLiveVolumes( scene ) / 3 + 1;
	for ( int i = 0; i < destroyCount; ++i )
	{
		DestroyLookupVolume( scene, RandomLiveVolume( scene, false, false ) );
	}

	ENSURE( CheckLookupScene( scene, "destroy", 4 ) == 0 );
	PopulateLookupScene( scene, config->volumeCount / 4 + 1, config->unboundedCount );
	ENSURE( CheckLookupScene( scene, "refill", 2 ) == 0 );

	// Turn, turn again, turn back to either zero, and turn unturned volumes, while the bounds still
	// keep the grid's cells small
	if ( scene->rotate )
	{
		int turnCount = CountLiveVolumes( scene ) / 2 + 1;
		for ( int i = 0; i < turnCount; ++i )
		{
			int index = RandomLiveVolume( scene, false, false );
			if ( index >= 0 )
			{
				SetLookupRotation( scene, index, RandomLookupRotation( scene ) );
			}
		}

		ENSURE( CheckLookupScene( scene, "rotation", 4 ) == 0 );
	}

	// Move, shrink, grow, flatten, unbound and widen
	int changeCount = CountLiveVolumes( scene ) / 3 + 1;
	for ( int i = 0; i < changeCount; ++i )
	{
		int index = RandomLiveVolume( scene, false, false );
		if ( index >= 0 )
		{
			SetLookupBounds( scene, index, MutateLookupBounds( scene, index ) );
		}
	}

	ENSURE( CheckLookupScene( scene, "bounds", 2 ) == 0 );

	// Volumes on the grid's x and z edges face out with bands past the grid, others swing axis
	int swungIndices[LOOKUP_CAPACITY];
	b3Plane swungSurfaces[LOOKUP_CAPACITY];
	int swingCount = 0;
	for ( int edge = 0; edge < 4; ++edge )
	{
		int axis = edge < 2 ? 0 : 2;
		float sign = edge % 2 == 0 ? 1.0f : -1.0f;
		int index = FindEdgeVolume( scene, axis, sign );
		if ( index < 0 )
		{
			continue;
		}

		// Submerged up to 2 m past the face
		const LookupVolume* volume = scene->volumes + index;
		float face = sign > 0.0f ? volume->upper[axis] : volume->lower[axis];
		scene->edgeIndices[scene->edgeCount++] = index;
		swungIndices[swingCount] = index;
		swungSurfaces[swingCount] = volume->surface;
		swingCount += 1;
		SetLookupSurface( scene, index,
						  (b3Plane){ MakeAxisVector( axis, sign ), sign * face + 2.0f } );
	}

	changeCount = CountLiveVolumes( scene ) / 3 + 1;
	for ( int i = 0; i < changeCount; ++i )
	{
		int index = RandomLiveVolume( scene, false, false );
		if ( index < 0 || IsEdgeVolume( scene, index ) )
		{
			continue;
		}

		swungIndices[swingCount] = index;
		swungSurfaces[swingCount] = scene->volumes[index].surface;
		swingCount += 1;
		SetLookupSurface( scene, index, SwingLookupSurface( scene, index ) );
	}

	ENSURE( CheckLookupScene( scene, "surface", 2 ) == 0 );
	scene->edgeCount = 0;

	// In reverse, so a volume swung twice gets its first surface back
	for ( int i = swingCount - 1; i >= 0; --i )
	{
		SetLookupSurface( scene, swungIndices[i], swungSurfaces[i] );
	}

	ENSURE( CheckLookupScene( scene, "surface back", 4 ) == 0 );

	// Every volume back to an unturned footprint, by either zero
	if ( scene->rotate )
	{
		for ( int index = 0; index < scene->count; ++index )
		{
			if ( B3_IS_NON_NULL( scene->volumes[index].id ) )
			{
				SetLookupRotation( scene, index, index % 2 == 0 ? 0.0f : -0.0f );
			}
		}

		ENSURE( CheckLookupScene( scene, "rotation zero", 2 ) == 0 );
	}

	// Destroy everything, then build a new scene in the freed slots
	for ( int index = 0; index < scene->count; ++index )
	{
		if ( B3_IS_NON_NULL( scene->volumes[index].id ) )
		{
			DestroyLookupVolume( scene, index );
		}
	}

	ENSURE( CheckLookupScene( scene, "empty", 2 ) == 0 );
	PopulateLookupScene( scene, config->volumeCount, config->unboundedCount );
	ENSURE( CheckLookupScene( scene, "rebuild", 2 ) == 0 );

	b3DestroyWorld( scene->worldId );
	return 0;
}

// Point queries and bodies match a scan of every volume through every kind of volume change
static int LookupMatchesScan( void )
{
	LookupConfig configs[] = {
		{ 0x1234567u, 8, 0, { 0.0, 0.0, 0.0 }, 10.0f, false },
		{ 0x2468ACEu, 40, 2, { 0.0, 0.0, 0.0 }, 20.0f, false },
		{ 0x13579BDu, 200, 1, { -30.0, 5.0, 40.0 }, 60.0f, false },
		{ 0x0F1E2D3u, 40, 1, { 5000.0, -20.0, 5000.0 }, 30.0f, false },
		{ 0x7E57A11u, 12, 0, { 0.0, 0.0, 0.0 }, 10.0f, true },
		{ 0x3C3C3C3u, 60, 2, { 0.0, 0.0, 0.0 }, 20.0f, true },
		{ 0x6B8B456u, 200, 1, { 40.0, -5.0, -30.0 }, 60.0f, true },
		{ 0x1D2C3B4u, 40, 1, { -5000.0, 20.0, 5000.0 }, 30.0f, true },
#if defined( BOX3D_DOUBLE_PRECISION )
		{ 0x5A5A5A5u, 40, 2, { 3.0e6 + 0.3, -1.0e3, -7.5e6 }, 200.0f, false },
		{ 0x4B4B4B4u, 40, 2, { -6.0e6 - 0.7, 2.0e3, 4.5e6 }, 200.0f, true },
#endif
	};

	int tieCount = 0;
	int wetBodyCount = 0;
	int bandBodyCount = 0;
	int turnedPointCount = 0;
	int turnedBodyCount = 0;
	for ( int i = 0; i < ARRAY_COUNT( configs ); ++i )
	{
		ENSURE( RunLookupScene( configs + i ) == 0 );
		tieCount += s_lookupScene.tieCount;
		wetBodyCount += s_lookupScene.wetBodyCount;
		bandBodyCount += s_lookupScene.bandBodyCount;
		turnedPointCount += s_lookupScene.turnedPointCount;
		turnedBodyCount += s_lookupScene.turnedBodyCount;
	}

	// Ties decided by index, wet bodies, and wet bodies outside the bounds were all exercised
	ENSURE( tieCount > 0 && wetBodyCount > 0 && bandBodyCount > 0 );

	// So were points and bodies in turned volumes
	ENSURE( turnedPointCount > 0 && turnedBodyCount > 0 );
	return 0;
}

// Ignores only the bounds validation, so assert builds run inverted bounds as Release builds do
static int BoundsAssert( const char* condition, const char* fileName, int lineNumber )
{
	if ( strstr( condition, "b3IsValidAABB" ) != NULL )
	{
		return 0;
	}

	printf( "BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stdout );
	return 1;
}

static int FlushingAssert( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stdout );
	return 1;
}

// An empty lookup scene for hand-placed volumes, checked at points only
static LookupScene* StartFootprintScene( void )
{
	LookupScene* scene = &s_lookupScene;
	memset( scene, 0, sizeof( LookupScene ) );
	scene->seed = 0x3C6EF372u;
	scene->scale = 20.0f;

	b3WorldDef worldDef = b3DefaultWorldDef();
	scene->worldId = b3CreateWorld( &worldDef );
	return scene;
}

// A y-axis volume, returning its slot
static int AddFootprintVolume( LookupScene* scene, b3AABB bounds, float level )
{
	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	def.bounds = bounds;
	def.surface = (b3Plane){ b3Vec3_axisY, level };
	CreateLookupVolume( scene, &def );
	return scene->count - 1;
}

// Binned pools over x, z in [6, 9] and far off, plus an unbinned strip; returns the first slot
static int AddValidFootprints( LookupScene* scene )
{
	float huge = B3_HUGE;
	int poolIndex =
		AddFootprintVolume( scene, (b3AABB){ { 6.0f, 0.0f, 6.0f }, { 9.0f, 3.0f, 9.0f } }, 2.5f );
	AddFootprintVolume( scene, (b3AABB){ { -20.0f, -5.0f, -20.0f }, { -10.0f, 0.0f, -10.0f } },
						0.0f );
	AddFootprintVolume( scene, (b3AABB){ { -huge, -5.0f, 30.0f }, { huge, 5.0f, 35.0f } }, 0.0f );
	return poolIndex;
}

// A weightless ball that classification alone moves in or out of fluid
static b3BodyId CreateProbeBall( b3WorldId worldId, b3Pos position )
{
	b3BodyId ballId = CreateBall( worldId, position, 0.25f, 1000.0f, false );
	b3Body_SetGravityScale( ballId, 0.0f );
	return ballId;
}

static bool IsProbeIn( b3BodyId ballId, b3FluidVolumeId volumeId )
{
	return B3_ID_EQUALS( b3Body_GetFluidVolume( ballId ), volumeId ) &&
		   ( b3Body_GetSubmergedFraction( ballId ) > 0.0f ) == B3_IS_NON_NULL( volumeId );
}

static int InvertedFootprints( void )
{
	// Inverted on x, on z, on both, and on both with x barely inverted, in y [0, 5]
	b3AABB invertedBounds[4] = {
		{ { 10.0f, 0.0f, 0.0f }, { 5.0f, 5.0f, 10.0f } },
		{ { 5.0f, 0.0f, 10.0f }, { 10.0f, 5.0f, 0.0f } },
		{ { 10.0f, 0.0f, 10.0f }, { 5.0f, 5.0f, 0.0f } },
		{ { 10.0f, 0.0f, 10.0f }, { 9.995f, 5.0f, 0.0f } },
	};

	for ( int kind = 0; kind < 4; ++kind )
	{
		b3AABB inverted = invertedBounds[kind];
		b3AABB ordered = { b3Min( inverted.lowerBound, inverted.upperBound ),
						   b3Max( inverted.lowerBound, inverted.upperBound ) };
		b3Vec3 center = b3MulSV( 0.5f, b3Add( ordered.lowerBound, ordered.upperBound ) );
		b3Pos probe = { center.x, 2.0f, center.z };

		// Alone in its world, then created after valid volumes
		for ( int mixed = 0; mixed < 2; ++mixed )
		{
			LookupScene* scene = StartFootprintScene();
			b3BodyId ballId = CreateProbeBall( scene->worldId, probe );
			if ( mixed == 1 )
			{
				AddValidFootprints( scene );
			}

			AddFootprintVolume( scene, inverted, 4.0f );
			if ( mixed == 0 )
			{
				// Over the footprint and 1 m around it, below, in and above the bounds
				b3Vec3 start = b3Sub( ordered.lowerBound, (b3Vec3){ 1.0f, 0.5f, 1.0f } );
				b3Vec3 span = b3Sub( ordered.upperBound, ordered.lowerBound );
				for ( int i = 0; i < 15 * 13 * 4; ++i )
				{
					float x = start.x + ( span.x + 2.0f ) * ( i % 15 ) / 14.0f;
					float z = start.z + ( span.z + 2.0f ) * ( ( i / 15 ) % 13 ) / 12.0f;
					b3Pos point = { x, start.y + 1.75f * ( i / 195 ), z };
					ENSURE( B3_IS_NULL( b3World_FindFluidVolume( scene->worldId, point, NULL ) ) );
				}
			}

			ENSURE( CheckLookupPoints( scene ) == 0 );
			Simulate( scene->worldId, 60.0f, 4, 0.1f );
			ENSURE( IsProbeIn( ballId, b3_nullFluidVolumeId ) );
			b3DestroyWorld( scene->worldId );
		}

		// The ordered pool alone under the ball turns inverted and back
		LookupScene* scene = StartFootprintScene();
		b3BodyId ballId = CreateProbeBall( scene->worldId, probe );
		int index = AddFootprintVolume( scene, ordered, 4.0f );
		b3FluidVolumeId volumeId = scene->volumes[index].id;

		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, volumeId ) );

		SetLookupBounds( scene, index, inverted );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, b3_nullFluidVolumeId ) );

		SetLookupBounds( scene, index, ordered );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, volumeId ) );
		b3DestroyWorld( scene->worldId );
	}

	return 0;
}

static int ExtremeFootprints( void )
{
	// Zero width pairs near the float limit, never binned, and at 0.9 B3_HUGE, binned
	float edges[2] = { 3.0e38f, 0.9f * B3_HUGE };
	for ( int i = 0; i < 2; ++i )
	{
		LookupScene* scene = StartFootprintScene();
		b3BodyId ballId = CreateProbeBall( scene->worldId, (b3Pos){ 7.5f, 1.0f, 7.5f } );
		for ( int side = -1; side <= 1; side += 2 )
		{
			float x = side * edges[i];
			AddFootprintVolume( scene, (b3AABB){ { x, 0.0f, -1.0f }, { x, 5.0f, 1.0f } }, 4.0f );
		}

		ENSURE( CheckLookupPoints( scene ) == 0 );

		int poolIndex = AddValidFootprints( scene );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, scene->volumes[poolIndex].id ) );
		b3DestroyWorld( scene->worldId );
	}

	// A footprint far narrower than any cell, alone and then beside a pool
	LookupScene* scene = StartFootprintScene();
	b3AABB tinyBounds = { { 0.0f, 0.0f, 0.0f }, { 1.0e-38f, 5.0f, 0.0f } };
	int index = AddFootprintVolume( scene, tinyBounds, 4.0f );
	ENSURE( CheckLookupPoints( scene ) == 0 );

	float depth = 0.0f;
	b3Pos point = { 0.0f, 1.0f, 0.0f };
	b3FluidVolumeId volumeId = b3World_FindFluidVolume( scene->worldId, point, &depth );
	ENSURE( B3_ID_EQUALS( volumeId, scene->volumes[index].id ) && depth == 3.0f );

	AddValidFootprints( scene );
	ENSURE( CheckLookupPoints( scene ) == 0 );
	b3DestroyWorld( scene->worldId );
	return 0;
}

// Inverted and extreme footprints neither crash nor match, see docs/fluid.md "Volume lookup"
static int DegenerateFootprints( void )
{
	b3SetAssertFcn( BoundsAssert );
	int result = InvertedFootprints() || ExtremeFootprints();
	b3SetAssertFcn( FlushingAssert );
	return result;
}

#define POOL_SIDE 8
#define POOL_BODY_COUNT 256
#define DECOY_COUNT 30

typedef struct PoolRun
{
	b3WorldTransform transforms[POOL_BODY_COUNT];
	int volumeIndices[POOL_BODY_COUNT];
	uint32_t hash;
	int wetCount;
	int bandCount;
} PoolRun;

static PoolRun s_poolRuns[2];

// Dry volumes far out, over the pools and under the floor, and two kinds that are never binned
static void CreateDecoys( LookupScene* scene )
{
	uint32_t seed = 0x2545F491u;
	float huge = B3_HUGE;
	for ( int i = 0; i < DECOY_COUNT; ++i )
	{
		float x = (float)RandomRange( &seed, -12.0, 12.0 );
		float z = (float)RandomRange( &seed, -12.0, 12.0 );
		float size = (float)RandomRange( &seed, 0.5, 8.0 );
		float bottom = -6.0f;
		int kind = i % 5;
		if ( kind <= 1 )
		{
			x = 70.0f * cosf( 0.9f * (float)i );
			z = 70.0f * sinf( 0.9f * (float)i );
		}
		else if ( kind == 2 )
		{
			bottom = 40.0f;
		}
		else if ( kind == 3 )
		{
			bottom = -30.0f;
		}
		else
		{
			z += 100.0f;
		}

		b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
		def.bounds = (b3AABB){ { x, bottom, z }, { x + size, bottom + size, z + size } };
		def.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, bottom + 0.7f * size };
		def.flowVelocity = (b3Vec3){ -1.0f, 0.0f, 0.5f };

		if ( kind == 4 && i % 2 == 0 )
		{
			def.surface = (b3Plane){ { 1.0f, 0.0f, 0.0f }, x + 0.5f * size };
		}
		else if ( kind == 4 )
		{
			def.bounds = (b3AABB){ { -huge, -5.0f, z + 100.0f }, { huge, 5.0f, z + 110.0f } };
			def.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.0f };
		}

		CreateLookupVolume( scene, &def );
	}
}

// Mixed bodies dropped on 8x8 abutting pools with different levels and currents
static int RunPoolGrid( bool withDecoys, PoolRun* run )
{
	LookupScene* scene = &s_lookupScene;
	memset( scene, 0, sizeof( LookupScene ) );
	memset( run, 0, sizeof( PoolRun ) );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 4;
	scene->worldId = b3CreateWorld( &worldDef );
	CreateSlab( scene->worldId, (b3Vec3){ 0.0f, -4.5f, 0.0f }, (b3Vec3){ 40.0f, 0.5f, 40.0f } );

	// Every other pool is full to its top, so its floaters ride in the band above the bounds
	float cellSize = 3.0f;
	float origin = -12.0f;
	for ( int i = 0; i < POOL_SIDE; ++i )
	{
		for ( int j = 0; j < POOL_SIDE; ++j )
		{
			float level = -1.0f + 0.15f * (float)( ( 3 * i + 5 * j ) % 7 );
			float top = ( i + j ) % 2 == 0 ? level : level + 0.5f;
			float angle = 2.4f * (float)( POOL_SIDE * i + j );
			b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
			def.bounds.lowerBound = (b3Vec3){ origin + cellSize * i, -4.0f, origin + cellSize * j };
			def.bounds.upperBound =
				(b3Vec3){ origin + cellSize * ( i + 1 ), top, origin + cellSize * ( j + 1 ) };
			def.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, level };
			def.flowVelocity = (b3Vec3){ 0.3f * cosf( angle ), 0.0f, 0.3f * sinf( angle ) };
			CreateLookupVolume( scene, &def );
		}
	}

	// After the pools, so their indices match between runs
	if ( withDecoys )
	{
		CreateDecoys( scene );
	}

	ENSURE( scene->overflow == false );

	uint32_t seed = 0x6C8E9CF5u;
	float specificGravities[5] = { 0.3f, 0.5f, 0.7f, 0.9f, 1.5f };
	float radii[POOL_BODY_COUNT];
	b3BodyId bodyIds[POOL_BODY_COUNT];
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	for ( int k = 0; k < POOL_BODY_COUNT; ++k )
	{
		double x = RandomRange( &seed, -15.0, 15.0 );
		double y = RandomRange( &seed, 0.5, 6.0 );
		double z = RandomRange( &seed, -15.0, 15.0 );
		float angle = (float)RandomRange( &seed, 0.0, 3.0 );
		float size = (float)RandomRange( &seed, 0.1, 0.3 );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (LookupScalar)x, (LookupScalar)y, (LookupScalar)z };
		bodyDef.rotation =
			b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 0.3f, 1.0f, -0.6f } ), angle );
		bodyIds[k] = b3CreateBody( scene->worldId, &bodyDef );
		radii[k] = size;

		if ( k % 3 == 0 )
		{
			b3Sphere sphere = { b3Vec3_zero, size };
			b3CreateSphereShape( bodyIds[k], &shapeDef, &sphere );
		}
		else if ( k % 3 == 1 )
		{
			b3BoxHull box = b3MakeBoxHull( size, 0.6f * size, 0.8f * size );
			b3CreateHullShape( bodyIds[k], &shapeDef, &box.base );
		}
		else
		{
			b3Capsule capsule = { { -size, 0.0f, 0.0f }, { size, 0.0f, 0.0f }, 0.5f * size };
			b3CreateCapsuleShape( bodyIds[k], &shapeDef, &capsule );
		}

		SetSpecificGravity( bodyIds[k], specificGravities[k % 5] );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 2;
	float beta = (float)( subStepCount - 1 ) / (float)( 2 * subStepCount );
	float lead = beta * timeStep;
	bool awake[POOL_BODY_COUNT];
	int expected[POOL_BODY_COUNT];
	float separations[POOL_BODY_COUNT];
	float extents[POOL_BODY_COUNT];
	bool contains[POOL_BODY_COUNT];
	run->hash = B3_HASH_INIT;

	for ( int step = 0; step < 300; ++step )
	{
		// The outer pools grow 4 m outward, over bodies that landed dry past the grid
		if ( step == 150 )
		{
			for ( int index = 0; index < POOL_SIDE * POOL_SIDE; ++index )
			{
				int i = index / POOL_SIDE;
				int j = index % POOL_SIDE;
				b3AABB bounds = scene->volumes[index].bounds;
				bounds.lowerBound.x -= i == 0 ? 4.0f : 0.0f;
				bounds.upperBound.x += i == POOL_SIDE - 1 ? 4.0f : 0.0f;
				bounds.lowerBound.z -= j == 0 ? 4.0f : 0.0f;
				bounds.upperBound.z += j == POOL_SIDE - 1 ? 4.0f : 0.0f;
				SetLookupBounds( scene, index, bounds );
			}
		}

		for ( int k = 0; k < POOL_BODY_COUNT; ++k )
		{
			awake[k] = b3Body_IsAwake( bodyIds[k] );
			if ( awake[k] )
			{
				b3Vec3 shift = b3MulSV( lead, b3Body_GetLinearVelocity( bodyIds[k] ) );
				b3Pos center = b3OffsetPos( b3Body_GetWorldCenter( bodyIds[k] ), shift );
				extents[k] = b3Length( b3Body_GetMaxExtent( bodyIds[k] ) );
				expected[k] =
					FindReferenceVolume( scene, center, extents[k], separations + k, contains + k );
			}
		}

		b3World_Step( scene->worldId, timeStep, subStepCount );

		for ( int k = 0; k < POOL_BODY_COUNT; ++k )
		{
			b3FluidVolumeId volumeId = b3Body_GetFluidVolume( bodyIds[k] );
			int index = volumeId.index1 - 1;
			if ( awake[k] )
			{
				// Wet under the surface, dry when the reach misses it, either while crossing it
				float s = separations[k];
				bool reachable = expected[k] >= 0 && s < extents[k];
				bool valid = index < 0 ? ( reachable == false || s > -extents[k] )
									   : ( reachable && index == expected[k] );

				// A centered sphere is wet exactly when its center is less than a radius above
				if ( k % 3 == 0 )
				{
					valid = index == ( reachable && s < radii[k] ? expected[k] : -1 );
				}

				if ( valid == false )
				{
					printf( "  step %d body %d: volume %d, reference %d separation %g reach %g\n",
							step, k, index, expected[k], s, extents[k] );
					return 1;
				}

				run->bandCount += index >= 0 && contains[k] == false ? 1 : 0;
			}

			b3WorldTransform transform = b3Body_GetTransform( bodyIds[k] );
			b3Vec3 linearVelocity = b3Body_GetLinearVelocity( bodyIds[k] );
			run->hash = b3Hash( run->hash, (const uint8_t*)&transform, (int)sizeof( transform ) );
			run->hash =
				b3Hash( run->hash, (const uint8_t*)&linearVelocity, (int)sizeof( linearVelocity ) );
			run->hash = b3Hash( run->hash, (const uint8_t*)&index, (int)sizeof( index ) );
		}
	}

	for ( int k = 0; k < POOL_BODY_COUNT; ++k )
	{
		run->transforms[k] = b3Body_GetTransform( bodyIds[k] );
		run->volumeIndices[k] = b3Body_GetFluidVolume( bodyIds[k] ).index1 - 1;
		run->wetCount += run->volumeIndices[k] >= 0 ? 1 : 0;
	}

	b3DestroyWorld( scene->worldId );
	return 0;
}

// Bodies in many abutting pools match the reference every step, and decoys change nothing
static int ManyVolumesBodies( void )
{
	PoolRun* withDecoys = s_poolRuns + 0;
	PoolRun* withoutDecoys = s_poolRuns + 1;
	ENSURE( RunPoolGrid( true, withDecoys ) == 0 );
	ENSURE( RunPoolGrid( false, withoutDecoys ) == 0 );

	// Most bodies end in a pool, and some float above the bounds of a full pool
	ENSURE( withDecoys->wetCount > POOL_BODY_COUNT / 2 );
	ENSURE( withDecoys->bandCount > 0 );

	// The decoys reshape the lookup grid without changing any result
	ENSURE( withDecoys->hash == withoutDecoys->hash );
	ENSURE( memcmp( withDecoys->transforms, withoutDecoys->transforms,
					sizeof( withDecoys->transforms ) ) == 0 );
	ENSURE( memcmp( withDecoys->volumeIndices, withoutDecoys->volumeIndices,
					sizeof( withDecoys->volumeIndices ) ) == 0 );
	return 0;
}

// Rotation tests, see docs/fluid.md "Rotation"

// A hand-placed volume in a fresh lookup scene, returning its slot
static int AddTurnedFootprintVolume( LookupScene* scene, b3AABB bounds, b3Plane surface,
									 float rotation )
{
	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	def.bounds = bounds;
	def.rotation = rotation;
	def.surface = surface;
	CreateLookupVolume( scene, &def );
	return scene->count - 1;
}

// A point at a distance along and across a channel along x through the origin, turned so that +x
// turns toward -z, from libm rather than the engine's cosine and sine
static b3Pos ChannelPoint( float angle, float along, float across, float height )
{
	float c = cosf( angle );
	float s = sinf( angle );
	return (b3Pos){ c * along + s * across, height, c * across - s * along };
}

static b3Pos MirrorZ( b3Pos point )
{
	point.z = -point.z;
	return point;
}

// A 20 m by 2 m channel turned 45 and 30 degrees. Point queries, weightless probes and a floater
// agree with the turned box, out to its far corners past the unturned bounds, and a ball inside the
// box around the channel but off it stays dry.
static int RotatedChannel( void )
{
	float angles[2] = { 0.25f * B3_PI, B3_PI / 6.0f };
	for ( int i = 0; i < 2; ++i )
	{
		float angle = angles[i];
		LookupScene* scene = StartFootprintScene();
		b3WorldId worldId = scene->worldId;
		CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 12.0f, 0.5f, 12.0f } );
		int index = AddTurnedFootprintVolume(
			scene, (b3AABB){ { -10.0f, -2.0f, -1.0f }, { 10.0f, 0.0f, 1.0f } },
			(b3Plane){ b3Vec3_axisY, 0.0f }, angle );
		b3FluidVolumeId volumeId = scene->volumes[index].id;
		ENSURE( CheckLookupPoints( scene ) == 0 );

		// Along and across the channel, 1 m deep, then mirrored off it, past its ends and its side
		float wetPlaces[5][2] = {
			{ 0.0f, 0.0f }, { 8.0f, 0.5f }, { -9.0f, -0.8f }, { 9.8f, 0.9f }, { -9.8f, 0.9f } };
		b3Pos dryPoints[5] = {
			MirrorZ( ChannelPoint( angle, 5.0f, 0.0f, -1.0f ) ),
			MirrorZ( ChannelPoint( angle, -7.0f, 0.3f, -1.0f ) ),
			ChannelPoint( angle, 10.5f, 0.0f, -1.0f ),
			ChannelPoint( angle, -10.5f, 0.5f, -1.0f ),
			ChannelPoint( angle, 3.0f, 1.3f, -1.0f ),
		};

		b3BodyId wetProbeIds[5];
		b3BodyId dryProbeIds[5];
		for ( int k = 0; k < 5; ++k )
		{
			b3Pos wetPoint = ChannelPoint( angle, wetPlaces[k][0], wetPlaces[k][1], -1.0f );
			float depth = 0.0f;
			ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( worldId, wetPoint, &depth ), volumeId ) );
			ENSURE( depth == 1.0f );
			ENSURE( B3_IS_NULL( b3World_FindFluidVolume( worldId, dryPoints[k], &depth ) ) );
			ENSURE( depth == 0.0f );

			wetProbeIds[k] = CreateProbeBall( worldId, wetPoint );
			dryProbeIds[k] = CreateProbeBall( worldId, dryPoints[k] );
		}

		// A floater by the far corner, and a ball over the box corner that falls to the bed
		b3BodyId floaterId =
			CreateBall( worldId, ChannelPoint( angle, 9.3f, 0.6f, 0.0f ), 0.25f, 500.0f, false );
		b3BodyId dryId = CreateBall( worldId, MirrorZ( ChannelPoint( angle, 6.0f, 0.0f, 0.0f ) ),
									 0.25f, 500.0f, false );

		int floaterBeginCount = 0;
		int floaterEndCount = 0;
		int dryBeginCount = 0;
		for ( int step = 0; step < 180; ++step )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );
			b3FluidEvents events = b3World_GetFluidEvents( worldId );
			for ( int k = 0; k < events.beginCount; ++k )
			{
				floaterBeginCount += B3_ID_EQUALS( events.beginEvents[k].bodyId, floaterId ) ? 1 : 0;
				dryBeginCount += B3_ID_EQUALS( events.beginEvents[k].bodyId, dryId ) ? 1 : 0;
			}

			for ( int k = 0; k < events.endCount; ++k )
			{
				floaterEndCount += B3_ID_EQUALS( events.endEvents[k].bodyId, floaterId ) ? 1 : 0;
			}

			for ( int k = 0; k < 5 && step == 0; ++k )
			{
				ENSURE( IsProbeIn( wetProbeIds[k], volumeId ) );
				ENSURE( IsProbeIn( dryProbeIds[k], b3_nullFluidVolumeId ) );
			}
		}

		// At specific gravity 0.5 the ball's center rides on the surface
		ENSURE( floaterBeginCount == 1 && floaterEndCount == 0 && dryBeginCount == 0 );
		ENSURE( IsProbeIn( floaterId, volumeId ) );
		ENSURE_NEAR( GetHeight( floaterId ), 0.01f );
		ENSURE( IsProbeIn( dryId, b3_nullFluidVolumeId ) );
		ENSURE_NEAR( GetHeight( dryId ) + 1.75f, 0.01f );

		// Turned the other way, the channel runs through the mirrored points instead
		SetLookupRotation( scene, index, -angle );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		for ( int k = 0; k < 2; ++k )
		{
			b3Pos wetPoint = ChannelPoint( angle, wetPlaces[k + 1][0], wetPlaces[k + 1][1], -1.0f );
			ENSURE( B3_IS_NULL( b3World_FindFluidVolume( worldId, wetPoint, NULL ) ) );
			ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( worldId, dryPoints[k], NULL ), volumeId ) );
		}

		// Unturned, it reaches past the box around the turned channel along x
		SetLookupRotation( scene, index, 0.0f );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		ENSURE( B3_ID_EQUALS(
			b3World_FindFluidVolume( worldId, (b3Pos){ 9.5f, -1.0f, 0.5f }, NULL ), volumeId ) );
		ENSURE( B3_IS_NULL( b3World_FindFluidVolume(
			worldId, ChannelPoint( angle, wetPlaces[1][0], wetPlaces[1][1], -1.0f ), NULL ) ) );

		b3DestroyWorld( worldId );
	}

	return 0;
}

#define ZERO_TURN_BODY_COUNT 12

// A binned pool and an unbinned volume standing along x, with currents. Mode 0 never sets a
// rotation, modes 1 and 2 create with +0 and -0, and mode 3 sets +0 and -0 during the run.
static uint32_t RunZeroTurnScene( int mode )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -3.5f, 0.0f }, (b3Vec3){ 10.0f, 0.5f, 10.0f } );

	b3FluidVolumeDef poolDef = b3DefaultFluidVolumeDef();
	poolDef.bounds = (b3AABB){ { -6.0f, -3.0f, -4.0f }, { 0.0f, 0.0f, 4.0f } };
	poolDef.flowVelocity = (b3Vec3){ 0.4f, 0.0f, 0.2f };

	// Fluid fills x < 4 over a footprint in y and z
	b3FluidVolumeDef sideDef = b3DefaultFluidVolumeDef();
	sideDef.bounds = (b3AABB){ { 0.0f, -3.0f, -4.0f }, { 6.0f, 0.5f, 4.0f } };
	sideDef.surface = (b3Plane){ b3Vec3_axisX, 4.0f };
	sideDef.flowVelocity = (b3Vec3){ 0.3f, 0.0f, -0.1f };

	if ( mode == 1 || mode == 2 )
	{
		poolDef.rotation = mode == 1 ? 0.0f : -0.0f;
		sideDef.rotation = poolDef.rotation;
	}

	b3FluidVolumeId poolId = b3CreateFluidVolume( worldId, &poolDef );
	b3FluidVolumeId sideId = b3CreateFluidVolume( worldId, &sideDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.15f, 0.25f );
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3Capsule capsule = { { -0.25f, 0.0f, 0.0f }, { 0.25f, 0.0f, 0.0f }, 0.12f };
	float specificGravities[3] = { 0.3f, 0.7f, 1.5f };

	b3BodyId bodyIds[ZERO_TURN_BODY_COUNT];
	for ( int i = 0; i < ZERO_TURN_BODY_COUNT; ++i )
	{
		b3Pos position = { -4.5f + 0.8f * i, 0.6f + 0.2f * ( i % 3 ), -1.5f + 0.7f * ( i % 4 ) };
		b3Quat rotation =
			b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 0.3f, 1.0f, 0.7f } ), 0.4f * i );
		bodyIds[i] = CreateDynamicBody( worldId, position, rotation, true );

		if ( i % 3 == 0 )
		{
			b3CreateHullShape( bodyIds[i], &shapeDef, &box.base );
		}
		else if ( i % 3 == 1 )
		{
			b3CreateSphereShape( bodyIds[i], &shapeDef, &sphere );
		}
		else
		{
			b3CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );
		}

		SetSpecificGravity( bodyIds[i], specificGravities[( i / 3 ) % 3] );
	}

	uint32_t hash = B3_HASH_INIT;
	for ( int step = 0; step < 240; ++step )
	{
		if ( mode == 3 && ( step == 60 || step == 120 ) )
		{
			b3FluidVolume_SetRotation( poolId, step == 60 ? 0.0f : -0.0f );
			b3FluidVolume_SetRotation( sideId, step == 60 ? -0.0f : 0.0f );
		}

		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			hash = HashBeginEvent( hash, events.beginEvents + i );
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			hash = HashEndEvent( hash, events.endEvents + i );
		}
	}

	for ( int i = 0; i < ZERO_TURN_BODY_COUNT; ++i )
	{
		hash = HashBody( hash, bodyIds[i] );
	}

	b3DestroyWorld( worldId );
	return hash;
}

// Either zero, at creation or set later, runs bit-identical to a volume never turned. With all
// bodies asleep, zero on an unturned volume and a repeated turn are no-ops that wake nothing.
static int RotationZeroIdentity( void )
{
	uint32_t reference = RunZeroTurnScene( 0 );
	for ( int mode = 1; mode < 4; ++mode )
	{
		uint32_t hash = RunZeroTurnScene( mode );
		if ( hash != reference )
		{
			printf( "  mode %d hash=0x%08X reference=0x%08X\n", mode, hash, reference );
		}

		ENSURE( hash == reference );
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 8.0f, 0.5f, 4.0f } );
	b3FluidVolumeId poolId = CreateBasin(
		worldId, (b3AABB){ { -6.0f, -2.0f, -3.0f }, { -1.0f, 0.0f, 3.0f } }, 0.0f, b3Vec3_zero );

	b3FluidVolumeDef channelDef = b3DefaultFluidVolumeDef();
	channelDef.bounds = (b3AABB){ { 1.0f, -2.0f, -1.0f }, { 7.0f, 0.0f, 1.0f } };
	channelDef.rotation = 0.6f;
	b3FluidVolumeId channelId = b3CreateFluidVolume( worldId, &channelDef );

	b3Vec3 flat = { 0.15f, 0.05f, 0.15f };
	b3BodyId poolFloaterId =
		CreateBox( worldId, (b3Pos){ -3.5f, 0.0f, 0.0f }, b3Quat_identity, flat, 500.0f, true );
	b3BodyId floaterId =
		CreateBox( worldId, (b3Pos){ 4.0f, 0.0f, 0.0f }, b3Quat_identity, flat, 500.0f, true );
	b3BodyId sinkerId = CreateBox( worldId, (b3Pos){ 4.5f, -1.9f, -0.2f }, b3Quat_identity,
								   (b3Vec3){ 0.1f, 0.1f, 0.1f }, 3000.0f, true );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( poolFloaterId ), poolId ) );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( floaterId ), channelId ) );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( sinkerId ), channelId ) );

	b3FluidVolume_SetRotation( poolId, 0.0f );
	b3FluidVolume_SetRotation( poolId, -0.0f );
	b3FluidVolume_SetRotation( channelId, 0.6f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( b3FluidVolume_GetDef( poolId ).rotation == 0.0f );
	ENSURE( b3FluidVolume_GetDef( channelId ).rotation == 0.6f );

	// A real turn wakes the channel's bodies and leaves the pool asleep
	b3FluidVolume_SetRotation( channelId, 0.7f );
	ENSURE( b3Body_IsAwake( floaterId ) && b3Body_IsAwake( sinkerId ) );
	ENSURE( b3Body_IsAwake( poolFloaterId ) == false );
	b3DestroyWorld( worldId );

	// Negative zero takes the unturned path, so an axis change keeps a sinker below both planes asleep
	worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );
	b3FluidVolumeDef negativeDef = b3DefaultFluidVolumeDef();
	negativeDef.bounds = (b3AABB){ { -5.0f, -2.0f, -1.0f }, { 5.0f, 0.0f, 1.0f } };
	negativeDef.rotation = -0.0f;
	b3FluidVolumeId negativeId = b3CreateFluidVolume( worldId, &negativeDef );
	sinkerId = CreateBox( worldId, (b3Pos){ 0.0f, -1.9f, 0.0f }, b3Quat_identity, (b3Vec3){ 0.1f, 0.1f, 0.1f },
						  3000.0f, true );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( sinkerId ), negativeId ) );

	b3FluidVolume_SetSurface( negativeId, (b3Plane){ b3Vec3_axisX, 10.0f } );
	ENSURE( b3Body_IsAwake( sinkerId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// A heavy box the volume's mask skips, with a buoyant ball 4 m out along z. Resting on the bed with
// its center of mass in the footprint it is in fluid, while the ball stays outside the regions a
// volume change queries, so only the wake of the volume's recorded bodies reaches it.
static b3BodyId CreateOutrigger( b3WorldId worldId, b3Pos position )
{
	b3BodyId bodyId = CreateDynamicBody( worldId, position, b3Quat_identity, true );

	b3ShapeDef boxDef = b3DefaultShapeDef();
	boxDef.density = 5000.0f;
	boxDef.filter.categoryBits = 0x2;
	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.2f, 0.2f );
	b3CreateHullShape( bodyId, &boxDef, &box.base );

	b3ShapeDef ballDef = b3DefaultShapeDef();
	ballDef.density = 100.0f;
	ballDef.filter.categoryBits = 0x1;
	b3Sphere ball = { { 0.0f, 0.1f, 4.0f }, 0.3f };
	b3CreateSphereShape( bodyId, &ballDef, &ball );

	LockRotation( bodyId );
	return bodyId;
}

// A quarter turn wakes the floater it leaves, which ends its touch, and the sleeper on the bed it
// reaches, which floats up, but not a dry sleeper far away. A turn, and an axis change of a turned
// volume, wake the volume's bodies outside every queried region.
static int WakeOnRotation( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	def.bounds = (b3AABB){ { -5.0f, -2.0f, -1.0f }, { 5.0f, 0.0f, 1.0f } };
	def.rotation = 0.2f;
	b3FluidVolumeId volumeId = b3CreateFluidVolume( worldId, &def );

	b3BodyId dryId;
	b3BodyId floaterId = CreateWakeScene( worldId, (b3Pos){ 3.5f, 0.0f, 0.0f }, &dryId );
	b3BodyId sleeperId = CreateBox( worldId, (b3Pos){ 0.0f, -1.9f, 3.5f }, b3Quat_identity,
									(b3Vec3){ 0.15f, 0.05f, 0.15f }, 500.0f, true );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( EnsureFloating( floaterId, volumeId ) == 0 );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( sleeperId ) ) );

	b3FluidVolume_SetRotation( volumeId, 0.5f * B3_PI );
	ENSURE( b3Body_IsAwake( floaterId ) && b3Body_IsAwake( sleeperId ) );
	ENSURE( b3Body_IsAwake( dryId ) == false );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	b3FluidEvents events = b3World_GetFluidEvents( worldId );
	ENSURE( events.beginCount == 1 && events.endCount == 1 );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].bodyId, floaterId ) );
	ENSURE( B3_ID_EQUALS( events.endEvents[0].volumeId, volumeId ) );
	ENSURE( B3_ID_EQUALS( events.beginEvents[0].bodyId, sleeperId ) );
	ENSURE( B3_ID_EQUALS( events.beginEvents[0].volumeId, volumeId ) );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( floaterId ) ) );

	Simulate( worldId, 60.0f, 4, 10.0f );
	ENSURE( EnsureFloating( sleeperId, volumeId ) == 0 );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( floaterId ) ) );
	b3DestroyWorld( worldId );

	worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -2.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );
	def.maskBits = 0x1;
	volumeId = b3CreateFluidVolume( worldId, &def );
	b3BodyId outriggerId = CreateOutrigger( worldId, (b3Pos){ -2.0f, -1.8f, 0.0f } );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( outriggerId ), volumeId ) );

	// The mirrored turn keeps the box around the footprint
	b3FluidVolume_SetRotation( volumeId, -0.2f );
	ENSURE( b3Body_IsAwake( outriggerId ) );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( outriggerId ), volumeId ) );

	// A sleeper on the bed outside the turned footprint, which the axis change swings over
	b3BodyId bedId = CreateBox( worldId, (b3Pos){ 3.0f, -1.9f, -0.6f }, b3Quat_identity,
								(b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );
	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );
	ENSURE( B3_IS_NULL( b3Body_GetFluidVolume( bedId ) ) );

	// Standing along x, the footprint turns about x instead
	b3FluidVolume_SetSurface( volumeId, (b3Plane){ b3Vec3_axisX, 10.0f } );
	ENSURE( b3Body_IsAwake( outriggerId ) && b3Body_IsAwake( bedId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// The default def is unturned, GetDef returns the rotation as set, unwound or not, and a def read
// back from a turned volume creates the same volume
static int RotationApi( void )
{
	float positiveZero = 0.0f;
	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	ENSURE( memcmp( &def.rotation, &positiveZero, sizeof( float ) ) == 0 );

	LookupScene* scene = StartFootprintScene();
	def.bounds = (b3AABB){ { -2.0f, -1.0f, -4.0f }, { 2.0f, 1.0f, 4.0f } };
	def.rotation = 0.7f;
	CreateLookupVolume( scene, &def );
	b3FluidVolumeId volumeId = scene->volumes[0].id;
	ENSURE( b3FluidVolume_GetDef( volumeId ).rotation == 0.7f );

	float turns[4] = { -2.5f, 1000.0f, 0.0f, -7.0f };
	for ( int i = 0; i < 4; ++i )
	{
		b3FluidVolume_SetRotation( volumeId, turns[i] );
		ENSURE( b3FluidVolume_GetDef( volumeId ).rotation == turns[i] );
	}

	// The copy ties with the original everywhere, so the lower index wins every point
	b3FluidVolumeDef copyDef = b3FluidVolume_GetDef( volumeId );
	SetLookupRotation( scene, 0, -7.0f );
	CreateLookupVolume( scene, &copyDef );
	ENSURE( b3FluidVolume_GetDef( scene->volumes[1].id ).rotation == -7.0f );
	ENSURE( CheckLookupPoints( scene ) == 0 );

	float negativeZero = -0.0f;
	def.rotation = negativeZero;
	b3FluidVolumeId zeroId = b3CreateFluidVolume( scene->worldId, &def );
	float rotation = b3FluidVolume_GetDef( zeroId ).rotation;
	ENSURE( memcmp( &rotation, &negativeZero, sizeof( float ) ) == 0 );

	b3DestroyWorld( scene->worldId );
	return 0;
}

// Volumes standing along x and z turn about that axis, the b side turning toward -a: for x the
// long z side turns toward -y, for z the long y side toward -x. Neither is binned.
static int RotatedOffAxes( void )
{
	int axes[2] = { 0, 2 };
	for ( int i = 0; i < 2; ++i )
	{
		int axis = axes[i];
		int a = ( axis + 1 ) % 3;
		int b = ( axis + 2 ) % 3;
		b3AABB bounds;
		bounds.lowerBound = b3Add( MakeAxisVector( axis, -2.0f ),
								   b3Add( MakeAxisVector( a, -1.0f ), MakeAxisVector( b, -5.0f ) ) );
		bounds.upperBound = b3Add( MakeAxisVector( a, 1.0f ), MakeAxisVector( b, 5.0f ) );

		LookupScene* scene = StartFootprintScene();
		int index = AddTurnedFootprintVolume(
			scene, bounds, (b3Plane){ MakeAxisVector( axis, 1.0f ), 0.0f }, 0.25f * B3_PI );
		b3FluidVolumeId volumeId = scene->volumes[index].id;

		// 3 m along the turned long side and 1 m deep, then mirrored across a
		b3Pos inside = { 0 };
		SetPosComponent( &inside, axis, -1.0 );
		SetPosComponent( &inside, a, -2.1 );
		SetPosComponent( &inside, b, 2.1 );
		b3Pos outside = inside;
		SetPosComponent( &outside, a, 2.1 );

		float depth = 0.0f;
		ENSURE( B3_ID_EQUALS( b3World_FindFluidVolume( scene->worldId, inside, &depth ), volumeId ) );
		ENSURE( depth == 1.0f );
		ENSURE( B3_IS_NULL( b3World_FindFluidVolume( scene->worldId, outside, &depth ) ) );

		b3BodyId insideId = CreateProbeBall( scene->worldId, inside );
		b3BodyId outsideId = CreateProbeBall( scene->worldId, outside );
		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( insideId, volumeId ) );
		ENSURE( IsProbeIn( outsideId, b3_nullFluidVolumeId ) );
		ENSURE( CheckLookupPoints( scene ) == 0 );

		// Beside binned pools, through other turns and back to zero
		AddValidFootprints( scene );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		float turns[4] = { -0.25f * B3_PI, 3.0f, 1000.0f, 0.0f };
		for ( int k = 0; k < 4; ++k )
		{
			SetLookupRotation( scene, index, turns[k] );
			ENSURE( CheckLookupPoints( scene ) == 0 );
		}

		b3DestroyWorld( scene->worldId );
	}

	return 0;
}

// A box at the center of a turned volume rides exactly as in the unturned one, on every axis
static int RotatedDraft( void )
{
	int axes[4] = { 1, 0, 2, 1 };
	float ups[4] = { 1.0f, 1.0f, 1.0f, -1.0f };
	float turns[3] = { 0.25f * B3_PI, -2.0f, 1000.0f };
	for ( int i = 0; i < 4; ++i )
	{
		float reference;
		ENSURE( BoxDraft( 50.0f, 2, axes[i], ups[i], 1.0f, 0.0f, &reference ) == 0 );
		for ( int k = 0; k < 3; ++k )
		{
			float height;
			ENSURE( BoxDraft( 50.0f, 2, axes[i], ups[i], 1.0f, turns[k], &height ) == 0 );
			ENSURE( height == reference );
		}
	}

	return 0;
}

static int TurnedInvertedFootprints( void )
{
	// Inverted on x, on z, on both, and on both with x barely inverted, in y [0, 5]
	b3AABB invertedBounds[4] = {
		{ { 10.0f, 0.0f, 0.0f }, { 5.0f, 5.0f, 10.0f } },
		{ { 5.0f, 0.0f, 10.0f }, { 10.0f, 5.0f, 0.0f } },
		{ { 10.0f, 0.0f, 10.0f }, { 5.0f, 5.0f, 0.0f } },
		{ { 10.0f, 0.0f, 10.0f }, { 9.995f, 5.0f, 0.0f } },
	};

	float turns[3] = { 0.25f * B3_PI, -1.0f, 0.5f * B3_PI };
	b3Plane surface = { b3Vec3_axisY, 4.0f };
	for ( int kind = 0; kind < 4; ++kind )
	{
		b3AABB inverted = invertedBounds[kind];
		b3AABB ordered = { b3Min( inverted.lowerBound, inverted.upperBound ),
						   b3Max( inverted.lowerBound, inverted.upperBound ) };
		b3Vec3 center = b3MulSV( 0.5f, b3Add( ordered.lowerBound, ordered.upperBound ) );
		b3Vec3 span = b3Sub( ordered.upperBound, ordered.lowerBound );
		float reach = 0.5f * sqrtf( span.x * span.x + span.z * span.z ) + 1.0f;
		b3Pos probe = { center.x, 2.0f, center.z };
		float turn = turns[kind % 3];

		// Alone in its world, then created after valid volumes
		for ( int mixed = 0; mixed < 2; ++mixed )
		{
			LookupScene* scene = StartFootprintScene();
			b3BodyId ballId = CreateProbeBall( scene->worldId, probe );
			if ( mixed == 1 )
			{
				AddValidFootprints( scene );
			}

			int index = AddTurnedFootprintVolume( scene, inverted, surface, turn );
			if ( mixed == 0 )
			{
				// Over the box around any turn of the footprint and 1 m past it, below, in and above
				for ( int i = 0; i < 15 * 13 * 4; ++i )
				{
					float x = center.x - reach + 2.0f * reach * ( i % 15 ) / 14.0f;
					float z = center.z - reach + 2.0f * reach * ( ( i / 15 ) % 13 ) / 12.0f;
					b3Pos point = { x, -0.5f + 1.75f * ( i / 195 ), z };
					ENSURE( B3_IS_NULL( b3World_FindFluidVolume( scene->worldId, point, NULL ) ) );
				}
			}

			ENSURE( CheckLookupPoints( scene ) == 0 );
			Simulate( scene->worldId, 60.0f, 4, 0.1f );
			ENSURE( IsProbeIn( ballId, b3_nullFluidVolumeId ) );

			// Other turns, then unturned
			for ( int k = 1; k <= 3; ++k )
			{
				SetLookupRotation( scene, index, k < 3 ? turns[( kind + k ) % 3] : 0.0f );
				ENSURE( CheckLookupPoints( scene ) == 0 );
				b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
				ENSURE( IsProbeIn( ballId, b3_nullFluidVolumeId ) );
			}

			b3DestroyWorld( scene->worldId );
		}

		// The turned pool under the ball goes inverted and back
		LookupScene* scene = StartFootprintScene();
		b3BodyId ballId = CreateProbeBall( scene->worldId, probe );
		int index = AddTurnedFootprintVolume( scene, ordered, surface, turn );
		b3FluidVolumeId volumeId = scene->volumes[index].id;

		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, volumeId ) );

		SetLookupBounds( scene, index, inverted );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, b3_nullFluidVolumeId ) );

		SetLookupBounds( scene, index, ordered );
		ENSURE( CheckLookupPoints( scene ) == 0 );
		b3World_Step( scene->worldId, 1.0f / 60.0f, 4 );
		ENSURE( IsProbeIn( ballId, volumeId ) );
		b3DestroyWorld( scene->worldId );
	}

	return 0;
}

// Turned bounds near the float limit saturate their aabb, so create still wakes a sleeper under them
static int TurnedHugeBounds( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -0.5f, 0.0f }, (b3Vec3){ 5.0f, 0.5f, 5.0f } );
	b3BodyId bodyId = CreateBox( worldId, (b3Pos){ 0.0f, 0.1f, 0.0f }, b3Quat_identity,
								 (b3Vec3){ 0.1f, 0.1f, 0.1f }, 500.0f, true );

	Simulate( worldId, 60.0f, 4, 5.0f );
	ENSURE( b3World_GetAwakeBodyCount( worldId ) == 0 );

	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = (b3AABB){ { -3.0e38f, -5.0f, -3.0e38f }, { 3.0e38f, 5.0f, 3.0e38f } };
	volumeDef.rotation = 0.25f * B3_PI;
	volumeDef.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, 2.0f };
	b3FluidVolumeId volumeId = b3CreateFluidVolume( worldId, &volumeDef );
	ENSURE( b3Body_IsAwake( bodyId ) );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( B3_ID_EQUALS( b3Body_GetFluidVolume( bodyId ), volumeId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// A turned footprint in the float subnormal range, where rounding its aabb outruns the relative pad
static int TurnedSubnormalFootprint( void )
{
	LookupScene* scene = StartFootprintScene();
	float half = 1000.0f * FLT_TRUE_MIN;
	int index = AddFootprintVolume( scene, (b3AABB){ { -half, -1.0f, -half }, { half, 1.0f, half } }, 1.0f );
	SetLookupRotation( scene, index, 0.25f * B3_PI );
	ENSURE( CheckLookupPoints( scene ) == 0 );
	b3DestroyWorld( scene->worldId );
	return 0;
}

// Turned inverted footprints neither crash nor match, as in DegenerateFootprints
static int TurnedDegenerateFootprints( void )
{
	b3SetAssertFcn( BoundsAssert );
	int result = TurnedInvertedFootprints() || TurnedHugeBounds() || TurnedSubnormalFootprint();
	b3SetAssertFcn( FlushingAssert );
	return result;
}

#define TURNED_BODY_COUNT 120

// Turned binned and unbinned volumes with currents, turned, re-bounded, re-axed, created and
// destroyed during the run; hashes the bodies and the ordered event stream
static SceneResult RunTurnedScene( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CreateSlab( worldId, (b3Vec3){ 0.0f, -3.5f, 0.0f }, (b3Vec3){ 14.0f, 0.5f, 14.0f } );

	// A channel turned 35 degrees with a current along it, a pool turned past a half turn, a volume
	// standing along x, and a far decoy turned past a full turn
	b3FluidVolumeDef channelDef = b3DefaultFluidVolumeDef();
	channelDef.bounds = (b3AABB){ { -9.0f, -3.0f, -1.5f }, { 9.0f, 0.0f, 1.5f } };
	channelDef.rotation = 0.61f;
	channelDef.flowVelocity = (b3Vec3){ 0.66f, 0.0f, -0.46f };
	b3FluidVolumeId channelId = b3CreateFluidVolume( worldId, &channelDef );

	b3FluidVolumeDef poolDef = b3DefaultFluidVolumeDef();
	poolDef.bounds = (b3AABB){ { -3.0f, -3.0f, 2.0f }, { 3.0f, 0.4f, 8.0f } };
	poolDef.rotation = -2.3f;
	poolDef.surface = (b3Plane){ b3Vec3_axisY, 0.4f };
	poolDef.flowVelocity = (b3Vec3){ 0.2f, 0.0f, -0.3f };
	b3FluidVolumeId poolId = b3CreateFluidVolume( worldId, &poolDef );

	b3FluidVolumeDef sideDef = b3DefaultFluidVolumeDef();
	sideDef.bounds = (b3AABB){ { -8.0f, -3.0f, -9.0f }, { 0.0f, 1.0f, -4.0f } };
	sideDef.surface = (b3Plane){ b3Vec3_axisX, -2.0f };
	sideDef.rotation = 0.4f;
	b3CreateFluidVolume( worldId, &sideDef );

	b3FluidVolumeDef decoyDef = b3DefaultFluidVolumeDef();
	decoyDef.bounds = (b3AABB){ { 40.0f, -3.0f, 40.0f }, { 46.0f, 0.0f, 44.0f } };
	decoyDef.rotation = 1000.0f;
	b3CreateFluidVolume( worldId, &decoyDef );

	b3FluidVolumeDef extraDef = b3DefaultFluidVolumeDef();
	extraDef.bounds = (b3AABB){ { -4.0f, -3.0f, -3.0f }, { 4.0f, 0.2f, 3.0f } };
	extraDef.rotation = 2.0f;
	extraDef.surface = (b3Plane){ b3Vec3_axisY, 0.2f };

	b3BoxHull box = b3MakeBoxHull( 0.15f, 0.1f, 0.2f );
	b3Sphere sphere = { b3Vec3_zero, 0.15f };
	b3Capsule capsule = { { -0.2f, 0.0f, 0.0f }, { 0.2f, 0.0f, 0.0f }, 0.1f };
	float densities[3] = { 300.0f, 700.0f, 1500.0f };

	b3BodyId bodyIds[TURNED_BODY_COUNT];
	for ( int i = 0; i < TURNED_BODY_COUNT; ++i )
	{
		b3Pos position = { -9.0f + 1.6f * ( i % 12 ), 0.6f + 0.4f * ( i % 3 ),
						   -9.0f + 1.8f * ( i / 12 ) };
		b3Quat rotation =
			b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 0.3f, 1.0f, 0.7f } ), 0.37f * i );
		bodyIds[i] = CreateDynamicBody( worldId, position, rotation, true );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = densities[( i / 3 ) % 3];
		if ( i % 3 == 0 )
		{
			b3CreateHullShape( bodyIds[i], &shapeDef, &box.base );
		}
		else if ( i % 3 == 1 )
		{
			b3CreateSphereShape( bodyIds[i], &shapeDef, &sphere );
		}
		else
		{
			b3CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );
		}
	}

	b3FluidVolumeId extraId = b3_nullFluidVolumeId;
	SceneResult result = { B3_HASH_INIT, 0, 0 };
	for ( int step = 0; step < 260; ++step )
	{
		if ( step == 60 )
		{
			b3FluidVolume_SetRotation( channelId, -0.4f );
		}
		else if ( step == 90 )
		{
			b3FluidVolume_SetSurface( poolId, (b3Plane){ { -1.0f, 0.0f, 0.0f }, 2.0f } );
		}
		else if ( step == 120 )
		{
			b3FluidVolume_SetBounds( channelId,
									 (b3AABB){ { -10.0f, -3.0f, -2.5f }, { 10.0f, 0.0f, 2.5f } } );
			b3FluidVolume_SetSurface( poolId, poolDef.surface );
		}
		else if ( step == 150 )
		{
			b3FluidVolume_SetRotation( poolId, 0.0f );
		}
		else if ( step == 170 )
		{
			extraId = b3CreateFluidVolume( worldId, &extraDef );
		}
		else if ( step == 210 )
		{
			b3DestroyFluidVolume( extraId );
		}

		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3FluidEvents events = b3World_GetFluidEvents( worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			result.hash = HashBeginEvent( result.hash, events.beginEvents + i );
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			result.hash = HashEndEvent( result.hash, events.endEvents + i );
		}

		result.beginCount += events.beginCount;
		result.endCount += events.endCount;
	}

	for ( int i = 0; i < TURNED_BODY_COUNT; ++i )
	{
		result.hash = HashBody( result.hash, bodyIds[i] );
	}

	b3DestroyWorld( worldId );
	return result;
}

static int TurnedWorkerDeterminism( void )
{
	SceneResult reference = RunTurnedScene( 1 );
	ENSURE( reference.beginCount > TURNED_BODY_COUNT / 4 && reference.endCount > 0 );

	for ( int workerCount = 2; workerCount <= 5; ++workerCount )
	{
		SceneResult result = RunTurnedScene( workerCount );
		if ( result.hash != reference.hash )
		{
			printf( "  workers=%d hash=0x%08X reference=0x%08X\n", workerCount, result.hash,
					reference.hash );
		}

		ENSURE( result.hash == reference.hash );
		ENSURE( result.beginCount == reference.beginCount &&
				result.endCount == reference.endCount );
	}

	return 0;
}

int FluidTest( void )
{
	RUN_SUBTEST( BoxEquilibrium );
	RUN_SUBTEST( AxisFrames );
	RUN_SUBTEST( GravityScale );
	RUN_SUBTEST( SphereEquilibrium );
	RUN_SUBTEST( CapsuleEquilibrium );
	RUN_SUBTEST( DisplacedVolumeOverride );
	RUN_SUBTEST( PlankFloatsFlat );
	RUN_SUBTEST( BottomHeavyRights );
	RUN_SUBTEST( CapsuleVolumeContinuity );
	RUN_SUBTEST( HeaveStability );
	RUN_SUBTEST( RollStability );
	RUN_SUBTEST( PitchStability );
	RUN_SUBTEST( DefaultDragSleeps );
	RUN_SUBTEST( Sinking );
	RUN_SUBTEST( CurrentSpeed );
	RUN_SUBTEST( TerminalSpeed );
	RUN_SUBTEST( QuadraticDecay );
	RUN_SUBTEST( AngularDrag );
	RUN_SUBTEST( IneligibleBodies );
	RUN_SUBTEST( HullCornerGuard );
	RUN_SUBTEST( TwoSpheresOneDry );
	RUN_SUBTEST( EnterExitEvents );
	RUN_SUBTEST( VolumeSwitchEvents );
	RUN_SUBTEST( LifecycleEvents );
	RUN_SUBTEST( NoRepeatedBegins );
	RUN_SUBTEST( BeginPayload );
	RUN_SUBTEST( DefsBeforeFirstVolume );
	RUN_SUBTEST( DefsSurviveVolumeDestroy );
	RUN_SUBTEST( BatchSetter );
	RUN_SUBTEST( WakeOnDensity );
	RUN_SUBTEST( SubmergedSleeperStaysAsleep );
	RUN_SUBTEST( OverlapSleeperWake );
	RUN_SUBTEST( WakeOnCreate );
	RUN_SUBTEST( WakeOnBoundsShrink );
	RUN_SUBTEST( WakeOnBoundsGrow );
	RUN_SUBTEST( WakeOnDisplacedVolume );
	RUN_SUBTEST( WakeOnReenable );
	RUN_SUBTEST( OutsideTankWall );
	RUN_SUBTEST( AbuttingVolumes );
	RUN_SUBTEST( UnderTheFloor );
	RUN_SUBTEST( PointQuery );
	RUN_SUBTEST( VolumeApi );
	RUN_SUBTEST( DampingRestored );
	RUN_SUBTEST( DistantVolumeBitIdentical );
	RUN_SUBTEST( WorkerDeterminism );
	RUN_SUBTEST( EventStreamDeterminism );
	RUN_SUBTEST( LookupMatchesScan );
	RUN_SUBTEST( DegenerateFootprints );
	RUN_SUBTEST( ManyVolumesBodies );
	RUN_SUBTEST( RotatedChannel );
	RUN_SUBTEST( RotationZeroIdentity );
	RUN_SUBTEST( WakeOnRotation );
	RUN_SUBTEST( RotationApi );
	RUN_SUBTEST( RotatedOffAxes );
	RUN_SUBTEST( RotatedDraft );
	RUN_SUBTEST( TurnedDegenerateFootprints );
	RUN_SUBTEST( TurnedWorkerDeterminism );
	return 0;
}
