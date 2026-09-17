// Regression checks for the convex contact path: the convex rest offset (B3_CONVEX_REST_OFFSET)
// and grazing-contact rejection (B3_GRAZING_FACE_ALIGNMENT) must not disturb ordinary
// simulation. Stacking stability, resting gap, drop penetration, ramp sliding, fast impact.
//
// Run this against a build with the rules on and a build with them off and compare. See
// tools/seam-tests/README.md.

#include <box3d/box3d.h>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEPS 4

static b3WorldId MakeWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	return b3CreateWorld( &worldDef );
}

static b3BodyId MakeGround( b3WorldId worldId )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull ground = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
	b3CreateHullShape( groundId, &shapeDef, &ground.base );
	return groundId;
}

// A vertical stack of boxes: does it stay standing, and how far does it drift or sag?
static void TestStack( int height )
{
	b3WorldId worldId = MakeWorld();
	MakeGround( worldId );

	float h = 0.25f;
	b3BodyId bodies[16];
	for ( int i = 0; i < height; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = ( b3Pos ){ 0.0f, h + (float)i * 2.0f * h, 0.0f };
		bodies[i] = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.6f;
		b3BoxHull box = b3MakeBoxHull( h, h, h );
		b3CreateHullShape( bodies[i], &shapeDef, &box.base );
	}

	for ( int step = 0; step < 300; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
	}

	float maxDrift = 0.0f;
	float topError = 0.0f;
	float maxSpeed = 0.0f;
	for ( int i = 0; i < height; ++i )
	{
		b3Pos p = b3Body_GetPosition( bodies[i] );
		b3Vec3 v = b3Body_GetLinearVelocity( bodies[i] );
		maxDrift = fmaxf( maxDrift, sqrtf( (float)( p.x * p.x + p.z * p.z ) ) );
		maxSpeed = fmaxf( maxSpeed, b3Length( v ) );
		if ( i == height - 1 )
		{
			topError = (float)p.y - ( h + (float)i * 2.0f * h );
		}
	}

	printf( "stack of %2d   drift %.4f m   top y error %+.4f m   max speed %.4f m/s   %s\n", height, maxDrift, topError,
			maxSpeed, maxDrift < 0.05f ? "standing" : "COLLAPSED" );

	b3DestroyWorld( worldId );
}

// Drop a body from a height: how deep does it penetrate, where does it come to rest?
static void TestDrop( float dropHeight, bool sphere )
{
	b3WorldId worldId = MakeWorld();
	MakeGround( worldId );

	float h = 0.25f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, dropHeight, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	if ( sphere )
	{
		b3Sphere s = { .center = { 0.0f, 0.0f, 0.0f }, .radius = h };
		b3CreateSphereShape( bodyId, &shapeDef, &s );
	}
	else
	{
		b3BoxHull box = b3MakeBoxHull( h, h, h );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	float minY = FLT_MAX;
	for ( int step = 0; step < 400; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		b3Pos p = b3Body_GetPosition( bodyId );
		minY = fminf( minY, (float)p.y );
	}

	b3Pos p = b3Body_GetPosition( bodyId );
	printf( "drop %s from %.1f m   deepest %+.4f   resting gap %+.4f m\n", sphere ? "sphere" : "box  ", dropHeight, minY - h,
			(float)p.y - h );

	b3DestroyWorld( worldId );
}

// A box on a ramp steeper than the friction angle should slide, not stick or jitter.
static void TestRamp( float angleDeg, float friction )
{
	b3WorldId worldId = MakeWorld();

	float angle = angleDeg * B3_PI / 180.0f;

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	groundDef.position = ( b3Pos ){ 0.0f, 0.0f, 0.0f };
	groundDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, angle );
	b3BodyId rampId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef rampShape = b3DefaultShapeDef();
	rampShape.baseMaterial.friction = friction;
	b3BoxHull ramp = b3MakeBoxHull( 5.0f, 0.5f, 20.0f );
	b3CreateHullShape( rampId, &rampShape, &ramp.base );

	float h = 0.25f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 0.5f / cosf( angle ) + h, 0.0f };
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, angle );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = friction;
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	float maxNormalSpeed = 0.0f;
	b3Vec3 rampUp = { 0.0f, cosf( angle ), -sinf( angle ) };
	for ( int step = 0; step < 300; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		if ( step > 30 )
		{
			b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
			maxNormalSpeed = fmaxf( maxNormalSpeed, fabsf( b3Dot( v, rampUp ) ) );
		}
	}

	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	printf( "ramp %.0f deg friction %.1f   slide speed %.3f m/s   off-surface speed %.4f m/s\n", angleDeg, friction,
			b3Length( v ), maxNormalSpeed );

	b3DestroyWorld( worldId );
}

// Fast horizontal impact into a static wall: does it stop, and how far does it get inside?
static void TestFastImpact( float speed )
{
	b3WorldId worldId = MakeWorld();

	b3BodyDef wallDef = b3DefaultBodyDef();
	wallDef.type = b3_staticBody;
	wallDef.position = ( b3Pos ){ 0.0f, 2.0f, 5.0f };
	b3BodyId wallId = b3CreateBody( worldId, &wallDef );

	b3ShapeDef wallShape = b3DefaultShapeDef();
	b3BoxHull wall = b3MakeBoxHull( 5.0f, 2.0f, 0.5f );
	b3CreateHullShape( wallId, &wallShape, &wall.base );

	float h = 0.25f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 2.0f, 0.0f };
	bodyDef.gravityScale = 0.0f;
	bodyDef.linearVelocity = ( b3Vec3 ){ 0.0f, 0.0f, speed };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	float deepest = 0.0f;
	for ( int step = 0; step < 200; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		b3Pos p = b3Body_GetPosition( bodyId );
		// wall front face is at z = 4.5
		float overlap = ( (float)p.z + h ) - 4.5f;
		deepest = fmaxf( deepest, overlap );
	}

	b3Pos p = b3Body_GetPosition( bodyId );
	printf( "impact at %5.1f m/s   deepest overlap %.4f m   final z %.3f   %s\n", speed, deepest, (float)p.z,
			(float)p.z < 4.5f ? "stopped" : "TUNNELED" );

	b3DestroyWorld( worldId );
}

int main( void )
{
	printf( "box3d convex-contact regression checks\n\n" );

	TestStack( 5 );
	TestStack( 10 );
	printf( "\n" );

	TestDrop( 1.0f, false );
	TestDrop( 5.0f, false );
	TestDrop( 1.0f, true );
	printf( "\n" );

	TestRamp( 10.0f, 0.6f );
	TestRamp( 40.0f, 0.3f );
	printf( "\n" );

	TestFastImpact( 5.0f );
	TestFastImpact( 20.0f );
	TestFastImpact( 50.0f );

	return 0;
}
