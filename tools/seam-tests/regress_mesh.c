// Regression checks for the mesh contact path after adding grazing-contact rejection.
// The mesh code warns that a hull can tunnel if the time of impact lands on a concave edge
// ("flat box sliding down a ramp to a flat bottom"), so that case is checked directly, along
// with resting, blocking, containment and internal-edge sliding.
//
// Every result should be identical between a build with the grazing rule on and one with it off.

#include <box3d/box3d.h>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEPS 4
#define MAX_PROFILE 8

static b3WorldId MakeWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	return b3CreateWorld( &worldDef );
}

// A strip mesh following a (z, y) profile, extruded along x. Interior vertices are shared so
// segment joins are real internal edges.
static b3MeshData* CreateProfileMesh( const float* pz, const float* py, int count, float halfWidth )
{
	b3Vec3 vertices[2 * MAX_PROFILE];
	int32_t indices[6 * ( MAX_PROFILE - 1 )];

	for ( int i = 0; i < count; ++i )
	{
		vertices[2 * i + 0] = ( b3Vec3 ){ -halfWidth, py[i], pz[i] };
		vertices[2 * i + 1] = ( b3Vec3 ){ halfWidth, py[i], pz[i] };
	}

	int n = 0;
	for ( int i = 0; i < count - 1; ++i )
	{
		indices[n++] = 2 * i + 0;
		indices[n++] = 2 * i + 2;
		indices[n++] = 2 * i + 3;
		indices[n++] = 2 * i + 0;
		indices[n++] = 2 * i + 3;
		indices[n++] = 2 * i + 1;
	}

	b3MeshDef meshDef = { 0 };
	meshDef.vertices = vertices;
	meshDef.vertexCount = 2 * count;
	meshDef.indices = indices;
	meshDef.triangleCount = 2 * ( count - 1 );
	meshDef.identifyEdges = true;

	return b3CreateMesh( &meshDef, NULL, 0 );
}

static float ProfileHeight( const float* pz, const float* py, int count, float z )
{
	if ( z <= pz[0] )
	{
		return py[0];
	}
	for ( int i = 0; i < count - 1; ++i )
	{
		if ( z <= pz[i + 1] )
		{
			float t = ( z - pz[i] ) / ( pz[i + 1] - pz[i] );
			return py[i] + t * ( py[i + 1] - py[i] );
		}
	}
	return py[count - 1];
}

static b3BodyId CreateBoxItem( b3WorldId worldId, b3Pos position, float h, float friction )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = position;
	bodyDef.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = friction;
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
	return bodyId;
}

// The case the mesh code warns about: a box slides down a ramp onto a flat bottom. The join at
// the base is a concave edge, exactly where a dropped contact would let the box tunnel.
static void TestRampToFlat( float friction )
{
	// Steep enough that the box slides at both friction values tested, with a long run-out so it
	// never reaches the far edge of the strip.
	float pz[3] = { 0.0f, 6.0f, 60.0f };
	float py[3] = { 4.0f, 0.0f, 0.0f };

	b3WorldId worldId = MakeWorld();

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	groundShape.baseMaterial.friction = friction;
	b3MeshData* mesh = CreateProfileMesh( pz, py, 3, 3.0f );
	b3CreateMeshShape( groundId, &groundShape, mesh, b3Vec3_one );

	float h = 0.25f;
	b3BodyId boxId = CreateBoxItem( worldId, ( b3Pos ){ 0.0f, 4.0f + h + 0.05f, 0.3f }, h, friction );

	float worstPenetration = 0.0f;
	bool fellThrough = false;

	for ( int step = 0; step < 900; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );

		b3Pos p = b3Body_GetPosition( boxId );
		if ( (float)p.z > 55.0f )
		{
			break;
		}

		float surface = ProfileHeight( pz, py, 3, (float)p.z );
		float clearance = (float)p.y - surface - h;
		worstPenetration = fminf( worstPenetration, clearance );

		if ( clearance < -0.5f )
		{
			fellThrough = true;
			break;
		}
	}

	b3Pos p = b3Body_GetPosition( boxId );
	printf( "ramp->flat friction %.1f   final z %6.2f y %+.3f   worst penetration %+.4f   %s\n", friction, (float)p.z,
			(float)p.y, worstPenetration, fellThrough ? "FELL THROUGH" : ( (float)p.z > 6.5f ? "reached flat" : "stalled" ) );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
}

// Resting on a flat mesh: gap and jitter.
static void TestFlatRest( void )
{
	float pz[2] = { -10.0f, 10.0f };
	float py[2] = { 0.0f, 0.0f };

	b3WorldId worldId = MakeWorld();

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3MeshData* mesh = CreateProfileMesh( pz, py, 2, 5.0f );
	b3CreateMeshShape( groundId, &groundShape, mesh, b3Vec3_one );

	float h = 0.25f;
	b3BodyId boxId = CreateBoxItem( worldId, ( b3Pos ){ 0.0f, 1.0f, 0.0f }, h, 0.5f );

	float deepest = FLT_MAX;
	for ( int step = 0; step < 400; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		b3Pos p = b3Body_GetPosition( boxId );
		deepest = fminf( deepest, (float)p.y );
	}

	b3Pos p = b3Body_GetPosition( boxId );
	b3Vec3 v = b3Body_GetLinearVelocity( boxId );
	printf( "flat mesh rest             deepest %+.4f   resting gap %+.4f   residual speed %.4f\n", deepest - h,
			(float)p.y - h, b3Length( v ) );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
}

// A vertical mesh wall must still block. Grazing rejection must not turn walls into curtains.
static void TestWallBlocks( float speed )
{
	b3WorldId worldId = MakeWorld();

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	// Wall quad in the XY plane at z = 5, facing -z.
	b3Vec3 vertices[4] = {
		{ -5.0f, -1.0f, 5.0f },
		{ 5.0f, -1.0f, 5.0f },
		{ 5.0f, 6.0f, 5.0f },
		{ -5.0f, 6.0f, 5.0f },
	};
	int32_t indices[6] = { 0, 2, 1, 0, 3, 2 };

	b3MeshDef meshDef = { 0 };
	meshDef.vertices = vertices;
	meshDef.vertexCount = 4;
	meshDef.indices = indices;
	meshDef.triangleCount = 2;
	meshDef.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &groundShape, mesh, b3Vec3_one );

	float h = 0.25f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 2.0f, 0.0f };
	bodyDef.gravityScale = 0.0f;
	bodyDef.linearVelocity = ( b3Vec3 ){ 0.0f, 0.0f, speed };
	b3BodyId boxId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3CreateHullShape( boxId, &shapeDef, &box.base );

	float deepest = 0.0f;
	for ( int step = 0; step < 200; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		b3Pos p = b3Body_GetPosition( boxId );
		deepest = fmaxf( deepest, ( (float)p.z + h ) - 5.0f );
	}

	b3Pos p = b3Body_GetPosition( boxId );
	printf( "mesh wall at %5.1f m/s     deepest overlap %.4f   final z %6.2f   %s\n", speed, deepest, (float)p.z,
			(float)p.z < 5.0f ? "blocked" : "TUNNELED" );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
}

// Sliding across many internal edges of one mesh, driven like a belt. Any upward velocity while
// grounded is a ghost collision.
static void TestInternalEdgeSlide( float yawDeg )
{
	b3WorldId worldId = MakeWorld();

	float pz[MAX_PROFILE];
	float py[MAX_PROFILE];
	for ( int i = 0; i < MAX_PROFILE; ++i )
	{
		pz[i] = (float)i * 2.0f;
		py[i] = 0.0f;
	}

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	groundShape.baseMaterial.friction = 0.4f;
	groundShape.baseMaterial.tangentVelocity = ( b3Vec3 ){ 0.0f, 0.0f, 2.0f };
	b3MeshData* mesh = CreateProfileMesh( pz, py, MAX_PROFILE, 2.0f );
	b3CreateMeshShape( groundId, &groundShape, mesh, b3Vec3_one );

	float h = 0.25f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, h, 0.5f };
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, yawDeg * B3_PI / 180.0f );
	bodyDef.enableSleep = false;
	b3BodyId boxId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.4f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3CreateHullShape( boxId, &shapeDef, &box.base );

	int launches = 0;
	float maxUp = 0.0f;
	bool wasLaunched = false;
	float speedSum = 0.0f;
	int samples = 0;

	for ( int step = 0; step < 600; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		if ( step < 60 )
		{
			continue;
		}

		b3Pos p = b3Body_GetPosition( boxId );
		b3Vec3 v = b3Body_GetLinearVelocity( boxId );
		if ( (float)p.z > pz[MAX_PROFILE - 1] - 0.5f )
		{
			break;
		}

		speedSum += v.z;
		samples += 1;

		bool launched = v.y > 0.05f;
		if ( launched && wasLaunched == false )
		{
			launches += 1;
		}
		maxUp = fmaxf( maxUp, v.y );
		wasLaunched = launched;
	}

	printf( "internal edge slide yaw %2.0f  launches %d   max up %.3f   avg vz %.3f\n", yawDeg, launches, maxUp,
			samples > 0 ? speedSum / (float)samples : 0.0f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
}

// A closed mesh pit must contain a body dropped into it.
static void TestPitContains( void )
{
	b3WorldId worldId = MakeWorld();

	// Floor plus four walls, one mesh.
	b3Vec3 vertices[8] = {
		{ -1.0f, 0.0f, -1.0f }, { 1.0f, 0.0f, -1.0f }, { 1.0f, 0.0f, 1.0f },	{ -1.0f, 0.0f, 1.0f },
		{ -1.0f, 2.0f, -1.0f }, { 1.0f, 2.0f, -1.0f }, { 1.0f, 2.0f, 1.0f },	{ -1.0f, 2.0f, 1.0f },
	};
	int32_t indices[30] = {
		0, 2, 1, 0, 3, 2,		 // floor
		0, 1, 5, 0, 5, 4,		 // -z wall
		1, 2, 6, 1, 6, 5,		 // +x wall
		2, 3, 7, 2, 7, 6,		 // +z wall
		3, 0, 4, 3, 4, 7,		 // -x wall
	};

	b3MeshDef meshDef = { 0 };
	meshDef.vertices = vertices;
	meshDef.vertexCount = 8;
	meshDef.indices = indices;
	meshDef.triangleCount = 10;
	meshDef.identifyEdges = true;
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &groundShape, mesh, b3Vec3_one );

	float h = 0.25f;
	// Dropped off-centre and spinning so it works the walls and the concave floor/wall joins.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.4f, 1.6f, -0.35f };
	bodyDef.linearVelocity = ( b3Vec3 ){ 2.0f, 0.0f, -1.5f };
	bodyDef.angularVelocity = ( b3Vec3 ){ 3.0f, 2.0f, 1.0f };
	b3BodyId boxId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.5f;
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3CreateHullShape( boxId, &shapeDef, &box.base );

	bool escaped = false;
	for ( int step = 0; step < 600; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		b3Pos p = b3Body_GetPosition( boxId );
		if ( (float)p.y < -0.5f || fabsf( (float)p.x ) > 1.5f || fabsf( (float)p.z ) > 1.5f )
		{
			escaped = true;
			break;
		}
	}

	b3Pos p = b3Body_GetPosition( boxId );
	printf( "pit containment            final (%+.2f %+.2f %+.2f)   %s\n", (float)p.x, (float)p.y, (float)p.z,
			escaped ? "ESCAPED" : "contained" );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
}

int main( void )
{
	printf( "box3d mesh-contact regression checks\n\n" );

	TestRampToFlat( 0.1f );
	TestRampToFlat( 0.5f );
	printf( "\n" );

	TestFlatRest();
	printf( "\n" );

	TestWallBlocks( 5.0f );
	TestWallBlocks( 20.0f );
	TestWallBlocks( 50.0f );
	printf( "\n" );

	TestInternalEdgeSlide( 0.0f );
	TestInternalEdgeSlide( 17.0f );
	TestInternalEdgeSlide( 45.0f );
	printf( "\n" );

	TestPitContains();

	return 0;
}
