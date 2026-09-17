// Ghost collisions and sticking where two flush colliders meet.
//
// Models a conveyor run built from tiles that each carry their own collider, butted together
// with coplanar top faces. An item is driven along +z by tangentVelocity and should glide across
// the seams. Without grazing-contact rejection it is instead launched upward, or catches on the
// leading face of the next tile and loses belt speed.
//
// Compare belt types: a single merged collider has no seam and never launches, which isolates
// the seam as the cause. Build with B3_GRAZING_FACE_ALIGNMENT=-1.0f and B3_CONVEX_REST_OFFSET=0
// to see the original behaviour.
//
// See tools/seam-tests/README.md for how to build and what the numbers should look like.

#include <box3d/box3d.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TILE_COUNT 10
#define TILE_LEN 1.0f		// belt tile length along travel (z)
#define TILE_HALF_X 0.472f	// from rollerstraight.prefab: size 0.944 x 0.159 x 1
#define TILE_HALF_Y 0.0795f
#define BELT_SPEED 2.0f
#define TIME_STEP ( 1.0f / 60.0f )
#define SUB_STEPS 4

typedef enum
{
	ITEM_BOX,
	ITEM_CHAMFER,
	ITEM_SPHERE,
} ItemType;

typedef enum
{
	BELT_SEPARATE_HULLS,
	BELT_MERGED_HULL,
	BELT_SEPARATE_MESHES,
	BELT_MERGED_MESH,
	BELT_MIXED,
} BeltType;

static const char* ItemName( ItemType t )
{
	switch ( t )
	{
		case ITEM_BOX: return "box";
		case ITEM_CHAMFER: return "chamfered box";
		case ITEM_SPHERE: return "sphere";
	}
	return "?";
}

static const char* BeltName( BeltType t )
{
	switch ( t )
	{
		case BELT_SEPARATE_HULLS: return "separate hulls";
		case BELT_MERGED_HULL: return "one merged hull";
		case BELT_SEPARATE_MESHES: return "separate meshes";
		case BELT_MERGED_MESH: return "one merged mesh";
		case BELT_MIXED: return "hulls + mesh corner";
	}
	return "?";
}

// Convex hull of the six inset face rectangles: a box with all twelve edges chamfered, which is
// what a crate collision mesh looks like once it has been run through hull generation. The
// chamfer is what grazes the next tile's top corner.
static b3HullData* CreateChamferBoxHull( float h, float c )
{
	b3Vec3 points[24];
	int n = 0;
	float i = h - c;

	for ( int s = -1; s <= 1; s += 2 )
	{
		float f = (float)s * h;

		points[n++] = ( b3Vec3 ){ f, i, i };
		points[n++] = ( b3Vec3 ){ f, i, -i };
		points[n++] = ( b3Vec3 ){ f, -i, i };
		points[n++] = ( b3Vec3 ){ f, -i, -i };

		points[n++] = ( b3Vec3 ){ i, f, i };
		points[n++] = ( b3Vec3 ){ i, f, -i };
		points[n++] = ( b3Vec3 ){ -i, f, i };
		points[n++] = ( b3Vec3 ){ -i, f, -i };

		points[n++] = ( b3Vec3 ){ i, i, f };
		points[n++] = ( b3Vec3 ){ i, -i, f };
		points[n++] = ( b3Vec3 ){ -i, i, f };
		points[n++] = ( b3Vec3 ){ -i, -i, f };
	}

	return b3CreateHull( points, n, 24 );
}

// Flat quad strip at y = 0 covering [z0, z1]. Interior vertices are shared, so a multi-quad
// strip has real internal edges for the mesh contact filter to work with.
static b3MeshData* CreateStripMesh( float z0, float z1, int quadCount )
{
	static b3Vec3 vertices[2 * ( TILE_COUNT + 1 )];
	static int32_t indices[6 * TILE_COUNT];

	float dz = ( z1 - z0 ) / (float)quadCount;
	for ( int i = 0; i <= quadCount; ++i )
	{
		float z = z0 + (float)i * dz;
		vertices[2 * i + 0] = ( b3Vec3 ){ -TILE_HALF_X, 0.0f, z };
		vertices[2 * i + 1] = ( b3Vec3 ){ TILE_HALF_X, 0.0f, z };
	}

	int n = 0;
	for ( int i = 0; i < quadCount; ++i )
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
	meshDef.vertexCount = 2 * ( quadCount + 1 );
	meshDef.indices = indices;
	meshDef.triangleCount = 2 * quadCount;
	meshDef.identifyEdges = true;

	return b3CreateMesh( &meshDef, NULL, 0 );
}

typedef struct
{
	int launchCount;
	float maxUpSpeed;
	float maxRise;
	float endZ;
	float avgSpeed;
} Result;

static Result RunScenario( ItemType itemType, BeltType beltType, bool verbose, float beltSpeed, float yawDeg )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3MeshData* meshes[TILE_COUNT] = { 0 };
	int meshCount = 0;

	b3ShapeDef beltShapeDef = b3DefaultShapeDef();
	beltShapeDef.baseMaterial.friction = 0.4f;
	beltShapeDef.baseMaterial.tangentVelocity = ( b3Vec3 ){ 0.0f, 0.0f, beltSpeed };

	float halfRun = 0.5f * (float)TILE_COUNT * TILE_LEN;

	if ( beltType == BELT_SEPARATE_HULLS )
	{
		for ( int i = 0; i < TILE_COUNT; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_staticBody;
			bodyDef.position = ( b3Pos ){ 0.0f, -TILE_HALF_Y, ( (float)i + 0.5f ) * TILE_LEN };
			b3BodyId tileId = b3CreateBody( worldId, &bodyDef );

			b3BoxHull tile = b3MakeBoxHull( TILE_HALF_X, TILE_HALF_Y, 0.5f * TILE_LEN );
			b3CreateHullShape( tileId, &beltShapeDef, &tile.base );
		}
	}
	else if ( beltType == BELT_MERGED_HULL )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		bodyDef.position = ( b3Pos ){ 0.0f, -TILE_HALF_Y, halfRun };
		b3BodyId beltId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull belt = b3MakeBoxHull( TILE_HALF_X, TILE_HALF_Y, halfRun );
		b3CreateHullShape( beltId, &beltShapeDef, &belt.base );
	}
	else if ( beltType == BELT_SEPARATE_MESHES )
	{
		for ( int i = 0; i < TILE_COUNT; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_staticBody;
			bodyDef.position = ( b3Pos ){ 0.0f, 0.0f, 0.0f };
			b3BodyId tileId = b3CreateBody( worldId, &bodyDef );

			meshes[meshCount] = CreateStripMesh( (float)i * TILE_LEN, (float)( i + 1 ) * TILE_LEN, 1 );
			b3CreateMeshShape( tileId, &beltShapeDef, meshes[meshCount], b3Vec3_one );
			meshCount += 1;
		}
	}
	else if ( beltType == BELT_MERGED_MESH )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		bodyDef.position = ( b3Pos ){ 0.0f, 0.0f, 0.0f };
		b3BodyId beltId = b3CreateBody( worldId, &bodyDef );

		meshes[meshCount] = CreateStripMesh( 0.0f, (float)TILE_COUNT * TILE_LEN, TILE_COUNT );
		b3CreateMeshShape( beltId, &beltShapeDef, meshes[meshCount], b3Vec3_one );
		meshCount += 1;
	}
	else
	{
		// Straight belts (box colliders, so hulls) feeding a corner piece (a non-convex mesh
		// collider, so a mesh shape) and out the other side. This is the real prefab mix.
		for ( int i = 0; i < TILE_COUNT; ++i )
		{
			bool isCorner = ( i == 4 || i == 5 );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_staticBody;
			bodyDef.position = isCorner ? ( b3Pos ){ 0.0f, 0.0f, 0.0f }
										: ( b3Pos ){ 0.0f, -TILE_HALF_Y, ( (float)i + 0.5f ) * TILE_LEN };
			b3BodyId tileId = b3CreateBody( worldId, &bodyDef );

			if ( isCorner )
			{
				meshes[meshCount] = CreateStripMesh( (float)i * TILE_LEN, (float)( i + 1 ) * TILE_LEN, 1 );
				b3CreateMeshShape( tileId, &beltShapeDef, meshes[meshCount], b3Vec3_one );
				meshCount += 1;
			}
			else
			{
				b3BoxHull tile = b3MakeBoxHull( TILE_HALF_X, TILE_HALF_Y, 0.5f * TILE_LEN );
				b3CreateHullShape( tileId, &beltShapeDef, &tile.base );
			}
		}
	}

	// Item
	float halfHeight = 0.3f;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, halfHeight, 0.5f };
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, yawDeg * B3_PI / 180.0f );
	bodyDef.enableSleep = false;
	b3BodyId itemId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.4f;

	b3HullData* chamfer = NULL;
	if ( itemType == ITEM_BOX )
	{
		b3BoxHull box = b3MakeBoxHull( 0.3f, halfHeight, 0.3f );
		b3CreateHullShape( itemId, &shapeDef, &box.base );
	}
	else if ( itemType == ITEM_CHAMFER )
	{
		chamfer = CreateChamferBoxHull( halfHeight, 0.05f );
		b3CreateHullShape( itemId, &shapeDef, chamfer );
	}
	else
	{
		b3Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = halfHeight };
		b3CreateSphereShape( itemId, &shapeDef, &sphere );
	}

	Result result = { 0 };
	float restY = halfHeight;
	bool wasLaunched = false;
	float speedSum = 0.0f;
	int speedSamples = 0;

	int settleSteps = 90;
	int maxSteps = 4000;

	for ( int step = 0; step < maxSteps; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );

		b3Pos p = b3Body_GetPosition( itemId );
		b3Vec3 v = b3Body_GetLinearVelocity( itemId );

		if ( step < settleSteps )
		{
			continue;
		}

		if ( (float)p.z > (float)TILE_COUNT * TILE_LEN - 0.5f )
		{
			break;
		}

		speedSum += v.z;
		speedSamples += 1;

		result.maxRise = fmaxf( result.maxRise, (float)p.y - restY );
		result.endZ = (float)p.z;

		// The belt surface is exactly y = 0, so a grounded item never rises above rest height on
		// its own. Any upward velocity while grounded is a ghost collision.
		bool launched = v.y > 0.05f;
		if ( launched && wasLaunched == false )
		{
			result.launchCount += 1;

			if ( verbose && result.launchCount <= 4 )
			{
				printf( "  launch %d at z=%.3f (nearest seam z=%.0f): vy=%.3f vz=%.3f y-rest=%+.4f\n", result.launchCount,
						(float)p.z, roundf( (float)p.z ), v.y, v.z, (float)p.y - restY );

				b3ContactData contacts[8];
				int count = b3Body_GetContactData( itemId, contacts, 8 );
				for ( int i = 0; i < count; ++i )
				{
					for ( int m = 0; m < contacts[i].manifoldCount; ++m )
					{
						const b3Manifold* manifold = contacts[i].manifolds + m;
						printf( "      normal (%+.3f %+.3f %+.3f) points %d seps", manifold->normal.x, manifold->normal.y,
								manifold->normal.z, manifold->pointCount );
						for ( int k = 0; k < manifold->pointCount; ++k )
						{
							printf( " %+.4f", manifold->points[k].separation );
						}
						printf( "\n" );
					}
				}
			}
		}
		result.maxUpSpeed = fmaxf( result.maxUpSpeed, v.y );
		wasLaunched = launched;
	}

	result.avgSpeed = speedSamples > 0 ? speedSum / (float)speedSamples : 0.0f;

	if ( chamfer != NULL )
	{
		b3DestroyHull( chamfer );
	}
	b3DestroyWorld( worldId );

	for ( int i = 0; i < meshCount; ++i )
	{
		b3DestroyMesh( meshes[i] );
	}

	return result;
}

int main( int argc, char** argv )
{
	bool verbose = argc > 1 && strcmp( argv[1], "-v" ) == 0;

	// Focused debug: one config, dumping the manifolds that launch it.
	if ( argc > 3 && strcmp( argv[1], "-c" ) == 0 )
	{
		float speed = (float)atof( argv[2] );
		float yaw = (float)atof( argv[3] );
		printf( "chamfered box, separate hulls, speed %.2f yaw %.0f\n", speed, yaw );
		Result r = RunScenario( ITEM_CHAMFER, BELT_SEPARATE_HULLS, true, speed, yaw );
		printf( "launches %d  max up %.3f  avg vz %.3f\n", r.launchCount, r.maxUpSpeed, r.avgSpeed );
		return 0;
	}

	printf( "box3d conveyor seam ghost repro\n" );
	printf( "%d tiles, %.1f m each, belt speed %.1f m/s, tops coplanar at y=0\n\n", TILE_COUNT, TILE_LEN, BELT_SPEED );

	ItemType itemTypes[3] = { ITEM_BOX, ITEM_CHAMFER, ITEM_SPHERE };
	BeltType beltTypes[5] = { BELT_SEPARATE_HULLS, BELT_MERGED_HULL, BELT_SEPARATE_MESHES, BELT_MERGED_MESH, BELT_MIXED };

	printf( "%-22s %-14s %9s %10s %9s %9s\n", "belt", "item", "launches", "max up", "max rise", "avg vz" );
	for ( int b = 0; b < 5; ++b )
	{
		for ( int i = 0; i < 3; ++i )
		{
			if ( verbose )
			{
				printf( "\n--- %s / %s ---\n", BeltName( beltTypes[b] ), ItemName( itemTypes[i] ) );
			}

			Result r = RunScenario( itemTypes[i], beltTypes[b], verbose, BELT_SPEED, 0.0f );
			printf( "%-22s %-14s %9d %9.3f %9.4f %9.3f\n", BeltName( beltTypes[b] ), ItemName( itemTypes[i] ), r.launchCount,
					r.maxUpSpeed, r.maxRise, r.avgSpeed );
		}
	}

	// Sweep speed and yaw on the seamed belt setups to be sure a result is not one lucky
	// alignment. Only failures are printed, so a clean run is a short run.
	printf( "\nswept over belt speed and item yaw, failures only\n" );

	float speeds[4] = { 0.5f, 1.0f, 2.0f, 4.0f };
	float yaws[3] = { 0.0f, 17.0f, 45.0f };
	BeltType sweptBelts[3] = { BELT_SEPARATE_HULLS, BELT_SEPARATE_MESHES, BELT_MIXED };

	int runs = 0;
	int failures = 0;
	for ( int b = 0; b < 3; ++b )
	{
		for ( int i = 0; i < 2; ++i )
		{
			for ( int s = 0; s < 4; ++s )
			{
				for ( int y = 0; y < 3; ++y )
				{
					Result r = RunScenario( itemTypes[i], sweptBelts[b], false, speeds[s], yaws[y] );
					runs += 1;

					// A clean pass means no launch and the item tracking the belt.
					bool bad = r.launchCount > 0 || r.avgSpeed < 0.9f * speeds[s];
					if ( bad )
					{
						failures += 1;
						printf( "  %-22s %-14s speed %.1f yaw %2.0f -> launches %d, max up %.3f m/s, rise %.1f mm, avg vz %.3f\n",
								BeltName( sweptBelts[b] ), ItemName( itemTypes[i] ), speeds[s], yaws[y], r.launchCount,
								r.maxUpSpeed, 1000.0f * r.maxRise, r.avgSpeed );
					}
				}
			}
		}
	}

	printf( "%d of %d swept configurations failed\n", failures, runs );

	return 0;
}
