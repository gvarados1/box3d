// Exercises the build/teardown cycle the ConveyorSeamGhost sample performs whenever a control
// changes: destroy every body, then destroy the meshes those bodies referenced, then rebuild.
// Meshes are not cloned by b3CreateMeshShape, so the ordering matters. Run against a validation
// build so misuse asserts instead of silently corrupting.

#include <box3d/box3d.h>

#include <stdio.h>

#define TILE_COUNT 10
#define TILE_LEN 1.0f
#define TILE_HALF_X 0.472f
#define TILE_HALF_Y 0.0795f

typedef struct
{
	b3BodyId bodies[TILE_COUNT + 1];
	int bodyCount;
	b3MeshData* meshes[TILE_COUNT];
	int meshCount;
	b3HullData* chamfer;
} Scene;

static b3MeshData* CreateStripMesh( float z0, float z1, int quadCount )
{
	b3Vec3 vertices[2 * ( TILE_COUNT + 1 )];
	int32_t indices[6 * TILE_COUNT];

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

// Mirrors ConveyorSeamGhost::Clear
static void Clear( Scene* scene )
{
	for ( int i = 0; i < scene->bodyCount; ++i )
	{
		b3DestroyBody( scene->bodies[i] );
	}
	scene->bodyCount = 0;

	for ( int i = 0; i < scene->meshCount; ++i )
	{
		b3DestroyMesh( scene->meshes[i] );
	}
	scene->meshCount = 0;

	if ( scene->chamfer != NULL )
	{
		b3DestroyHull( scene->chamfer );
		scene->chamfer = NULL;
	}
}

// Mirrors ConveyorSeamGhost::Build
static void Build( b3WorldId worldId, Scene* scene, int beltType, int itemType )
{
	Clear( scene );

	b3ShapeDef beltDef = b3DefaultShapeDef();
	beltDef.baseMaterial.friction = 0.4f;
	beltDef.baseMaterial.tangentVelocity = ( b3Vec3 ){ 0.0f, 0.0f, 2.0f };

	float halfRun = 0.5f * (float)TILE_COUNT * TILE_LEN;

	for ( int i = 0; i < TILE_COUNT; ++i )
	{
		bool useMesh = ( beltType == 2 ) || ( beltType == 3 ) || ( beltType == 4 && ( i == 4 || i == 5 ) );
		bool merged = ( beltType == 1 || beltType == 3 );

		if ( merged && i > 0 )
		{
			break;
		}

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		bodyDef.position = useMesh ? ( b3Pos ){ 0.0f, 0.0f, 0.0f }
								   : ( merged ? ( b3Pos ){ 0.0f, -TILE_HALF_Y, halfRun }
											  : ( b3Pos ){ 0.0f, -TILE_HALF_Y, ( (float)i + 0.5f ) * TILE_LEN } );
		b3BodyId tileId = b3CreateBody( worldId, &bodyDef );
		scene->bodies[scene->bodyCount++] = tileId;

		if ( useMesh )
		{
			float z0 = merged ? 0.0f : (float)i * TILE_LEN;
			float z1 = merged ? (float)TILE_COUNT * TILE_LEN : (float)( i + 1 ) * TILE_LEN;
			int quads = merged ? TILE_COUNT : 1;
			scene->meshes[scene->meshCount] = CreateStripMesh( z0, z1, quads );
			b3CreateMeshShape( tileId, &beltDef, scene->meshes[scene->meshCount], b3Vec3_one );
			scene->meshCount += 1;
		}
		else
		{
			b3BoxHull tile = b3MakeBoxHull( TILE_HALF_X, TILE_HALF_Y, merged ? halfRun : 0.5f * TILE_LEN );
			b3CreateHullShape( tileId, &beltDef, &tile.base );
		}
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, 0.3f, 0.5f };
	bodyDef.enableSleep = false;
	b3BodyId itemId = b3CreateBody( worldId, &bodyDef );
	scene->bodies[scene->bodyCount++] = itemId;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.4f;

	if ( itemType == 0 )
	{
		b3BoxHull box = b3MakeBoxHull( 0.3f, 0.3f, 0.3f );
		b3CreateHullShape( itemId, &shapeDef, &box.base );
	}
	else if ( itemType == 1 )
	{
		scene->chamfer = CreateChamferBoxHull( 0.3f, 0.05f );
		b3CreateHullShape( itemId, &shapeDef, scene->chamfer );
	}
	else
	{
		b3Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = 0.3f };
		b3CreateSphereShape( itemId, &shapeDef, &sphere );
	}
}

int main( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	Scene scene = { 0 };
	int cycles = 0;

	// Every belt type against every item type, rebuilt in place like the ImGui controls do,
	// stepping in between so contacts, islands and the SAT cache are live across the teardown.
	for ( int pass = 0; pass < 3; ++pass )
	{
		for ( int beltType = 0; beltType < 5; ++beltType )
		{
			for ( int itemType = 0; itemType < 3; ++itemType )
			{
				Build( worldId, &scene, beltType, itemType );
				for ( int step = 0; step < 40; ++step )
				{
					b3World_Step( worldId, 1.0f / 60.0f, 4 );
				}
				cycles += 1;
			}
		}
	}

	Clear( &scene );
	b3DestroyWorld( worldId );

	printf( "%d build/teardown cycles completed with no assert\n", cycles );
	return 0;
}
