// Tunneling regression: fast bodies dropped onto static ground.
//
// Reported symptom (MineMogul): bodies with hull or box colliders sometimes pass straight
// through the ground when they fall from a great height. Spheres and capsules do not.
//
// That asymmetry is the interesting part. Continuous collision runs the same code for every
// shape, so if rounded shapes are immune the difference has to come from the proxy radius:
//
//   b3MakeShapeProxy gives a sphere/capsule its real radius, but gives a hull radius 0.
//   b3TimeOfImpact then bails out with state=Overlapped, fraction=0 as soon as the CORE
//   shapes touch. For a sphere the core is a point, so "core overlap" means the centre is
//   already a full radius inside the ground. For a hull the core IS the hull, so core
//   overlap means the surfaces merely grazed.
//
//   b3ContinuousQueryCallback only accepts a hit when 0 < fraction, so fraction=0 is thrown
//   away and the body advances the whole step - straight through thin ground.
//
// This program measures the end result: how often each shape ends up under the floor.

#include <box3d/box3d.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TIME_STEP ( 1.0f / 50.0f ) // MineMogul's fixed timestep
#define SUB_STEPS 2				   // MineMogul's B3World.subStepCount
#define MAX_STEPS 400

// ------------------------------------------------------------------ deterministic randomness

static uint32_t s_seed;

static void SeedRandom( uint32_t seed )
{
	s_seed = seed != 0 ? seed : 1u;
}

static float RandomUnit( void )
{
	s_seed = s_seed * 1664525u + 1013904223u;
	return (float)( ( s_seed >> 8 ) & 0xFFFFFFu ) / (float)0x1000000u;
}

static float RandomRange( float lo, float hi )
{
	return lo + ( hi - lo ) * RandomUnit();
}

// Uniform random orientation (Shoemake).
static b3Quat RandomQuat( void )
{
	float u1 = RandomUnit();
	float u2 = RandomUnit();
	float u3 = RandomUnit();
	float s1 = sqrtf( 1.0f - u1 );
	float s2 = sqrtf( u1 );
	const float tau = 6.28318531f;
	b3Quat q;
	q.v = (b3Vec3){ s1 * sinf( tau * u2 ), s1 * cosf( tau * u2 ), s2 * sinf( tau * u3 ) };
	q.s = s2 * cosf( tau * u3 );
	return q;
}

// ------------------------------------------------------------------ ground

typedef enum
{
	GroundHullThick, // 1 m thick box hull, top at y = 0
	GroundHullThin,	 // 10 cm thick box hull - a conveyor deck or a floor panel
	GroundMesh,		 // closed box mesh, the Unity MeshCollider case
	GroundMeshThin,	 // 10 cm thick closed box mesh - the case a thick mesh hides
	GroundCount,
} GroundKind;

static const char* s_groundNames[GroundCount] = { "hull 1.00 m", "hull 0.10 m", "mesh 1.00 m", "mesh 0.10 m" };
static const float s_groundHalfY[GroundCount] = { 0.5f, 0.05f, 0.5f, 0.05f };

static bool IsMeshGround( GroundKind kind )
{
	return kind == GroundMesh || kind == GroundMeshThin;
}

// Returns the mesh that must be destroyed after the world, or NULL.
static b3MeshData* BuildGround( b3WorldId worldId, GroundKind kind )
{
	float halfY = s_groundHalfY[kind];

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.5f;

	if ( IsMeshGround( kind ) )
	{
		b3MeshData* mesh = b3CreateBoxMesh( (b3Vec3){ 0.0f, -halfY, 0.0f }, (b3Vec3){ 40.0f, halfY, 40.0f }, true );
		b3CreateMeshShape( groundId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
		return mesh;
	}

	b3BoxHull slab = b3MakeOffsetBoxHull( 40.0f, halfY, 40.0f, (b3Vec3){ 0.0f, -halfY, 0.0f } );
	b3CreateHullShape( groundId, &shapeDef, &slab.base );
	return NULL;
}

// ------------------------------------------------------------------ falling items

typedef enum
{
	ItemBoxIngot, // the real IronIngot collider
	ItemBoxCube,
	ItemBoxPlate, // thin: the worst case for a zero-radius proxy
	ItemHullBlob, // a generic convex hull, like a cooked mesh collider
	ItemHullCylinder,
	ItemSphere,
	ItemCapsule,
	ItemCount,
} ItemKind;

static const char* s_itemNames[ItemCount] = {
	"box 221x95x478 (ingot)", "box 500 cube", "box 600x40x600 (plate)", "hull blob r=0.25",
	"hull cylinder r=0.15",	  "sphere r=0.15", "capsule r=0.10 l=0.30",
};

static bool IsRounded( ItemKind kind )
{
	return kind == ItemSphere || kind == ItemCapsule;
}

// An irregular convex point cloud, the sort of thing b3CreateHull sees from a cooked mesh.
static const b3Vec3 s_blobPoints[12] = {
	{ 0.250f, 0.060f, 0.040f },	  { -0.230f, 0.080f, 0.070f }, { 0.040f, 0.210f, -0.060f }, { 0.030f, -0.190f, 0.090f },
	{ 0.110f, 0.050f, 0.240f },	  { -0.070f, 0.030f, -0.220f }, { 0.180f, -0.130f, -0.110f }, { -0.160f, -0.120f, 0.140f },
	{ -0.150f, 0.140f, -0.130f }, { 0.140f, 0.150f, 0.130f },  { -0.040f, -0.200f, -0.050f }, { 0.200f, -0.050f, -0.180f },
};

// Returns a hull that must be destroyed by the caller, or NULL for the non-hull cases.
static b3HullData* CreateItemShape( b3BodyId bodyId, ItemKind kind, const b3ShapeDef* shapeDef )
{
	switch ( kind )
	{
		case ItemBoxIngot:
		{
			b3BoxHull box = b3MakeBoxHull( 0.11030f, 0.04723f, 0.23919f );
			b3CreateHullShape( bodyId, shapeDef, &box.base );
			return NULL;
		}

		case ItemBoxCube:
		{
			b3BoxHull box = b3MakeCubeHull( 0.25f );
			b3CreateHullShape( bodyId, shapeDef, &box.base );
			return NULL;
		}

		case ItemBoxPlate:
		{
			b3BoxHull box = b3MakeBoxHull( 0.30f, 0.02f, 0.30f );
			b3CreateHullShape( bodyId, shapeDef, &box.base );
			return NULL;
		}

		case ItemHullBlob:
		{
			b3HullData* hull = b3CreateHull( s_blobPoints, 12, 12 );
			b3CreateHullShape( bodyId, shapeDef, hull );
			return hull;
		}

		case ItemHullCylinder:
		{
			b3HullData* hull = b3CreateCylinder( 0.40f, 0.15f, 0.0f, 12 );
			b3CreateHullShape( bodyId, shapeDef, hull );
			return hull;
		}

		case ItemSphere:
		{
			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.15f };
			b3CreateSphereShape( bodyId, shapeDef, &sphere );
			return NULL;
		}

		case ItemCapsule:
		{
			b3Capsule capsule = { { 0.0f, -0.15f, 0.0f }, { 0.0f, 0.15f, 0.0f }, 0.10f };
			b3CreateCapsuleShape( bodyId, shapeDef, &capsule );
			return NULL;
		}

		default:
			return NULL;
	}
}

// ------------------------------------------------------------------ one drop

typedef struct
{
	bool tunneled;	   // finished below the ground
	float deepestY;	   // lowest body centre seen
	float impactSpeed; // downward speed on the step it first reached the deck
} DropResult;

static DropResult RunDrop( GroundKind ground, ItemKind item, float speed, b3Quat rotation, b3Vec3 spin )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	worldDef.enableContinuous = true;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3MeshData* mesh = BuildGround( worldId, ground );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 3.0f, 0.0f };
	bodyDef.rotation = rotation;
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -speed, 0.0f };
	bodyDef.angularVelocity = spin;
	bodyDef.linearDamping = 0.2f;  // MineMogul B3Body defaults
	bodyDef.angularDamping = 0.05f;
	b3BodyId itemId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 2000.0f;
	shapeDef.baseMaterial.friction = 0.5f;
	b3HullData* hull = CreateItemShape( itemId, item, &shapeDef );

	float groundBottom = -s_groundHalfY[ground];

	DropResult result = { 0 };
	result.deepestY = 1000.0f;
	result.impactSpeed = speed;

	for ( int step = 0; step < MAX_STEPS; ++step )
	{
		b3Vec3 v = b3Body_GetLinearVelocity( itemId );
		b3Pos before = b3Body_GetPosition( itemId );

		b3World_Step( worldId, TIME_STEP, SUB_STEPS );

		b3Pos p = b3Body_GetPosition( itemId );
		float y = (float)p.y;

		if ( (float)before.y > 0.0f && y <= 0.0f )
		{
			result.impactSpeed = -v.y;
		}

		if ( y < result.deepestY )
		{
			result.deepestY = y;
		}

		if ( y < groundBottom - 5.0f )
		{
			break;
		}
	}

	b3Pos p = b3Body_GetPosition( itemId );
	result.tunneled = (float)p.y < groundBottom - 0.25f;

	if ( hull != NULL )
	{
		b3DestroyHull( hull );
	}

	b3DestroyWorld( worldId );

	if ( mesh != NULL )
	{
		b3DestroyMesh( mesh );
	}

	return result;
}

// ------------------------------------------------------------------ main

#define TRIALS_PER_SPEED 24

int main( void )
{
	static const float speeds[] = { 10.0f, 20.0f, 30.0f, 40.0f, 60.0f, 80.0f, 100.0f };
	const int speedCount = (int)( sizeof( speeds ) / sizeof( speeds[0] ) );

	printf( "tunneling: fast bodies dropped onto static ground\n" );
	printf( "dt %.3f with %d substeps, continuous collision ON, non-bullet bodies\n", TIME_STEP, SUB_STEPS );
	printf( "%d orientations x %d speeds per row, tunneled = finished below the ground\n\n", TRIALS_PER_SPEED, speedCount );

	int totalTunneled = 0;
	int totalRuns = 0;
	int roundedTunneled = 0;
	int convexTunneled = 0;

	for ( int g = 0; g < GroundCount; ++g )
	{
		printf( "ground: %s\n", s_groundNames[g] );
		printf( "  %-24s", "item" );
		for ( int s = 0; s < speedCount; ++s )
		{
			printf( " %5.0f", speeds[s] );
		}
		printf( "   total\n" );

		for ( int i = 0; i < ItemCount; ++i )
		{
			printf( "  %-24s", s_itemNames[i] );

			int itemTunneled = 0;

			for ( int s = 0; s < speedCount; ++s )
			{
				// Same orientation sequence for every item and ground, so rows compare directly.
				SeedRandom( 0x9E3779B9u ^ (uint32_t)( s * 7919 ) );

				int tunneled = 0;
				for ( int t = 0; t < TRIALS_PER_SPEED; ++t )
				{
					b3Quat rotation = t == 0 ? b3Quat_identity : RandomQuat();
					b3Vec3 spin = { RandomRange( -8.0f, 8.0f ), RandomRange( -8.0f, 8.0f ), RandomRange( -8.0f, 8.0f ) };
					if ( t == 0 )
					{
						spin = b3Vec3_zero;
					}

					DropResult r = RunDrop( (GroundKind)g, (ItemKind)i, speeds[s], rotation, spin );
					totalRuns += 1;
					if ( r.tunneled )
					{
						tunneled += 1;
					}
				}

				printf( " %5d", tunneled );
				itemTunneled += tunneled;
			}

			printf( "   %5d\n", itemTunneled );

			totalTunneled += itemTunneled;
			if ( IsRounded( (ItemKind)i ) )
			{
				roundedTunneled += itemTunneled;
			}
			else
			{
				convexTunneled += itemTunneled;
			}
		}

		printf( "\n" );
	}

	printf( "%d of %d drops tunneled\n", totalTunneled, totalRuns );
	printf( "  hull/box shapes : %d\n", convexTunneled );
	printf( "  sphere/capsule  : %d\n", roundedTunneled );
	printf( "\n%s\n", totalTunneled == 0 ? "PASS" : "FAIL" );

	return totalTunneled == 0 ? 0 : 1;
}
