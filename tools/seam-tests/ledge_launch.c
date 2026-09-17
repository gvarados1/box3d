// Repro for long items being launched when they ride off a triangle-mesh ledge.
//
// MineMogul's RollerLedge collision profile (measured from its FBX) is extruded into a
// triangle mesh: flat deck -> three transition facets -> 31 deg ramp that is a 7.6 cm slab with
// a sloped underside -> 8.9 cm drop face. Painted top faces carry the belt tangent velocity and
// the game's rate-limited belt velocity write is emulated. Items (game sizes, 1 kg) start on the
// deck over a sweep of lateral offsets and yaws. A control replaces the mesh with one convex slab
// hull per top segment, which is the box-collider workaround used in the game.
//
// Per run: peak upward velocity and spin while near the drop edge, and the most negative contact
// separation handed to the solver (with its push direction and triangle). A launch is a number.
//
// Mechanism (see triangle_manifold.c): the instant the item's center crosses the drop face's
// plane that triangle stops being back-face culled. For hulls, SAT's hull-face axis (the right
// answer, ~0) is rejected by the "pushingDown" guard because it is >104 deg from the triangle
// normal, and the triangle-face axis is used with the item's rear corners 0.3 m behind the drop
// face's INFINITE plane. For capsules, the shallow branch takes the face path (closest direction
// within ~78 deg of the face normal) and clips the axis to the face, reporting plane distances for
// the rear of the rod. Either way the solver gets -0.25..-0.35 m of "penetration" pointing +z and
// pushes it out at contactSpeed. The rear of the item is in front of the adjacent ramp triangle
// across a convex edge, i.e. outside the solid, which is what a fix can test for.
//
//   ledge_launch.exe          summary sweep
//   ledge_launch.exe -v       also trace the worst mesh run per shape, step by step
//
// Expected before a fix: every rod/plate/ingot mesh run launches (25/25), spheres never, the
// slab-hull control never. Expected after: 0 launches everywhere.

#include <box3d/box3d.h>

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DT 0.02f			 // Unity fixed step
#define BELT_SPEED 1.1f		 // RollerLedge Speed
#define ASSIST_ACCEL 60.0f	 // RollerLedge B3AssistAcceleration (RetainYVelocity = true)
#define HALF_WIDTH 0.472f	 // rails at x = +/-0.47
#define EDGE_Z 0.5f
#define STEPS 250			 // 5 s
#define MAX_CONTACTS 32

// RollerLedge profile in (z, y), traversed back -> front over the top, then around. The deck is
// extended backward so every item starts fully on it.
typedef struct
{
	float z, y;
} Point2;

static const Point2 s_profile[] = {
	{ -2.000f, 0.880f }, // P0 deck start
	{ -0.226f, 0.883f }, // P1 deck end
	{ -0.130f, 0.894f }, // P2 facet 6.7 deg
	{ -0.030f, 0.922f }, // P3 facet 15.5 deg
	{ 0.067f, 0.965f },	 // P4 facet 24.6 deg -> ramp start
	{ 0.500f, 1.226f },	 // P5 ramp end / drop edge
	{ 0.500f, 1.137f },	 // P6 bottom of the 8.9 cm drop face
	{ -0.068f, 0.795f }, // P7 underside meets the flat bottom
	{ -2.000f, 0.795f }, // P8 bottom back corner
};
#define PROFILE_COUNT ( (int)( sizeof( s_profile ) / sizeof( s_profile[0] ) ) )
#define TOP_SEGMENTS 5 // P0..P5 are the belt surface

typedef enum
{
	shape_rod,
	shape_plate,
	shape_ingot,
	shape_sphere,
	shape_count
} ItemShape;

static const char* s_shapeNames[shape_count] = { "Rod", "Plate", "Ingot", "Sphere" };

typedef struct
{
	const char* name;
	bool mesh;
	float contactSpeed;
	bool assist;
	int subSteps;
} Config;

typedef struct
{
	float maxVy;
	int maxVyStep;
	float maxSpin;
	float minSep;
	int minSepStep;
	b3Vec3 minSepPush;
	int minSepTri;
	float startX, yawDeg;
	bool launched;
} Result;

static b3Vec3 BoxPoint( b3Vec3 center, float hx, float hy, float hz, int i )
{
	float sx = ( i & 4 ) ? 1.0f : -1.0f;
	float sy = ( i & 2 ) ? 1.0f : -1.0f;
	float sz = ( i & 1 ) ? 1.0f : -1.0f;
	return ( b3Vec3 ){ center.x + hx * sx, center.y + hy * sy, center.z + hz * sz };
}

static b3HullData* MakeBoxHull( b3Vec3 center, float hx, float hy, float hz )
{
	b3Vec3 points[8];
	for ( int i = 0; i < 8; ++i )
	{
		points[i] = BoxPoint( center, hx, hy, hz, i );
	}
	return b3CreateHull( points, 8, 32 );
}

static Result Run( ItemShape shape, const Config* cfg, float startX, float yawDeg, bool trace )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	worldDef.contactHertz = 30.0f;
	worldDef.contactDampingRatio = 3.0f;
	worldDef.contactSpeed = cfg->contactSpeed;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Landing floor: big hull box, top at y = 0.
	{
		b3HullData* hull = MakeBoxHull( ( b3Vec3 ){ 0.0f, -0.5f, 0.0f }, 30.0f, 0.5f, 30.0f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId floorId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.6f;
		b3CreateHullShape( floorId, &shapeDef, hull );
		b3DestroyHull( hull );
	}

	// Ledge. Materials: 0 = inert frame (ConveyorFrame), 1..TOP_SEGMENTS = painted belt faces
	// (ConveyorBeltHighRollingResistance: friction 0.8, rolling 0.6) with the tangent velocity
	// along each segment's own slope.
	b3SurfaceMaterial materials[TOP_SEGMENTS + 1];
	materials[0] = b3DefaultSurfaceMaterial();
	materials[0].friction = 0.05f;
	for ( int s = 0; s < TOP_SEGMENTS; ++s )
	{
		float dz = s_profile[s + 1].z - s_profile[s].z;
		float dy = s_profile[s + 1].y - s_profile[s].y;
		float len = sqrtf( dz * dz + dy * dy );
		materials[s + 1] = b3DefaultSurfaceMaterial();
		materials[s + 1].friction = 0.8f;
		materials[s + 1].rollingResistance = 0.6f;
		materials[s + 1].tangentVelocity = ( b3Vec3 ){ 0.0f, BELT_SPEED * dy / len, BELT_SPEED * dz / len };
	}

	b3MeshData* meshData = NULL;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId ledgeId = b3CreateBody( worldId, &bodyDef );

		if ( cfg->mesh )
		{
			b3Vec3 vertices[2 * PROFILE_COUNT];
			int32_t indices[6 * PROFILE_COUNT];
			uint8_t materialIndices[2 * PROFILE_COUNT];

			for ( int i = 0; i < PROFILE_COUNT; ++i )
			{
				vertices[2 * i] = ( b3Vec3 ){ -HALF_WIDTH, s_profile[i].y, s_profile[i].z };
				vertices[2 * i + 1] = ( b3Vec3 ){ HALF_WIDTH, s_profile[i].y, s_profile[i].z };
			}

			for ( int i = 0; i < PROFILE_COUNT; ++i )
			{
				int a = i, b = ( i + 1 ) % PROFILE_COUNT;
				int aL = 2 * a, aR = 2 * a + 1, bL = 2 * b, bR = 2 * b + 1;

				// The profile runs clockwise (top back->front, then around), so the outward
				// normal is the edge direction rotated: (-dy, dz).
				float dz = s_profile[b].z - s_profile[a].z;
				float dy = s_profile[b].y - s_profile[a].y;
				b3Vec3 want = { 0.0f, dz, -dy };

				// Candidate winding (aL, bL, bR) has normal cross(bL - aL, bR - aL).
				b3Vec3 e1 = b3Sub( vertices[bL], vertices[aL] );
				b3Vec3 e2 = b3Sub( vertices[bR], vertices[aL] );
				bool flip = b3Dot( b3Cross( e1, e2 ), want ) < 0.0f;

				int t = 6 * i;
				if ( flip == false )
				{
					indices[t + 0] = aL; indices[t + 1] = bL; indices[t + 2] = bR;
					indices[t + 3] = aL; indices[t + 4] = bR; indices[t + 5] = aR;
				}
				else
				{
					indices[t + 0] = aL; indices[t + 1] = bR; indices[t + 2] = bL;
					indices[t + 3] = aL; indices[t + 4] = aR; indices[t + 5] = bR;
				}

				uint8_t material = (uint8_t)( i < TOP_SEGMENTS ? i + 1 : 0 );
				materialIndices[2 * i] = material;
				materialIndices[2 * i + 1] = material;
			}

			b3MeshDef meshDef = { 0 };
			meshDef.vertices = vertices;
			meshDef.indices = indices;
			meshDef.materialIndices = materialIndices;
			meshDef.vertexCount = 2 * PROFILE_COUNT;
			meshDef.triangleCount = 2 * PROFILE_COUNT;
			meshDef.weldTolerance = 0.001f;
			meshDef.weldVertices = true;
			meshDef.identifyEdges = true;
			meshData = b3CreateMesh( &meshDef, NULL, 0 );
			if ( meshData == NULL )
			{
				// Seen once against a stale DLL whose b3MeshDef predated the stride field: the def
				// bytes were misread and every bake failed. Build the engine from the same tree.
				printf( "ledge mesh bake failed (header/DLL b3MeshDef mismatch?)\n" );
				exit( 1 );
			}

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.materials = materials;
			shapeDef.materialCount = TOP_SEGMENTS + 1;
			b3CreateMeshShape( ledgeId, &shapeDef, meshData, b3Vec3_one );
		}
		else
		{
			// One 10 cm slab hull under each top segment (the box-collider workaround).
			for ( int s = 0; s < TOP_SEGMENTS; ++s )
			{
				float az = s_profile[s].z, ay = s_profile[s].y;
				float bz = s_profile[s + 1].z, by = s_profile[s + 1].y;
				float dz = bz - az, dy = by - ay, len = sqrtf( dz * dz + dy * dy );
				float nz = -dy / len, ny = dz / len;
				const float thick = 0.1f;

				b3Vec3 points[8];
				int k = 0;
				for ( int side = 0; side < 2; ++side )
				{
					float x = side == 0 ? -HALF_WIDTH : HALF_WIDTH;
					points[k++] = ( b3Vec3 ){ x, ay, az };
					points[k++] = ( b3Vec3 ){ x, by, bz };
					points[k++] = ( b3Vec3 ){ x, ay - ny * thick, az - nz * thick };
					points[k++] = ( b3Vec3 ){ x, by - ny * thick, bz - nz * thick };
				}

				b3HullData* hull = b3CreateHull( points, 8, 32 );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				shapeDef.baseMaterial = materials[s + 1];
				b3CreateHullShape( ledgeId, &shapeDef, hull );
				b3DestroyHull( hull );
			}
		}
	}

	// Item: game values (1 kg via density, damping 0.2/0.05, Ingot material friction 0.35, rolling 0.05).
	float halfLen, restH;
	switch ( shape )
	{
		case shape_rod: halfLen = 0.41f; restH = 0.07f; break;
		case shape_plate: halfLen = 0.298f; restH = 0.035f; break;
		case shape_ingot: halfLen = 0.239f; restH = 0.047f; break;
		default: halfLen = 0.10f; restH = 0.10f; break;
	}

	b3BodyDef itemDef = b3DefaultBodyDef();
	itemDef.type = b3_dynamicBody;
	itemDef.linearDamping = 0.2f;
	itemDef.angularDamping = 0.05f;
	itemDef.position = ( b3Pos ){ startX, 0.882f + restH + 0.01f, -1.2f };
	// Yaw about +y, built by hand so this links against a release DLL (the header helper asserts).
	{
		float half = 0.5f * yawDeg * B3_PI / 180.0f;
		itemDef.rotation = ( b3Quat ){ { 0.0f, sinf( half ), 0.0f }, cosf( half ) };
	}
	b3BodyId itemId = b3CreateBody( worldId, &itemDef );

	b3ShapeDef itemShapeDef = b3DefaultShapeDef();
	itemShapeDef.baseMaterial.friction = 0.35f;
	itemShapeDef.baseMaterial.rollingResistance = 0.05f;
	b3ShapeId itemShapeId;
	switch ( shape )
	{
		case shape_rod:
		{
			const float r = 0.07f, h = 0.82f;
			itemShapeDef.density = 1.0f / ( B3_PI * r * r * ( h - 2.0f * r ) + 4.0f / 3.0f * B3_PI * r * r * r );
			b3Capsule capsule = { { 0.0f, 0.0f, -( h / 2.0f - r ) }, { 0.0f, 0.0f, h / 2.0f - r }, r };
			itemShapeId = b3CreateCapsuleShape( itemId, &itemShapeDef, &capsule );
			break;
		}
		case shape_plate:
		{
			itemShapeDef.density = 1.0f / ( 0.596f * 0.07f * 0.596f );
			b3HullData* hull = MakeBoxHull( b3Vec3_zero, 0.298f, 0.035f, 0.298f );
			itemShapeId = b3CreateHullShape( itemId, &itemShapeDef, hull );
			b3DestroyHull( hull );
			break;
		}
		case shape_ingot:
		{
			itemShapeDef.density = 1.0f / ( 0.2206f * 0.0945f * 0.4784f );
			b3HullData* hull = MakeBoxHull( b3Vec3_zero, 0.1103f, 0.04725f, 0.2392f );
			itemShapeId = b3CreateHullShape( itemId, &itemShapeDef, hull );
			b3DestroyHull( hull );
			break;
		}
		default:
		{
			const float r = 0.1f;
			itemShapeDef.density = 1.0f / ( 4.0f / 3.0f * B3_PI * r * r * r );
			b3Sphere sphere = { b3Vec3_zero, r };
			itemShapeId = b3CreateSphereShape( itemId, &itemShapeDef, &sphere );
			break;
		}
	}

	Result res = { 0 };
	res.minSep = FLT_MAX;
	res.startX = startX;
	res.yawDeg = yawDeg;

	b3ContactData contacts[MAX_CONTACTS];

	for ( int step = 0; step < STEPS; ++step )
	{
		b3Pos p = b3Body_GetPosition( itemId );

		// ConveyorBeltManager.ApplyAveragedAssist: rate-limited velocity write along the belt
		// axis (RetainY: horizontal only) while any part of the item is still over the deck.
		if ( cfg->assist && (float)p.z - halfLen < EDGE_Z && (float)p.y > 0.7f )
		{
			b3Vec3 v = b3Body_GetLinearVelocity( itemId );
			if ( v.z < BELT_SPEED )
			{
				v.z += fminf( ASSIST_ACCEL * DT, BELT_SPEED - v.z );
				b3Body_SetLinearVelocity( itemId, v );
			}
		}

		b3World_Step( worldId, DT, cfg->subSteps );

		p = b3Body_GetPosition( itemId );
		b3Vec3 v = b3Body_GetLinearVelocity( itemId );
		b3Vec3 w = b3Body_GetAngularVelocity( itemId );
		float spin = b3Length( w );
		bool nearEdge = (float)p.z > 0.2f && (float)p.z < 1.6f && (float)p.y > 0.7f;

		if ( nearEdge && v.y > res.maxVy )
		{
			res.maxVy = v.y;
			res.maxVyStep = step;
		}
		if ( nearEdge && spin > res.maxSpin )
		{
			res.maxSpin = spin;
		}

		// Contact separations handed to the solver this step.
		float stepMinSep = FLT_MAX;
		b3Vec3 stepPush = b3Vec3_zero;
		int stepTri = -1;
		int contactCount = b3Body_GetContactData( itemId, contacts, MAX_CONTACTS );
		for ( int c = 0; c < contactCount; ++c )
		{
			bool itemIsA = B3_ID_EQUALS( contacts[c].shapeIdA, itemShapeId );
			for ( int m = 0; m < contacts[c].manifoldCount; ++m )
			{
				const b3Manifold* manifold = contacts[c].manifolds + m;
				for ( int k = 0; k < manifold->pointCount; ++k )
				{
					const b3ManifoldPoint* mp = manifold->points + k;
					if ( mp->separation < stepMinSep )
					{
						stepMinSep = mp->separation;
						stepTri = mp->triangleIndex;
						// The normal points A->B: the push on the item is along it if the item is B.
						stepPush = itemIsA ? b3Neg( manifold->normal ) : manifold->normal;
					}
				}
			}
		}

		// Only ledge contacts count (the item is well above the floor there).
		if ( (float)p.y > 0.5f && stepMinSep < res.minSep )
		{
			res.minSep = stepMinSep;
			res.minSepStep = step;
			res.minSepPush = stepPush;
			res.minSepTri = stepTri;
		}

		if ( trace && step >= 30 && (float)p.z > 0.0f && (float)p.z < 1.2f )
		{
			printf( "  step %3d t=%5.2f  pos=(%6.3f,%6.3f,%6.3f)  vel=(%6.2f,%6.2f,%6.2f)  spin=%5.1f  minSep=%7.3f push=(%.2f,%.2f,%.2f) tri=%d\n",
					step, step * DT, (float)p.x, (float)p.y, (float)p.z, v.x, v.y, v.z, spin,
					stepMinSep == FLT_MAX ? 0.0f : stepMinSep, stepPush.x, stepPush.y, stepPush.z, stepTri );
		}
	}

	res.launched = res.maxVy > 1.0f || res.minSep < -0.05f;

	b3DestroyWorld( worldId );
	if ( meshData != NULL )
	{
		b3DestroyMesh( meshData );
	}
	return res;
}

int main( int argc, char** argv )
{
	bool verbose = argc > 1 && strcmp( argv[1], "-v" ) == 0;

	b3Version version = b3GetVersion();
	printf( "Box3D %d.%d.%d  dt=%.2f  belt=%.1f m/s  hertz 30, damping 3\n", version.major, version.minor, version.revision,
			DT, BELT_SPEED );
	printf( "Launch = upward velocity > 1.0 m/s near the edge (ramp exit alone gives ~0.57) or any ledge separation < -0.05 m.\n\n" );

	const Config configs[] = {
		{ "mesh  cs10  assist", true, 10.0f, true, 2 },
		{ "mesh  cs3   assist", true, 3.0f, true, 2 },
		{ "mesh  cs0.1 assist", true, 0.1f, true, 2 },
		{ "mesh  cs10  no-asst", true, 10.0f, false, 2 },
		{ "mesh  cs10  sub4", true, 10.0f, true, 4 },
		{ "SLABS cs10  assist", false, 10.0f, true, 2 },
	};
	const int configCount = (int)( sizeof( configs ) / sizeof( configs[0] ) );
	const float offsets[] = { -0.2f, -0.1f, 0.0f, 0.1f, 0.2f };
	const float yaws[] = { -12.0f, -6.0f, 0.0f, 6.0f, 12.0f };
	const int sweep = 25;

	int meshLaunches = 0;
	for ( int shape = 0; shape < shape_count; ++shape )
	{
		printf( "=== %s ===\n", s_shapeNames[shape] );
		printf( "%-20s %7s %8s %6s %-11s %8s %-20s %4s %-11s\n", "config", "launch", "worstVy", "spin", "@x/yaw", "minSep",
				"push(x,y,z)", "tri", "@x/yaw" );

		Result worstMesh = { 0 };
		for ( int c = 0; c < configCount; ++c )
		{
			const Config* cfg = configs + c;
			int launched = 0;
			Result worstVy = { 0 };
			Result worstSep = { 0 };
			worstSep.minSep = FLT_MAX;

			for ( int i = 0; i < 5; ++i )
			{
				for ( int j = 0; j < 5; ++j )
				{
					Result r = Run( (ItemShape)shape, cfg, offsets[i], yaws[j], false );
					if ( r.launched )
					{
						launched += 1;
					}
					if ( r.maxVy > worstVy.maxVy )
					{
						worstVy = r;
					}
					if ( r.minSep < worstSep.minSep )
					{
						worstSep = r;
					}
				}
			}

			if ( cfg->mesh )
			{
				meshLaunches += launched;
			}
			if ( c == 0 )
			{
				worstMesh = worstVy;
			}

			char at1[16], at2[16], push[32];
			snprintf( at1, sizeof( at1 ), "%+.1f/%+.0f", worstVy.startX, worstVy.yawDeg );
			snprintf( at2, sizeof( at2 ), "%+.1f/%+.0f", worstSep.startX, worstSep.yawDeg );
			snprintf( push, sizeof( push ), "(%.2f,%.2f,%.2f)", worstSep.minSepPush.x, worstSep.minSepPush.y, worstSep.minSepPush.z );
			printf( "%-20s %4d/%d %8.2f %6.1f %-11s %8.3f %-20s %4d %-11s\n", cfg->name, launched, sweep, worstVy.maxVy,
					worstVy.maxSpin, at1, worstSep.minSep, push, worstSep.minSepTri, at2 );
		}

		if ( verbose && worstMesh.launched )
		{
			printf( "--- trace: %s, x=%+.1f yaw=%+.0f, game settings (mesh cs10 assist) ---\n", s_shapeNames[shape],
					worstMesh.startX, worstMesh.yawDeg );
			Run( (ItemShape)shape, configs + 0, worstMesh.startX, worstMesh.yawDeg, true );
		}
		printf( "\n" );
	}

	if ( meshLaunches > 0 )
	{
		printf( "%d mesh-ledge launches across the sweep (expected 0 once the convex-edge neighbor rule is in).\n", meshLaunches );
	}
	else
	{
		printf( "No mesh-ledge launches.\n" );
	}
	return 0;
}
