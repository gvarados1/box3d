// Thin-walled concave mesh container - the "minecart made from its render mesh" case.
//
// MineMogul has a static prop whose collider is a concave MeshCollider using the render mesh, and
// dynamic bodies dropped into it sometimes end up outside. A render mesh differs from the simple
// test grounds elsewhere in this harness in three ways that all matter here:
//
//   1. the walls have NO THICKNESS - a single layer of triangles, so there is no solid interior
//      to stop a body that gets past the surface
//   2. the walls are CONCAVE - a body in the corner touches several faces at steep angles at once,
//      which is exactly what the grazing rule in mesh_contact.c classifies and can drop
//   3. it is DENSE - a body's query bounds can cover many triangles, and mesh_contact.c caps a
//      single convex-vs-mesh pair at B3_MAX_MESH_CONTACT_TRIANGLES (256) and then silently
//      truncates, warning only once per process
//
// So this sweeps wall tessellation against impact speed and counts escapes, and captures Box3D's
// log so the triangle-cap warning is reported as data rather than missed in a console.
//
// Run it against both engine flavors (build.ps1 and build.ps1 -RulesOff) to see whether the
// grazing/rest-offset rules are involved at all.

#include <box3d/base.h>
#include <box3d/box3d.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP ( 1.0f / 50.0f )
#define SUB_STEPS 2
#define MAX_STEPS 400

// Roughly a minecart interior.
#define HX 0.45f
#define HY 0.30f
#define HZ 0.75f

// ------------------------------------------------------------------ log capture

static bool s_sawTriangleCap = false;

static void CaptureLog( const char* message )
{
	if ( strstr( message, "complex mesh" ) != NULL || strstr( message, "triangle buffer capacity" ) != NULL )
	{
		s_sawTriangleCap = true;
	}
}

// ------------------------------------------------------------------ deterministic randomness

static uint32_t s_seed = 1u;

static float RandomUnit( void )
{
	s_seed = s_seed * 1664525u + 1013904223u;
	return (float)( ( s_seed >> 8 ) & 0xFFFFFFu ) / (float)0x1000000u;
}

static float RandomRange( float lo, float hi )
{
	return lo + ( hi - lo ) * RandomUnit();
}

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

// ------------------------------------------------------------------ container mesh

typedef struct
{
	b3Vec3 origin;
	b3Vec3 u, v;   // edge vectors spanning the face
	b3Vec3 inward; // which way the surface must face to hold something in
} Face;

// An open-topped box: floor plus four walls, each tessellated into 2*n*n triangles. Vertices are
// duplicated along the seams and left for the welder, exactly like a cooked Unity mesh.
static b3MeshData* CreateContainerMesh( int n, int* outTriangleCount )
{
	const Face faces[5] = {
		{ { -HX, -HY, -HZ }, { 2 * HX, 0, 0 }, { 0, 0, 2 * HZ }, { 0, 1, 0 } },	 // floor
		{ { HX, -HY, -HZ }, { 0, 2 * HY, 0 }, { 0, 0, 2 * HZ }, { -1, 0, 0 } },	 // +x wall
		{ { -HX, -HY, -HZ }, { 0, 2 * HY, 0 }, { 0, 0, 2 * HZ }, { 1, 0, 0 } },	 // -x wall
		{ { -HX, -HY, HZ }, { 2 * HX, 0, 0 }, { 0, 2 * HY, 0 }, { 0, 0, -1 } },	 // +z wall
		{ { -HX, -HY, -HZ }, { 2 * HX, 0, 0 }, { 0, 2 * HY, 0 }, { 0, 0, 1 } },	 // -z wall
	};

	int perFace = ( n + 1 ) * ( n + 1 );
	int vertexCount = 5 * perFace;
	int triangleCount = 5 * 2 * n * n;

	b3Vec3* vertices = malloc( (size_t)vertexCount * sizeof( b3Vec3 ) );
	int* indices = malloc( (size_t)triangleCount * 3 * sizeof( int ) );

	int written = 0;

	for ( int f = 0; f < 5; ++f )
	{
		const Face* face = faces + f;
		int base = f * perFace;

		for ( int r = 0; r <= n; ++r )
		{
			for ( int c = 0; c <= n; ++c )
			{
				float a = (float)r / (float)n;
				float b = (float)c / (float)n;
				vertices[base + r * ( n + 1 ) + c] =
					b3Add( face->origin, b3Add( b3MulSV( a, face->u ), b3MulSV( b, face->v ) ) );
			}
		}

		for ( int r = 0; r < n; ++r )
		{
			for ( int c = 0; c < n; ++c )
			{
				int i00 = base + r * ( n + 1 ) + c;
				int i10 = base + ( r + 1 ) * ( n + 1 ) + c;
				int i11 = base + ( r + 1 ) * ( n + 1 ) + c + 1;
				int i01 = base + r * ( n + 1 ) + c + 1;

				int quad[2][3] = { { i00, i10, i11 }, { i00, i11, i01 } };
				for ( int t = 0; t < 2; ++t )
				{
					int a = quad[t][0], b = quad[t][1], c2 = quad[t][2];

					// Wind so the face points into the container, whatever the parametrization did.
					b3Vec3 normal = b3Cross( b3Sub( vertices[b], vertices[a] ), b3Sub( vertices[c2], vertices[a] ) );
					if ( b3Dot( normal, face->inward ) < 0.0f )
					{
						int swap = b;
						b = c2;
						c2 = swap;
					}

					indices[3 * written + 0] = a;
					indices[3 * written + 1] = b;
					indices[3 * written + 2] = c2;
					written += 1;
				}
			}
		}
	}

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.vertexCount = vertexCount;
	def.indices = indices;
	def.triangleCount = written;
	def.weldTolerance = 0.001f; // MineMogul's B3MeshCache settings
	def.weldVertices = true;
	def.identifyEdges = true;

	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );

	free( vertices );
	free( indices );

	*outTriangleCount = written;
	return mesh;
}

// ------------------------------------------------------------------ one drop

// Real MineMogul collider sizes. The point of the sweep is BODY SIZE: a contact pair queries the
// body's AABB enlarged by B3_MAX_AABB_MARGIN + B3_SPECULATIVE_DISTANCE (0.07 m each side), and
// mesh_contact.c caps that query at B3_MAX_MESH_CONTACT_TRIANGLES triangles and then truncates.
// A crate whose query box swallows the whole cart asks for every triangle it has.
typedef struct
{
	const char* name;
	float hx, hy, hz;
} Item;

static const Item s_items[] = {
	{ "IronIngot 221x95x478", 0.11030f, 0.04723f, 0.23919f },
	{ "CrateSmall 904x651x889", 0.45217f, 0.32529f, 0.44441f },
	{ "CrateMedium 1450x1082x734", 0.72481f, 0.54077f, 0.36691f },
	{ "CrateTall 962x2380x1231", 0.48097f, 1.18985f, 0.61544f },
};
#define ITEM_COUNT ( (int)( sizeof( s_items ) / sizeof( s_items[0] ) ) )

static bool RunDrop( const Item* item, float speed, int trial, b3MeshData* mesh )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	worldDef.enableContinuous = true;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef cartDef = b3DefaultBodyDef();
	cartDef.type = b3_staticBody;
	b3BodyId cartId = b3CreateBody( worldId, &cartDef );

	b3ShapeDef cartShapeDef = b3DefaultShapeDef();
	cartShapeDef.baseMaterial.friction = 0.6f;
	b3CreateMeshShape( cartId, &cartShapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

	// Land it on the cart at low speed. A big crate rests across the rim rather than dropping
	// inside, which is the reported case: nudging a crate into the cart, not throwing it.
	b3BodyDef itemDef = b3DefaultBodyDef();
	itemDef.type = b3_dynamicBody;
	itemDef.position = (b3Pos){ RandomRange( -0.1f, 0.1f ), HY + item->hy + 0.15f, RandomRange( -0.2f, 0.2f ) };
	// Only a small tilt: a crate being pushed around is roughly upright, not tumbling.
	itemDef.rotation = trial == 0 ? b3Quat_identity
								  : b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ RandomRange( -1.0f, 1.0f ), 1.0f,
																				   RandomRange( -1.0f, 1.0f ) } ),
															 RandomRange( -0.35f, 0.35f ) );
	itemDef.linearVelocity = (b3Vec3){ RandomRange( -0.3f, 0.3f ), -speed, RandomRange( -0.3f, 0.3f ) };
	itemDef.linearDamping = 0.2f;
	itemDef.angularDamping = 0.05f;
	b3BodyId itemId = b3CreateBody( worldId, &itemDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	// MineMogul stamps massOverride = 1 on every BasePhysicsObject (nothing in the game ever
	// authored a Rigidbody mass), so every body is 1 kg regardless of size. Using a real density
	// here would make the tall crate ~5.6 tonnes and manufacture a failure the game cannot have.
	shapeDef.density = 1.0f / ( 8.0f * item->hx * item->hy * item->hz );
	shapeDef.baseMaterial.friction = 0.6f;
	b3BoxHull box = b3MakeBoxHull( item->hx, item->hy, item->hz );
	b3CreateHullShape( itemId, &shapeDef, &box.base );

	bool escaped = false;
	for ( int step = 0; step < MAX_STEPS; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );

		b3Pos p = b3Body_GetPosition( itemId );
		// Below the cart floor AND still over its footprint means it went through the mesh.
		// A crate too big to sit in the cart can legitimately slide off the rim and fall past it,
		// which leaves the footprint - that must not be counted as phasing.
		if ( (float)p.y < -HY - 0.05f && fabsf( (float)p.x ) < HX && fabsf( (float)p.z ) < HZ )
		{
			escaped = true;
			break;
		}
	}

	b3DestroyWorld( worldId );
	return escaped;
}

// ------------------------------------------------------------------ main

#define TRIALS 64

int main( void )
{
	b3SetLogFcn( CaptureLog );

	static const int tessellations[] = { 4, 8, 16 };
	static const float speeds[] = { 1.0f, 2.0f, 5.0f };
	const int tessCount = (int)( sizeof( tessellations ) / sizeof( tessellations[0] ) );
	const int speedCount = (int)( sizeof( speeds ) / sizeof( speeds[0] ) );

	printf( "thin-walled concave container (open-top box, zero-thickness walls, normals inward)\n" );
	printf( "interior %.2f x %.2f x %.2f m, dt %.3f with %d substeps\n", 2 * HX, 2 * HY, 2 * HZ, TIME_STEP, SUB_STEPS );
	printf( "landing bodies on the cart at LOW speed, counting those that end up BELOW its floor\n" );
	printf( "%d drops per cell. A contact pair queries the body AABB grown by 0.07 m per side,\n", TRIALS );
	printf( "capped at B3_MAX_MESH_CONTACT_TRIANGLES triangles and then silently truncated.\n\n" );

	int grandTotal = 0;

	for ( int t = 0; t < tessCount; ++t )
	{
		int triangleCount = 0;
		b3MeshData* mesh = CreateContainerMesh( tessellations[t], &triangleCount );
		if ( mesh == NULL )
		{
			printf( "tessellation %d: MESH BAKE FAILED\n", tessellations[t] );
			continue;
		}

		printf( "cart mesh: %d triangles\n", triangleCount );
		printf( "  %-28s", "body" );
		for ( int s = 0; s < speedCount; ++s )
		{
			printf( " %5.0f", speeds[s] );
		}
		printf( "   total   cap hit\n" );

		for ( int i = 0; i < ITEM_COUNT; ++i )
		{
			s_sawTriangleCap = false;
			int rowTotal = 0;

			printf( "  %-28s", s_items[i].name );

			for ( int s = 0; s < speedCount; ++s )
			{
				s_seed = 0x9E3779B9u ^ (uint32_t)( s * 7919 );

				int escapes = 0;
				for ( int trial = 0; trial < TRIALS; ++trial )
				{
					escapes += RunDrop( s_items + i, speeds[s], trial, mesh ) ? 1 : 0;
				}

				printf( " %5d", escapes );
				rowTotal += escapes;
			}

			printf( "   %5d   %s\n", rowTotal, s_sawTriangleCap ? "YES" : "no" );
			grandTotal += rowTotal;
		}

		printf( "\n" );
		b3DestroyMesh( mesh );
	}

	// ---- cost of the triangles themselves ----
	// Removing the cap does not make the allocator slower; it makes the contact do the collision
	// work it was previously skipping. That cost is what this measures: one settled crate on carts
	// of increasing density, with a cap high enough that nothing is truncated, so every triangle in
	// the query is actually processed. The slope is what you would pay by lifting the cap - and
	// equally, what a simpler collider would save you.
	printf( "steady-state cost of a settled CrateSmall vs cart density (no truncation)\n" );
	printf( "  %-12s %-12s %s\n", "triangles", "ms/step", "vs coarsest" );

	float baseline = 0.0f;
	for ( int t = 0; t < tessCount; ++t )
	{
		int triangleCount = 0;
		b3MeshData* mesh = CreateContainerMesh( tessellations[t], &triangleCount );
		if ( mesh == NULL )
		{
			continue;
		}

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
		worldDef.enableContinuous = true;
		worldDef.enableSleep = false; // keep it solving, otherwise we would time a sleeping body
		worldDef.workerCount = 1;
		b3WorldId worldId = b3CreateWorld( &worldDef );

		// One pair per step sits at the QueryPerformanceCounter resolution floor (~100 ns), so
		// replicate the scenario until the narrow phase is comfortably measurable.
		const int copies = 64;
		// A flat slab that actually FITS the interior and lies on the floor, so its contact
		// patch covers most of the floor's triangles. CrateSmall is 2 mm wider than the cart, so
		// the solver ejects it onto the rim where it only ever touches a handful of triangles -
		// which would measure the wrong thing entirely.
		static const Item slab = { "slab 800x200x1300", 0.40f, 0.10f, 0.65f };
		const Item* item = &slab;

		for ( int copy = 0; copy < copies; ++copy )
		{
			float offset = (float)copy * 4.0f;

			b3BodyDef cartDef = b3DefaultBodyDef();
			cartDef.type = b3_staticBody;
			cartDef.position = (b3Pos){ offset, 0.0f, 0.0f };
			b3BodyId cartId = b3CreateBody( worldId, &cartDef );
			b3ShapeDef cartShapeDef = b3DefaultShapeDef();
			cartShapeDef.baseMaterial.friction = 0.6f;
			b3CreateMeshShape( cartId, &cartShapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

			b3BodyDef itemDef = b3DefaultBodyDef();
			itemDef.type = b3_dynamicBody;
			itemDef.position = (b3Pos){ offset, -HY + item->hy + 0.02f, 0.0f };
			itemDef.linearDamping = 0.2f;
			itemDef.angularDamping = 0.05f;
			b3BodyId itemId = b3CreateBody( worldId, &itemDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 1.0f / ( 8.0f * item->hx * item->hy * item->hz );
			shapeDef.baseMaterial.friction = 0.6f;
			b3BoxHull box = b3MakeBoxHull( item->hx, item->hy, item->hz );
			b3CreateHullShape( itemId, &shapeDef, &box.base );
		}

		for ( int i = 0; i < 200; ++i )
		{
			b3World_Step( worldId, TIME_STEP, SUB_STEPS );
		}

		// Total step time in a two-body world is mostly fixed overhead, which buries the signal.
		// profile.collide is the narrow phase, which is where the per-triangle work actually is.
		const int timed = 2000;
		double collide = 0.0;
		int graphContacts = 0;
		for ( int i = 0; i < timed; ++i )
		{
			b3World_Step( worldId, TIME_STEP, SUB_STEPS );
			collide += b3World_GetProfile( worldId ).collide;
			b3Counters counters = b3World_GetCounters( worldId );
			int solved = 0;
			for ( int c = 0; c < 24; ++c )
			{
				solved += counters.colorCounts[c];
			}
			graphContacts += solved;
		}
		float ms = (float)( collide / timed );

		if ( t == 0 )
		{
			baseline = ms;
		}

		printf( "  %-12d %-12.5f %-12.2fx contacts solved %.1f\n", triangleCount, ms,
				baseline > 0.0f ? ms / baseline : 1.0f, (double)graphContacts / timed );

		b3DestroyWorld( worldId );
		b3DestroyMesh( mesh );
	}

	printf( "\n%d escapes total\n", grandTotal );
	printf( "%s\n", grandTotal == 0 ? "PASS" : "FAIL" );

	return grandTotal == 0 ? 0 : 1;
}
