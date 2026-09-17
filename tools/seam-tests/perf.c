// Performance A/B for the seam/CCD contact rules, shaped like MineMogul's real workload.
//
// The question this answers: the two tunneling fixes both land on bodies flagged b3_isFast, and in
// a factory game EVERY CONVEYOR RIDER IS FAST. An IronIngot hull has innerRadius 47.2 mm, so the
// continuous stage triggers at maxMotion > 0.5 * 47.2 mm = 23.6 mm per step, which at dt = 1/50 is
// 1.18 m/s. Belts run 1.4-2.0 m/s. So the whole awake set takes the fast path every step, and any
// cost added there is multiplied by thousands of bodies.
//
// Build the exe ONCE and swap box3d.dll between runs - the fixes change no exports, so the same
// binary measures every engine variant with identical benchmark code. That removes the benchmark
// itself as a variable.
//
//   perf.exe                 # both scenes, default reps
//   perf.exe belt 5          # one scene, 5 reps
//   perf.exe mixed 5
//
// Scenes:
//   belt   - only belts and riders. Nothing but the hot path, so a regression there is not diluted.
//   mixed  - belts and riders over a triangle-mesh ground, plus sleeping hull piles, sized to the
//            project's "moderate" profile capture (8.5k bodies, ~1.6k awake).

#include <box3d/base.h>
#include <box3d/box3d.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP ( 1.0f / 50.0f ) // MineMogul's fixed timestep
#define SUB_STEPS 2				   // MineMogul's B3World.subStepCount

#define BELT_SPEED 1.6f // above the 1.18 m/s fast threshold, like the real belts
#define TILE_LEN 1.0f
#define TILE_HALF_X 0.47f
#define TILE_HALF_Y 0.0795f

#define LINE_COUNT 24
#define TILES_PER_LINE 30
#define RIDERS_PER_LINE 45
#define LINE_PITCH 1.4f

#define PILE_COUNT 6000

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

// ------------------------------------------------------------------ shapes

// The real IronIngot collider.
#define ITEM_HX 0.11030f
#define ITEM_HY 0.04723f
#define ITEM_HZ 0.23919f

// An irregular convex point cloud, the sort of thing b3CreateHull sees from a cooked Unity mesh.
static const b3Vec3 s_blobPoints[12] = {
	{ 0.125f, 0.030f, 0.020f },	  { -0.115f, 0.040f, 0.035f },	{ 0.020f, 0.105f, -0.030f },  { 0.015f, -0.095f, 0.045f },
	{ 0.055f, 0.025f, 0.120f },	  { -0.035f, 0.015f, -0.110f }, { 0.090f, -0.065f, -0.055f }, { -0.080f, -0.060f, 0.070f },
	{ -0.075f, 0.070f, -0.065f }, { 0.070f, 0.075f, 0.065f },	{ -0.020f, -0.100f, -0.025f }, { 0.100f, -0.025f, -0.090f },
};

typedef struct
{
	b3BodyId id;
	float resetZ;
	float startZ;
	int recycleCount;
} Rider;

typedef struct
{
	b3WorldId worldId;
	b3MeshData* mesh;
	b3HullData* blob;
	Rider* riders;
	int riderCount;
} Scene;

// ------------------------------------------------------------------ scene construction

static void BuildBelts( Scene* scene )
{
	b3ShapeDef beltDef = b3DefaultShapeDef();
	beltDef.baseMaterial.friction = 0.9f;
	beltDef.baseMaterial.rollingResistance = 0.4f;
	beltDef.baseMaterial.tangentVelocity = (b3Vec3){ 0.0f, 0.0f, BELT_SPEED };

	float lineLength = (float)TILES_PER_LINE * TILE_LEN;

	for ( int line = 0; line < LINE_COUNT; ++line )
	{
		float x = ( (float)line - 0.5f * (float)LINE_COUNT ) * LINE_PITCH;

		// Each tile is its own static body, butted flush against its neighbours. That is what the
		// game does, and it is what makes every contact on a belt be born at a seam.
		for ( int t = 0; t < TILES_PER_LINE; ++t )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_staticBody;
			bodyDef.position = (b3Pos){ x, -TILE_HALF_Y, ( (float)t + 0.5f ) * TILE_LEN };
			b3BodyId tileId = b3CreateBody( scene->worldId, &bodyDef );

			b3BoxHull tile = b3MakeBoxHull( TILE_HALF_X, TILE_HALF_Y, 0.5f * TILE_LEN );
			b3CreateHullShape( tileId, &beltDef, &tile.base );
		}

		for ( int r = 0; r < RIDERS_PER_LINE; ++r )
		{
			float z = ( (float)r + 0.5f ) * ( lineLength / (float)RIDERS_PER_LINE );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ x + RandomRange( -0.12f, 0.12f ), ITEM_HY + 0.02f, z };
			// Small yaw spread, like items that landed slightly turned.
			bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, RandomRange( -0.35f, 0.35f ) );
			bodyDef.linearVelocity = (b3Vec3){ 0.0f, 0.0f, BELT_SPEED };
			bodyDef.linearDamping = 0.2f;
			bodyDef.angularDamping = 0.05f;
			b3BodyId itemId = b3CreateBody( scene->worldId, &bodyDef );

			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = 2000.0f;
			shapeDef.baseMaterial.friction = 0.6f;

			// Mix the shapes the way real ore does: mostly boxes, some cooked hulls.
			if ( ( r % 4 ) == 3 )
			{
				b3CreateHullShape( itemId, &shapeDef, scene->blob );
			}
			else
			{
				b3BoxHull box = b3MakeBoxHull( ITEM_HX, ITEM_HY, ITEM_HZ );
				b3CreateHullShape( itemId, &shapeDef, &box.base );
			}

			scene->riders[scene->riderCount].id = itemId;
			scene->riders[scene->riderCount].resetZ = lineLength;
			scene->riders[scene->riderCount].startZ = 0.25f;
			scene->riderCount += 1;
		}
	}
}

static void BuildMeshGround( Scene* scene )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, -0.20f, 0.5f * (float)TILES_PER_LINE * TILE_LEN };
	b3BodyId groundId = b3CreateBody( scene->worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;

	// A triangle mesh, so the mesh contact path is exercised too, not just hull vs hull.
	scene->mesh = b3CreateGridMesh( 30, 40, 2.0f, 1, true );
	b3CreateMeshShape( groundId, &shapeDef, scene->mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
}

static void BuildPiles( Scene* scene )
{
	// Loose hull bodies that settle and sleep. They are most of the body count and most of the
	// broad-phase tree, but almost none of the per-step cost - which is exactly the real shape of
	// this game's scene.
	const int perRow = 40;
	float baseZ = 0.5f * (float)TILES_PER_LINE * TILE_LEN;

	for ( int i = 0; i < PILE_COUNT; ++i )
	{
		int col = i % perRow;
		int row = ( i / perRow ) % perRow;
		int layer = i / ( perRow * perRow );

		float x = ( (float)col - 0.5f * (float)perRow ) * 0.55f - 26.0f;
		float z = baseZ + ( (float)row - 0.5f * (float)perRow ) * 0.55f;
		float y = 0.12f + (float)layer * 0.30f;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ x + RandomRange( -0.05f, 0.05f ), y, z + RandomRange( -0.05f, 0.05f ) };
		bodyDef.rotation = RandomQuat();
		bodyDef.linearDamping = 0.2f;
		bodyDef.angularDamping = 0.05f;
		b3BodyId bodyId = b3CreateBody( scene->worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = 2000.0f;
		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.05f;

		if ( ( i % 3 ) == 0 )
		{
			b3CreateHullShape( bodyId, &shapeDef, scene->blob );
		}
		else
		{
			b3BoxHull box = b3MakeBoxHull( ITEM_HX, ITEM_HY, ITEM_HZ );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

static void BuildFlatGround( Scene* scene )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( scene->worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;
	b3BoxHull slab = b3MakeOffsetBoxHull( 60.0f, 0.5f, 60.0f, (b3Vec3){ 0.0f, -0.5f, 0.0f } );
	b3CreateHullShape( groundId, &shapeDef, &slab.base );
}

static Scene CreateScene( bool mixed, uint32_t workerCount )
{
	s_seed = 0x9E3779B9u;

	Scene scene = { 0 };

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	worldDef.enableContinuous = true; // MineMogul's B3World setting
	worldDef.enableSleep = true;
	worldDef.workerCount = workerCount;
	scene.worldId = b3CreateWorld( &worldDef );

	scene.blob = b3CreateHull( s_blobPoints, 12, 12 );

	int maxRiders = LINE_COUNT * RIDERS_PER_LINE;
	scene.riders = calloc( (size_t)maxRiders, sizeof( Rider ) );

	BuildBelts( &scene );

	if ( mixed )
	{
		BuildMeshGround( &scene );
		BuildFlatGround( &scene );
		BuildPiles( &scene );
	}

	return scene;
}

static void DestroyScene( Scene* scene )
{
	b3DestroyWorld( scene->worldId );
	if ( scene->mesh != NULL )
	{
		b3DestroyMesh( scene->mesh );
	}
	if ( scene->blob != NULL )
	{
		b3DestroyHull( scene->blob );
	}
	free( scene->riders );
}

// Keep the belts in steady state: an item that runs off the end goes back to the start with its
// velocity intact. A factory never drains, and a benchmark that drains stops measuring the thing
// it was built to measure.
static void Recirculate( Scene* scene )
{
	for ( int i = 0; i < scene->riderCount; ++i )
	{
		Rider* rider = scene->riders + i;
		b3Pos p = b3Body_GetPosition( rider->id );
		if ( (float)p.z > rider->resetZ )
		{
			// Per-rider deterministic yaw, NOT a draw from the shared random stream. Two engine
			// variants recirculate at slightly different steps, and a shared stream would then feed
			// them different values and make the scenes diverge - which would show up as a
			// performance difference that is really just a different scene.
			rider->recycleCount += 1;
			float yaw = 0.35f * sinf( 2.399963f * (float)( i + 7 * rider->recycleCount ) );

			b3Body_SetTransform( rider->id, (b3Pos){ p.x, ITEM_HY + 0.01f, rider->startZ },
								 b3MakeQuatFromAxisAngle( b3Vec3_axisY, yaw ) );
			b3Body_SetLinearVelocity( rider->id, (b3Vec3){ 0.0f, 0.0f, BELT_SPEED } );
			b3Body_SetAngularVelocity( rider->id, b3Vec3_zero );
		}
	}
}

// ------------------------------------------------------------------ statistics

static int CompareFloat( const void* a, const void* b )
{
	float fa = *(const float*)a;
	float fb = *(const float*)b;
	return fa < fb ? -1 : ( fa > fb ? 1 : 0 );
}

typedef struct
{
	float median;
	float mean;
	float p95;
	float min;
} Stats;

static Stats Summarize( float* samples, int count )
{
	qsort( samples, (size_t)count, sizeof( float ), CompareFloat );

	double sum = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		sum += samples[i];
	}

	Stats stats;
	stats.median = samples[count / 2];
	stats.mean = (float)( sum / (double)count );
	stats.p95 = samples[(int)( 0.95f * (float)( count - 1 ) )];
	stats.min = samples[0];
	return stats;
}

// ------------------------------------------------------------------ one run

typedef struct
{
	Stats step;
	float pairs, collide, solve, transforms, refit;
	int bodyCount, awakeContactCount, contactCount, satCallCount;
	int graphContactCount; // contacts actually coloured into the constraint graph, i.e. solved
	int awakeBodies;
	int distanceIterations, pushBackIterations, rootIterations;
} RunResult;

// awakeContactCount counts contacts the collide pass touched, touching or not, so it does not move
// when a rule changes whether a manifold is KEPT. The constraint-graph colour counts do: a contact
// only gets coloured once it is touching, so their sum is the number of contacts the solver
// actually works on.
static int SumGraphContacts( const b3Counters* counters )
{
	int total = 0;
	for ( int i = 0; i < 24; ++i )
	{
		total += counters->colorCounts[i];
	}
	return total;
}

static RunResult RunScene( bool mixed, uint32_t workerCount, int warmupSteps, int timedSteps )
{
	Scene scene = CreateScene( mixed, workerCount );

	for ( int i = 0; i < warmupSteps; ++i )
	{
		b3World_Step( scene.worldId, TIME_STEP, SUB_STEPS );
		Recirculate( &scene );
	}

	float* samples = malloc( (size_t)timedSteps * sizeof( float ) );
	double pairs = 0.0, collide = 0.0, solve = 0.0, transforms = 0.0, refit = 0.0;
	double awakeContacts = 0.0, satCalls = 0.0, graphContacts = 0.0;
	int distanceIterations = 0, pushBackIterations = 0, rootIterations = 0;

	for ( int i = 0; i < timedSteps; ++i )
	{
		uint64_t ticks = b3GetTicks();
		b3World_Step( scene.worldId, TIME_STEP, SUB_STEPS );
		samples[i] = b3GetMilliseconds( ticks );

		// Recirculation is deliberately outside the timed region.
		Recirculate( &scene );

		b3Profile profile = b3World_GetProfile( scene.worldId );
		pairs += profile.pairs;
		collide += profile.collide;
		solve += profile.solve;
		transforms += profile.transforms;
		refit += profile.refit;

		b3Counters counters = b3World_GetCounters( scene.worldId );
		awakeContacts += counters.awakeContactCount;
		satCalls += counters.satCallCount;
		graphContacts += SumGraphContacts( &counters );
		distanceIterations = counters.distanceIterations > distanceIterations ? counters.distanceIterations : distanceIterations;
		pushBackIterations = counters.pushBackIterations > pushBackIterations ? counters.pushBackIterations : pushBackIterations;
		rootIterations = counters.rootIterations > rootIterations ? counters.rootIterations : rootIterations;
	}

	b3Counters final = b3World_GetCounters( scene.worldId );

	int awakeBodies = 0;
	for ( int i = 0; i < scene.riderCount; ++i )
	{
		awakeBodies += b3Body_IsAwake( scene.riders[i].id ) ? 1 : 0;
	}

	RunResult result = { 0 };
	result.step = Summarize( samples, timedSteps );
	result.pairs = (float)( pairs / timedSteps );
	result.collide = (float)( collide / timedSteps );
	result.solve = (float)( solve / timedSteps );
	result.transforms = (float)( transforms / timedSteps );
	result.refit = (float)( refit / timedSteps );
	result.bodyCount = final.bodyCount;
	result.contactCount = final.contactCount;
	result.awakeContactCount = (int)( awakeContacts / timedSteps );
	result.satCallCount = (int)( satCalls / timedSteps );
	result.graphContactCount = (int)( graphContacts / timedSteps );
	result.awakeBodies = awakeBodies;
	result.distanceIterations = distanceIterations;
	result.pushBackIterations = pushBackIterations;
	result.rootIterations = rootIterations;

	free( samples );
	DestroyScene( &scene );

	return result;
}

// ------------------------------------------------------------------ main

static void RunSuite( const char* name, bool mixed, uint32_t workerCount, int reps, int warmupSteps, int timedSteps )
{
	printf( "scene %s  workers %u  warmup %d  timed %d  reps %d\n", name, workerCount, warmupSteps, timedSteps, reps );

	float* medians = malloc( (size_t)reps * sizeof( float ) );
	RunResult last = { 0 };

	for ( int r = 0; r < reps; ++r )
	{
		RunResult result = RunScene( mixed, workerCount, warmupSteps, timedSteps );
		medians[r] = result.step.median;
		last = result;

		printf( "  rep %d  median %6.3f  mean %6.3f  p95 %6.3f  min %6.3f ms\n", r, result.step.median, result.step.mean,
				result.step.p95, result.step.min );
	}

	Stats across = Summarize( medians, reps );

	printf( "  bodies %d   riders awake %d   contacts %d   awake contacts %d   SOLVED contacts %d   SAT calls/step %d\n",
			last.bodyCount, last.awakeBodies, last.contactCount, last.awakeContactCount, last.graphContactCount,
			last.satCallCount );
	printf( "  max TOI iterations: distance %d  pushBack %d  root %d\n", last.distanceIterations, last.pushBackIterations,
			last.rootIterations );
	printf( "  stages ms/step: pairs %.3f  collide %.3f  solve %.3f  transforms %.3f  refit %.3f\n", last.pairs, last.collide,
			last.solve, last.transforms, last.refit );
	printf( "  RESULT %s  best-of-reps median %.4f ms   (median-of-medians %.4f, spread %.4f)\n\n", name, across.min,
			across.median, across.p95 - across.min );

	free( medians );
}

int main( int argc, char** argv )
{
	const char* scene = argc > 1 ? argv[1] : "all";
	int reps = argc > 2 ? atoi( argv[2] ) : 3;
	uint32_t workerCount = argc > 3 ? (uint32_t)atoi( argv[3] ) : 1;

	printf( "box3d contact/CCD performance, MineMogul-shaped workload\n" );
	printf( "dt %.4f, %d substeps, continuous ON, belt %.1f m/s (fast threshold for the ingot hull is 1.18 m/s)\n\n", TIME_STEP,
			SUB_STEPS, BELT_SPEED );

	// Single threaded by default: it is the cleanest A/B signal. Thread scheduling noise on a
	// desktop is larger than the effect being measured.
	if ( strcmp( scene, "belt" ) == 0 || strcmp( scene, "all" ) == 0 )
	{
		RunSuite( "belt", false, workerCount, reps, 250, 600 );
	}

	if ( strcmp( scene, "mixed" ) == 0 || strcmp( scene, "all" ) == 0 )
	{
		RunSuite( "mixed", true, workerCount, reps, 400, 600 );
	}

	return 0;
}
