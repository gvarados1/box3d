// Regression test for the grazing-flag latch.
//
// A contact that grazes once - which is exactly what happens as a body crosses a seam - must not
// keep that classification after it settles into a flat face pair. If it does, contact.c keeps
// its rest offset suppressed and the body rides BELOW the deck instead of B3_CONVEX_REST_OFFSET
// above it. Every seam downstream then becomes a step up, and a step up above the resting gap
// traps a flat item permanently.
//
// The latch was: b3ClassifyGrazingContact wrote cache->grazing = 1 and nothing ever wrote 0, and
// the cached fast paths restored it without redoing the axis query. A contact born at a seam
// therefore stayed flagged for as long as the cached feature kept hitting - and because each
// belt tile is its own body, every contact is born at a seam.
//
// Pass criteria, on a perfectly flush belt:
//   - a body dropped hard settles at +restOffset above the deck, not below it
//   - it settles there regardless of where it lands relative to a seam
//   - it keeps that gap as it crosses seam after seam
//   - it never stops

#include <box3d/box3d.h>

#include <math.h>
#include <stdio.h>

#define TILE_COUNT 8
#define TILE_LEN 1.0f
#define TILE_HALF_X 0.472f
#define TILE_HALF_Y 0.0795f
#define BELT_SPEED 1.4f
#define TIME_STEP ( 1.0f / 50.0f )	// MineMogul's fixed timestep
#define SUB_STEPS 2					// MineMogul's B3World.subStepCount

#define ITEM_MASS 1.0f

// Item shapes to try. The latch turned out to be shape sensitive, so test more than one:
// the real IronIngot collider (220.6 x 94.5 x 478.4 mm) and the shorter slab that first
// exposed the sunk state.
typedef struct
{
	const char* name;
	float hx, hy, hz;
} ItemShape;

static const ItemShape s_shapes[] = {
	{ "IronIngot 221x95x478", 0.11030f, 0.04723f, 0.23919f },
	{ "slab 300x80x150", 0.15000f, 0.04000f, 0.07500f },
	{ "slab 300x80x300", 0.15000f, 0.04000f, 0.15000f },
	{ "cube 150", 0.07500f, 0.07500f, 0.07500f },
};
#define SHAPE_COUNT ( (int)( sizeof( s_shapes ) / sizeof( s_shapes[0] ) ) )

static ItemShape g_shape;
#define ITEM_HX g_shape.hx
#define ITEM_HY g_shape.hy
#define ITEM_HZ g_shape.hz

static void BuildBelt( b3WorldId worldId )
{
	b3ShapeDef beltDef = b3DefaultShapeDef();
	beltDef.baseMaterial.friction = 0.6f;
	beltDef.baseMaterial.tangentVelocity = ( b3Vec3 ){ 0.0f, 0.0f, BELT_SPEED };

	for ( int i = 0; i < TILE_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		bodyDef.position = ( b3Pos ){ 0.0f, -TILE_HALF_Y, ( (float)i + 0.5f ) * TILE_LEN };
		b3BodyId tileId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull tile = b3MakeBoxHull( TILE_HALF_X, TILE_HALF_Y, 0.5f * TILE_LEN );
		b3CreateHullShape( tileId, &beltDef, &tile.base );
	}
}

// Returns the worst (lowest) settled gap above the deck seen after the item has calmed down,
// and how far it travelled.
static void RunDrop( float dropHeight, float landingZ, float* outWorstSettledGap, float* outEndZ, int* outSunkSteps )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	b3WorldId worldId = b3CreateWorld( &worldDef );

	BuildBelt( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = ( b3Pos ){ 0.0f, ITEM_HY + dropHeight, landingZ };
	bodyDef.enableSleep = false;
	b3BodyId itemId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6f;
	shapeDef.density = ITEM_MASS / ( 8.0f * ITEM_HX * ITEM_HY * ITEM_HZ );
	b3BoxHull box = b3MakeBoxHull( ITEM_HX, ITEM_HY, ITEM_HZ );
	b3CreateHullShape( itemId, &shapeDef, &box.base );

	float worstGap = 1000.0f;
	int sunkSteps = 0;

	// Let it fall and land, then watch the settled state for the rest of the run.
	int fallSteps = (int)( 50.0f * sqrtf( 2.0f * dropHeight / 9.81f ) ) + 40;
	int maxSteps = fallSteps + 800;

	for ( int step = 0; step < maxSteps; ++step )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEPS );

		b3Pos p = b3Body_GetPosition( itemId );
		if ( (float)p.z > (float)TILE_COUNT * TILE_LEN - 0.35f )
		{
			break;
		}

		if ( step < fallSteps )
		{
			continue;
		}

		float gap = (float)p.y - ITEM_HY;
		worstGap = fminf( worstGap, gap );
		if ( gap < 0.0f )
		{
			sunkSteps += 1;
		}
	}

	b3Pos p = b3Body_GetPosition( itemId );
	*outWorstSettledGap = worstGap;
	*outEndZ = (float)p.z;
	*outSunkSteps = sunkSteps;

	b3DestroyWorld( worldId );
}

int main( void )
{
	printf( "grazing-flag latch regression\n" );
	printf( "flush belt, %d tiles of %.1f m, belt %.1f m/s, dt %.3f with %d substeps, %.1f kg\n\n", TILE_COUNT, TILE_LEN,
			BELT_SPEED, TIME_STEP, SUB_STEPS, ITEM_MASS );

	float heights[6] = { 0.25f, 0.5f, 1.0f, 3.0f, 8.0f, 20.0f };
	int failures = 0;
	int runs = 0;

	printf( "%-22s %6s %14s %12s %10s\n", "item", "drop", "worst settled gap", "sunk steps", "stopped" );

	for ( int s = 0; s < SHAPE_COUNT; ++s )
	{
		g_shape = s_shapes[s];

		for ( int h = 0; h < 6; ++h )
		{
			float worstOfAll = 1000.0f;
			int totalSunk = 0;
			int stopped = 0;
			const int offsets = 21;

			for ( int i = 0; i < offsets; ++i )
			{
				// Sweep the landing point across the seam at z = 4.
				float landingZ = 4.0f - 0.25f + 0.5f * (float)i / (float)( offsets - 1 );

				float worstGap, endZ;
				int sunkSteps;
				RunDrop( heights[h], landingZ, &worstGap, &endZ, &sunkSteps );

				runs += 1;
				worstOfAll = fminf( worstOfAll, worstGap );
				totalSunk += sunkSteps;
				if ( endZ < (float)TILE_COUNT * TILE_LEN - 0.35f )
				{
					stopped += 1;
				}
			}

			bool bad = worstOfAll < 0.0f || stopped > 0;
			failures += bad ? 1 : 0;

			printf( "%-22s %5.2fm %11.3f mm %12d %10d   %s\n", g_shape.name, heights[h], 1000.0f * worstOfAll, totalSunk,
					stopped, bad ? "FAIL" : "ok" );
		}
	}

	printf( "\n%d of %d item/drop combinations settled below the deck or stopped (%d landings total)\n", failures,
			SHAPE_COUNT * 6, runs );
	printf( "%s\n", failures == 0 ? "PASS" : "FAIL" );
	return failures == 0 ? 0 : 1;
}
