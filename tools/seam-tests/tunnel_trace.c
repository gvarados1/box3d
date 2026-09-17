// Step-by-step trace of a drop that still ends up under the floor.
//
// The point is to tell two very different failures apart:
//
//   tunneling      - one step moves the body from above the slab to below it, no contact ever
//   wrong-side pop - the body is stopped inside the slab, then the contact solver resolves
//                    against the bottom face and pushes it out the wrong way over a few steps
//
// The first is a continuous collision problem. The second is a contact problem and no amount
// of TOI work will fix it.

#include <box3d/box3d.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TIME_STEP ( 1.0f / 50.0f )
#define SUB_STEPS 2
#define MAX_STEPS 200

static uint32_t s_seed;

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

#define GROUND_HALF_Y 0.05f

// The ingot, which is one of the shapes that still fails on a thin deck.
#define ITEM_HX 0.11030f
#define ITEM_HY 0.04723f
#define ITEM_HZ 0.23919f

static float RunDrop( float speed, b3Quat rotation, b3Vec3 spin, bool trace )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	worldDef.enableContinuous = true;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	groundShapeDef.baseMaterial.friction = 0.5f;
	b3BoxHull slab = b3MakeOffsetBoxHull( 40.0f, GROUND_HALF_Y, 40.0f, (b3Vec3){ 0.0f, -GROUND_HALF_Y, 0.0f } );
	b3CreateHullShape( groundId, &groundShapeDef, &slab.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 3.0f, 0.0f };
	bodyDef.rotation = rotation;
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -speed, 0.0f };
	bodyDef.angularVelocity = spin;
	bodyDef.linearDamping = 0.2f;
	bodyDef.angularDamping = 0.05f;
	b3BodyId itemId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 2000.0f;
	shapeDef.baseMaterial.friction = 0.5f;
	b3BoxHull box = b3MakeBoxHull( ITEM_HX, ITEM_HY, ITEM_HZ );
	b3CreateHullShape( itemId, &shapeDef, &box.base );

	if ( trace )
	{
		printf( "  %5s %11s %11s %11s %9s\n", "step", "y before", "y after", "vy after", "contacts" );
	}

	for ( int step = 0; step < MAX_STEPS; ++step )
	{
		b3Pos before = b3Body_GetPosition( itemId );

		b3World_Step( worldId, TIME_STEP, SUB_STEPS );

		b3Pos after = b3Body_GetPosition( itemId );
		b3Vec3 v = b3Body_GetLinearVelocity( itemId );
		int contacts = b3Body_GetContactCapacity( itemId );

		// Only print around the interesting region.
		if ( trace && (float)before.y < 1.0f )
		{
			printf( "  %5d %11.4f %11.4f %11.3f %9d%s\n", step, (float)before.y, (float)after.y, v.y, contacts,
					(float)before.y > 0.0f && (float)after.y < -GROUND_HALF_Y ? "   <-- crossed the slab in one step" : "" );
		}

		if ( (float)after.y < -3.0f )
		{
			break;
		}
	}

	b3Pos p = b3Body_GetPosition( itemId );
	b3DestroyWorld( worldId );
	return (float)p.y;
}

int main( void )
{
	static const float speeds[] = { 20.0f, 30.0f, 40.0f, 60.0f, 80.0f, 100.0f };
	const int speedCount = (int)( sizeof( speeds ) / sizeof( speeds[0] ) );

	printf( "trace of ingot drops onto a %.0f mm deck that still finish below it\n\n", 2000.0f * GROUND_HALF_Y );

	int traced = 0;

	for ( int s = 0; s < speedCount && traced < 3; ++s )
	{
		s_seed = 0x9E3779B9u ^ (uint32_t)( s * 7919 );

		for ( int t = 0; t < 24 && traced < 3; ++t )
		{
			b3Quat rotation = t == 0 ? b3Quat_identity : RandomQuat();
			b3Vec3 spin = { RandomRange( -8.0f, 8.0f ), RandomRange( -8.0f, 8.0f ), RandomRange( -8.0f, 8.0f ) };
			if ( t == 0 )
			{
				spin = b3Vec3_zero;
			}

			float finalY = RunDrop( speeds[s], rotation, spin, false );
			if ( finalY < -GROUND_HALF_Y - 0.25f )
			{
				printf( "speed %.0f m/s, trial %d, final y = %.2f m\n", speeds[s], t, finalY );
				RunDrop( speeds[s], rotation, spin, true );
				printf( "\n" );
				traced += 1;
			}
		}
	}

	if ( traced == 0 )
	{
		printf( "no failures found\n" );
	}

	return 0;
}
