// Direct probe of b3TimeOfImpact, isolating the shape-type asymmetry behind hull tunneling.
//
// b3ContinuousQueryCallback only accepts a TOI hit when 0 < fraction < currentBest. Anything
// that comes back as fraction 0 is silently discarded and the body advances the entire step,
// which is exactly what "fell through the floor" looks like.
//
// b3TimeOfImpact returns fraction 0 in one place:
//
//     if ( distanceOutput.distance <= 0.0f ) { state = Overlapped; fraction = 0.0f; break; }
//
// distance here is the CORE distance - the distance between the proxy point clouds with
// useRadii = false. b3MakeShapeProxy gives a sphere/capsule its real radius and a hull radius
// zero, so the same physical configuration reads completely differently:
//
//     sphere r=0.15 whose surface is 0.1 mm inside the deck -> core distance 149.9 mm  -> Hit
//     box              whose surface is 0.1 mm inside the deck -> core distance -0.1 mm -> Overlapped
//
// A resting body is allowed to penetrate up to B3_LINEAR_SLOP (5 mm), so "surface slightly
// inside the deck" is the normal state of anything sitting on the floor, not an edge case.
//
// Each row prints the raw TOI result plus whether the CCD caller would have used it.

#include <box3d/box3d.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static const char* StateName( b3TOIState state )
{
	switch ( state )
	{
		case b3_toiStateUnknown:
			return "Unknown";
		case b3_toiStateFailed:
			return "Failed";
		case b3_toiStateOverlapped:
			return "Overlapped";
		case b3_toiStateHit:
			return "Hit";
		case b3_toiStateSeparated:
			return "Separated";
		default:
			return "?";
	}
}

static b3Sweep MakeStaticSweep( void )
{
	b3Sweep sweep = { 0 };
	sweep.localCenter = b3Vec3_zero;
	sweep.c1 = b3Vec3_zero;
	sweep.c2 = b3Vec3_zero;
	sweep.q1 = b3Quat_identity;
	sweep.q2 = b3Quat_identity;
	return sweep;
}

static b3Sweep MakeFallSweep( float y1, float y2, b3Quat q1, b3Quat q2 )
{
	b3Sweep sweep = { 0 };
	sweep.localCenter = b3Vec3_zero;
	sweep.c1 = (b3Vec3){ 0.0f, y1, 0.0f };
	sweep.c2 = (b3Vec3){ 0.0f, y2, 0.0f };
	sweep.q1 = q1;
	sweep.q2 = q2;
	return sweep;
}

static int s_discarded = 0;

static void Probe( const char* label, const char* shape, b3ShapeProxy groundProxy, b3ShapeProxy itemProxy, float y1, float y2,
				   b3Quat q1, b3Quat q2 )
{
	b3TOIInput input = { 0 };
	input.proxyA = groundProxy;
	input.proxyB = itemProxy;
	input.sweepA = MakeStaticSweep();
	input.sweepB = MakeFallSweep( y1, y2, q1, q2 );
	input.maxFraction = 1.0f;

	b3TOIOutput output = b3TimeOfImpact( &input );

	bool accepted = 0.0f < output.fraction && output.fraction < input.maxFraction;
	if ( accepted == false )
	{
		s_discarded += 1;
	}

	printf( "  %-26s %-22s %-11s f=%6.4f  core d=%8.4f mm  %s\n", label, shape, StateName( output.state ), output.fraction,
			1000.0f * output.distance, accepted ? "used by CCD" : "DISCARDED -> body advances full step" );
}

int main( void )
{
	// Ground: 1 m thick slab, top face at y = 0.
	b3BoxHull ground = b3MakeOffsetBoxHull( 5.0f, 0.5f, 5.0f, (b3Vec3){ 0.0f, -0.5f, 0.0f } );
	b3ShapeProxy groundProxy = { b3GetHullPoints( &ground.base ), ground.base.vertexCount, 0.0f };

	// Item shapes, all with a 150 mm half height so "surface at the deck" means centre y = 0.15.
	b3BoxHull box = b3MakeCubeHull( 0.15f );
	b3ShapeProxy boxProxy = { b3GetHullPoints( &box.base ), box.base.vertexCount, 0.0f };

	b3Vec3 sphereCenter = b3Vec3_zero;
	b3ShapeProxy sphereProxy = { &sphereCenter, 1, 0.15f };

	b3Vec3 capsulePoints[2] = { { 0.0f, -0.05f, 0.0f }, { 0.0f, 0.05f, 0.0f } };
	b3ShapeProxy capsuleProxy = { capsulePoints, 2, 0.15f };

	const float halfHeight = 0.15f;
	const float travel = 1.0f; // one step at 50 m/s

	printf( "b3TimeOfImpact probe: 1 m thick ground slab, item half height %.0f mm, sweep %.2f m down\n\n",
			1000.0f * halfHeight, travel );

	// ---- 1. clean approach from a comfortable gap. Everything should find the impact.
	printf( "clean approach, surface starts 200 mm above the deck:\n" );
	{
		float y1 = halfHeight + 0.200f;
		float y2 = y1 - travel;
		Probe( "gap 200 mm", "box 300 cube", groundProxy, boxProxy, y1, y2, b3Quat_identity, b3Quat_identity );
		Probe( "gap 200 mm", "sphere r=150", groundProxy, sphereProxy, y1, y2, b3Quat_identity, b3Quat_identity );
		Probe( "gap 200 mm", "capsule r=150", groundProxy, capsuleProxy, y1, y2, b3Quat_identity, b3Quat_identity );
	}

	// ---- 2. the body starts within the allowed penetration slop, which is the resting state
	//         of anything sitting on the floor, then is launched downward.
	printf( "\nstarts touching / slightly penetrating (all within the 5 mm penetration slop):\n" );
	{
		static const float overlaps[] = { 0.0f, 0.0001f, 0.001f, 0.004f };
		for ( int i = 0; i < 4; ++i )
		{
			char label[64];
			snprintf( label, sizeof( label ), "surface %.1f mm inside", 1000.0f * overlaps[i] );

			float y1 = halfHeight - overlaps[i];
			float y2 = y1 - travel;
			Probe( label, "box 300 cube", groundProxy, boxProxy, y1, y2, b3Quat_identity, b3Quat_identity );
			Probe( label, "sphere r=150", groundProxy, sphereProxy, y1, y2, b3Quat_identity, b3Quat_identity );
			Probe( label, "capsule r=150", groundProxy, capsuleProxy, y1, y2, b3Quat_identity, b3Quat_identity );
		}
	}

	// ---- 3. tumbling. The separation function is only exact for a fixed axis, so a rotating
	//         hull can be advanced past the true impact and land in the overlapped branch.
	printf( "\ntumbling through the deck (90 deg of rotation across the step):\n" );
	{
		b3Quat q1 = b3Quat_identity;
		b3Quat q2 = b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ 1.0f, 0.3f, 0.5f } ), 1.5708f );

		static const float gaps[] = { 0.300f, 0.150f, 0.050f, 0.010f, 0.000f };
		for ( int i = 0; i < 5; ++i )
		{
			char label[64];
			snprintf( label, sizeof( label ), "spinning, gap %.0f mm", 1000.0f * gaps[i] );

			float y1 = halfHeight + gaps[i];
			float y2 = y1 - travel;
			Probe( label, "box 300 cube", groundProxy, boxProxy, y1, y2, q1, q2 );
			Probe( label, "sphere r=150", groundProxy, sphereProxy, y1, y2, q1, q2 );
			Probe( label, "capsule r=150", groundProxy, capsuleProxy, y1, y2, q1, q2 );
		}
	}

	printf( "\n%d of the probes above produce no usable TOI\n", s_discarded );
	return 0;
}
