// SPDX-License-Identifier: MIT

// MineMogul fork: fluid volumes, see docs/fluid.md. Buoyancy, drag and currents for awake bodies,
// evaluated once per step at a predicted pose and folded into the solver's implicit damping.

#include "fluid.h"

#include "body.h"
#include "core.h"
#include "ctz.h"
#include "math_internal.h"
#include "parallel_for.h"
#include "physics_world.h"
#include "recording.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include "box3d/box3d.h"

#include <string.h>

// Defined in physics_world.c
extern b3World b3_worlds[B3_MAX_WORLDS];

// Gauss-Legendre after x = sin(pi * u / 2) to smooth the band ends; weights include the Jacobian
#define B3_FLUID_QUADRATURE_COUNT 6

static const float b3_fluidNodes[B3_FLUID_QUADRATURE_COUNT] = {
	-0.994379147f, -0.861707499f, -0.366107024f, 0.366107024f, 0.861707499f, 0.994379147f,
};

static const float b3_fluidWeights[B3_FLUID_QUADRATURE_COUNT] = {
	0.028493402f, 0.287537961f, 0.683968637f, 0.683968637f, 0.287537961f, 0.028493402f,
};

typedef enum b3FluidWakeRule
{
	// Every sleeping body in the region
	b3_fluidWakeRegion,

	// Bodies whose immersion may change between the previous and the current surface
	b3_fluidWakeSurface,

	// Bodies not fully above the surface
	b3_fluidWakeImmersed,
} b3FluidWakeRule;

// The n-th root from a bit pattern seed and fixed Newton steps, using only + - * / for determinism.
static float b3FluidRoot( float x, int n )
{
	if ( !( x > 0.0f ) )
	{
		return 0.0f;
	}

	int32_t bits;
	memcpy( &bits, &x, sizeof( bits ) );
	bits = ( bits - 0x3F800000 ) / n + 0x3F800000;

	float y;
	memcpy( &y, &bits, sizeof( y ) );

	// Six steps reach float precision up to n = 48
	for ( int i = 0; i < 6; ++i )
	{
		float power = 1.0f;
		for ( int j = 1; j < n; ++j )
		{
			power *= y;
		}

		y = ( (float)( n - 1 ) * y + x / power ) / (float)n;
	}

	return y;
}

static float b3GetAxis( b3Vec3 v, int axis )
{
	return axis == 0 ? v.x : ( axis == 1 ? v.y : v.z );
}

static void b3SetAxis( b3Vec3* v, int axis, float value )
{
	if ( axis == 0 )
	{
		v->x = value;
	}
	else if ( axis == 1 )
	{
		v->y = value;
	}
	else
	{
		v->z = value;
	}
}

// Exact in both precision modes
static double b3GetPosAxis( b3Pos p, int axis )
{
	return axis == 0 ? p.x : ( axis == 1 ? p.y : p.z );
}

// The bounds axis most aligned with the surface normal
static int b3GetFluidAxis( b3Vec3 normal )
{
	float ax = b3AbsFloat( normal.x );
	float ay = b3AbsFloat( normal.y );
	float az = b3AbsFloat( normal.z );

	if ( ax >= ay && ax >= az )
	{
		return 0;
	}

	return ay >= az ? 1 : 2;
}

static float b3GetFluidSeparation( b3Plane plane, b3Pos point )
{
	return (float)( plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z -
					plane.offset );
}

// The point turned back about the bounds center is inside the footprint, see docs/fluid.md "Rotation"
static bool b3IsInRotatedFootprint( const b3FluidVolume* volume, b3Pos point )
{
	int axis = volume->axis;
	int axisA = axis == 2 ? 0 : axis + 1;
	int axisB = axis == 0 ? 2 : axis - 1;

	double lowerA = b3GetAxis( volume->bounds.lowerBound, axisA );
	double upperA = b3GetAxis( volume->bounds.upperBound, axisA );
	double lowerB = b3GetAxis( volume->bounds.lowerBound, axisB );
	double upperB = b3GetAxis( volume->bounds.upperBound, axisB );
	double halfA = 0.5 * ( upperA - lowerA );
	double halfB = 0.5 * ( upperB - lowerB );
	double da = b3GetPosAxis( point, axisA ) - 0.5 * ( lowerA + upperA );
	double db = b3GetPosAxis( point, axisB ) - 0.5 * ( lowerB + upperB );

	double c = volume->cosSin.cosine;
	double s = volume->cosSin.sine;
	double localA = c * da + s * db;
	double localB = c * db - s * da;
	return -halfA <= localA && localA <= halfA && -halfB <= localB && localB <= halfB;
}

// Inside the bounds off the vertical axis, at or above the bottom and at most radius above the top
static bool b3IsFluidCandidate( const b3FluidVolume* volume, b3Pos point, float radius,
								bool* contains )
{
	int axis = volume->axis;
	if ( volume->rotation == 0.0f )
	{
		for ( int i = 0; i < 3; ++i )
		{
			if ( i == axis )
			{
				continue;
			}

			double p = b3GetPosAxis( point, i );
			if ( ( b3GetAxis( volume->bounds.lowerBound, i ) <= p &&
				   p <= b3GetAxis( volume->bounds.upperBound, i ) ) == false )
			{
				return false;
			}
		}
	}
	else if ( b3IsInRotatedFootprint( volume, point ) == false )
	{
		return false;
	}

	double p = b3GetPosAxis( point, axis );
	float lower = b3GetAxis( volume->bounds.lowerBound, axis );
	float upper = b3GetAxis( volume->bounds.upperBound, axis );

	if ( b3GetAxis( volume->surface.normal, axis ) >= 0.0f )
	{
		if ( ( lower <= p && p <= upper + radius ) == false )
		{
			return false;
		}
	}
	else if ( ( lower - radius <= p && p <= upper ) == false )
	{
		return false;
	}

	*contains = lower <= p && p <= upper;
	return true;
}

// Binned: y axis and an ordered footprint inside B3_HUGE, narrower than half of it
static bool b3IsFluidVolumeBinned( const b3FluidVolume* volume )
{
	float huge = B3_HUGE;
	b3Vec3 lower = volume->aabb.lowerBound;
	b3Vec3 upper = volume->aabb.upperBound;
	bool insideX =
		-huge < lower.x && lower.x <= upper.x && upper.x < huge && upper.x - lower.x < 0.5f * huge;
	bool insideZ =
		-huge < lower.z && lower.z <= upper.z && upper.z < huge && upper.z - lower.z < 0.5f * huge;
	return volume->axis == 1 && insideX && insideZ;
}

// Rounds to float, saturating at the float limit instead of overflowing to infinity
static float b3SaturateToFloat( double value )
{
	if ( value < -FLT_MAX )
	{
		return -FLT_MAX;
	}

	return value > FLT_MAX ? FLT_MAX : (float)value;
}

// Caches the cosine and sine and the box around the turned footprint, padded past float rounding
static void b3UpdateFluidVolumeAABB( b3FluidVolume* volume )
{
	if ( volume->rotation == 0.0f )
	{
		volume->cosSin = ( b3CosSin ){ 1.0f, 0.0f };
		volume->aabb = volume->bounds;
		return;
	}

	volume->cosSin = b3ComputeCosSin( volume->rotation );

	int axis = volume->axis;
	int axisA = axis == 2 ? 0 : axis + 1;
	int axisB = axis == 0 ? 2 : axis - 1;

	double lowerA = b3GetAxis( volume->bounds.lowerBound, axisA );
	double upperA = b3GetAxis( volume->bounds.upperBound, axisA );
	double lowerB = b3GetAxis( volume->bounds.lowerBound, axisB );
	double upperB = b3GetAxis( volume->bounds.upperBound, axisB );
	double centerA = 0.5 * ( lowerA + upperA );
	double centerB = 0.5 * ( lowerB + upperB );
	double halfA = 0.5 * ( upperA - lowerA );
	double halfB = 0.5 * ( upperB - lowerB );

	double c = fabs( (double)volume->cosSin.cosine );
	double s = fabs( (double)volume->cosSin.sine );
	double extentA = c * halfA + s * halfB;
	double extentB = s * halfA + c * halfB;
	double padA = ( fabs( centerA ) + fabs( extentA ) ) / 65536.0 + FLT_TRUE_MIN;
	double padB = ( fabs( centerB ) + fabs( extentB ) ) / 65536.0 + FLT_TRUE_MIN;

	b3AABB aabb = volume->bounds;
	b3SetAxis( &aabb.lowerBound, axisA, b3SaturateToFloat( centerA - extentA - padA ) );
	b3SetAxis( &aabb.upperBound, axisA, b3SaturateToFloat( centerA + extentA + padA ) );
	b3SetAxis( &aabb.lowerBound, axisB, b3SaturateToFloat( centerB - extentB - padB ) );
	b3SetAxis( &aabb.upperBound, axisB, b3SaturateToFloat( centerB + extentB + padB ) );
	volume->aabb = aabb;
}

// Monotonic in p, so the cells of a footprint's ends bound the cell of every point inside it
static int b3GetFluidCell( double p, float lower, float inverseCellSize, int count )
{
	int index = (int)( ( p - lower ) * inverseCellSize );
	return b3MinInt( index, count - 1 );
}

void b3RebuildFluidLookup( b3FluidWorld* fluid )
{
	b3Array_Clear( fluid->unbinnedVolumes );
	b3Array_Clear( fluid->cellStarts );
	b3Array_Clear( fluid->cellVolumes );
	fluid->cellCountX = 0;
	fluid->cellCountZ = 0;

	b3AABB grid = { 0 };
	int binnedCount = 0;
	for ( int i = 0; i < fluid->volumes.count; ++i )
	{
		const b3FluidVolume* volume = fluid->volumes.data + i;
		if ( volume->id == B3_NULL_INDEX )
		{
			continue;
		}

		if ( b3IsFluidVolumeBinned( volume ) == false )
		{
			b3Array_Push( fluid->unbinnedVolumes, i );
			continue;
		}

		grid = binnedCount == 0 ? volume->aabb : b3AABB_Union( grid, volume->aabb );
		binnedCount += 1;
	}

	if ( binnedCount == 0 )
	{
		return;
	}

	// Square cells, about 1024 of them and at most 256 along a side
	float extentX = grid.upperBound.x - grid.lowerBound.x;
	float extentZ = grid.upperBound.z - grid.lowerBound.z;
	float cellSize = sqrtf( extentX * extentZ / 1024.0f );
	cellSize = b3MaxFloat( cellSize, b3MaxFloat( extentX, extentZ ) / 256.0f );
	if ( ( cellSize > 1.0e-30f ) == false )
	{
		cellSize = 1.0f;
	}

	float inverseCellSize = 1.0f / cellSize;
	int countX = b3MinInt( (int)( extentX * inverseCellSize ) + 1, 256 );
	int countZ = b3MinInt( (int)( extentZ * inverseCellSize ) + 1, 256 );
	fluid->gridLowerX = grid.lowerBound.x;
	fluid->gridLowerZ = grid.lowerBound.z;
	fluid->gridUpperX = grid.upperBound.x;
	fluid->gridUpperZ = grid.upperBound.z;
	fluid->inverseCellSize = inverseCellSize;
	fluid->cellCountX = countX;
	fluid->cellCountZ = countZ;

	int cellCount = countX * countZ;
	b3Array_Resize( fluid->cellStarts, cellCount + 1 );
	memset( fluid->cellStarts.data, 0, (size_t)( cellCount + 1 ) * sizeof( int ) );
	int* starts = fluid->cellStarts.data;

	// Count, take running sums, then fill backward so each cell lists ascending volume indices
	for ( int pass = 0; pass < 2; ++pass )
	{
		for ( int k = 0; k < fluid->volumes.count; ++k )
		{
			int i = pass == 0 ? k : fluid->volumes.count - 1 - k;
			const b3FluidVolume* volume = fluid->volumes.data + i;
			if ( volume->id == B3_NULL_INDEX || b3IsFluidVolumeBinned( volume ) == false )
			{
				continue;
			}

			b3AABB bounds = volume->aabb;
			float lowerX = grid.lowerBound.x;
			float lowerZ = grid.lowerBound.z;
			int x1 = b3GetFluidCell( bounds.lowerBound.x, lowerX, inverseCellSize, countX );
			int x2 = b3GetFluidCell( bounds.upperBound.x, lowerX, inverseCellSize, countX );
			int z1 = b3GetFluidCell( bounds.lowerBound.z, lowerZ, inverseCellSize, countZ );
			int z2 = b3GetFluidCell( bounds.upperBound.z, lowerZ, inverseCellSize, countZ );

			for ( int z = z1; z <= z2; ++z )
			{
				for ( int x = x1; x <= x2; ++x )
				{
					int cell = z * countX + x;
					if ( pass == 0 )
					{
						starts[cell] += 1;
					}
					else
					{
						starts[cell] -= 1;
						fluid->cellVolumes.data[starts[cell]] = i;
					}
				}
			}
		}

		if ( pass == 0 )
		{
			for ( int cell = 1; cell <= cellCount; ++cell )
			{
				starts[cell] += starts[cell - 1];
			}

			b3Array_Resize( fluid->cellVolumes, starts[cellCount] );
		}
	}
}

// The binned volumes whose footprint may hold the point, empty outside the grid
static void b3GetFluidCellRange( const b3FluidWorld* fluid, b3Pos point, int* start, int* end )
{
	*start = 0;
	*end = 0;

	double x = point.x;
	double z = point.z;
	if ( fluid->cellCountX == 0 || ( fluid->gridLowerX <= x && x <= fluid->gridUpperX &&
									 fluid->gridLowerZ <= z && z <= fluid->gridUpperZ ) == false )
	{
		return;
	}

	int cellX = b3GetFluidCell( x, fluid->gridLowerX, fluid->inverseCellSize, fluid->cellCountX );
	int cellZ = b3GetFluidCell( z, fluid->gridLowerZ, fluid->inverseCellSize, fluid->cellCountZ );
	int cell = cellZ * fluid->cellCountX + cellX;
	*start = fluid->cellStarts.data[cell];
	*end = fluid->cellStarts.data[cell + 1];
}

typedef struct b3FluidCandidate
{
	int index;
	bool contains;
	float separation;
} b3FluidCandidate;

// Containing bounds win, then the highest surface, then the lowest index, in any visiting order
static void b3TestFluidVolume( const b3FluidWorld* fluid, int index, b3Pos point, float radius,
							   b3FluidCandidate* best )
{
	bool contains = false;
	const b3FluidVolume* volume = fluid->volumes.data + index;
	if ( b3IsFluidCandidate( volume, point, radius, &contains ) == false )
	{
		return;
	}

	float s = b3GetFluidSeparation( volume->surface, point );
	if ( best->index == B3_NULL_INDEX || ( contains && best->contains == false ) ||
		 ( contains == best->contains &&
		   ( s < best->separation || ( s == best->separation && index < best->index ) ) ) )
	{
		best->index = index;
		best->contains = contains;
		best->separation = s;
	}
}

// The chosen volume for a center of mass, see docs/fluid.md "Classification"; writes its separation
static int b3FindFluidVolumeIndex( const b3FluidWorld* fluid, b3Pos point, float radius,
								   float* separation )
{
	b3FluidCandidate best = { B3_NULL_INDEX, false, 0.0f };

	for ( int i = 0; i < fluid->unbinnedVolumes.count; ++i )
	{
		b3TestFluidVolume( fluid, fluid->unbinnedVolumes.data[i], point, radius, &best );
	}

	int start, end;
	b3GetFluidCellRange( fluid, point, &start, &end );
	for ( int i = start; i < end; ++i )
	{
		b3TestFluidVolume( fluid, fluid->cellVolumes.data[i], point, radius, &best );
	}

	*separation = best.separation;
	return best.index;
}

// Whether a volume other than the excluded one is a candidate for the point
static bool b3HasOtherFluidCandidate( const b3FluidWorld* fluid, b3Pos point, float radius,
									  int excludeIndex )
{
	if ( b3GetIdCount( &fluid->volumeIdPool ) == 1 )
	{
		return false;
	}

	bool contains;
	for ( int i = 0; i < fluid->unbinnedVolumes.count; ++i )
	{
		int index = fluid->unbinnedVolumes.data[i];
		if ( index != excludeIndex &&
			 b3IsFluidCandidate( fluid->volumes.data + index, point, radius, &contains ) )
		{
			return true;
		}
	}

	int start, end;
	b3GetFluidCellRange( fluid, point, &start, &end );
	for ( int i = start; i < end; ++i )
	{
		int index = fluid->cellVolumes.data[i];
		if ( index != excludeIndex &&
			 b3IsFluidCandidate( fluid->volumes.data + index, point, radius, &contains ) )
		{
			return true;
		}
	}

	return false;
}

static float b3GetSphereVolume( float radius )
{
	return ( 4.0f / 3.0f ) * B3_PI * radius * radius * radius;
}

static float b3GetCapsuleVolume( const b3Capsule* capsule )
{
	float radius = capsule->radius;
	float length = b3Distance( capsule->center1, capsule->center2 );
	return B3_PI * radius * radius * length + b3GetSphereVolume( radius );
}

// Submerged volume and first moment about the local center; body frame plane, fluid below
static float b3SubmergeSphere( b3Vec3 center, float radius, b3Plane plane, b3Vec3 localCenter,
							   b3Vec3* moment, bool* crossed )
{
	float distance = b3PlaneSeparation( plane, center );
	if ( distance >= radius )
	{
		*moment = b3Vec3_zero;
		return 0.0f;
	}

	float volume = b3GetSphereVolume( radius );
	b3Vec3 offset = b3Sub( center, localCenter );
	if ( distance <= -radius )
	{
		*moment = b3MulSV( volume, offset );
		return volume;
	}

	*crossed = true;

	// Spherical cap of height h and its first moment about the sphere center
	float h = b3ClampFloat( radius - distance, 0.0f, 2.0f * radius );
	float u = 2.0f * radius - h;
	float capVolume = B3_PI * h * h * ( 3.0f * radius - h ) / 3.0f;
	float capMoment = 0.25f * B3_PI * h * h * u * u;

	capVolume = b3ClampFloat( capVolume, 0.0f, volume );
	*moment = b3MulSub( b3MulSV( capVolume, offset ), capMoment, plane.normal );
	return capVolume;
}

// Axial integral over a capsule and its caps; s = 0 at center1, disc offset distance1 + cosine * s
typedef struct b3CapsuleIntegral
{
	float radius;
	float length;
	float cosine;
	float sine;
	float distance1;
	float volume;
	float axialMoment;
	float lateralMoment;
} b3CapsuleIntegral;

// Piece 0 is the cap at center1, piece 1 the cylinder and piece 2 the cap at center2
static float b3GetCapsuleDiscRadius( const b3CapsuleIntegral* integral, int piece, float s )
{
	float radius = integral->radius;
	if ( piece == 1 )
	{
		return radius;
	}

	float t = piece == 0 ? s : s - integral->length;
	return sqrtf( b3MaxFloat( radius * radius - t * t, 0.0f ) );
}

// Fully wet range in closed form
static void b3AddWetCapsuleRange( b3CapsuleIntegral* integral, int piece, float a, float b )
{
	float radius = integral->radius;
	if ( piece == 1 )
	{
		float area = B3_PI * radius * radius;
		integral->volume += area * ( b - a );
		integral->axialMoment += 0.5f * area * ( b - a ) * ( b + a );
		return;
	}

	// pi * (r^2 t - t^3 / 3) and pi * (r^2 t^2 / 2 - t^4 / 4) about the cap center
	float offset = piece == 0 ? 0.0f : integral->length;
	float ta = a - offset;
	float tb = b - offset;
	float volume =
		B3_PI * ( tb - ta ) * ( radius * radius - ( ta * ta + ta * tb + tb * tb ) / 3.0f );
	float moment = B3_PI * ( tb - ta ) * ( tb + ta ) *
				   ( 0.5f * radius * radius - 0.25f * ( ta * ta + tb * tb ) );
	integral->volume += volume;
	integral->axialMoment += moment + offset * volume;
}

// Partly wet range: circular segments below the waterline by quadrature
static void b3AddCapsuleBand( b3CapsuleIntegral* integral, int piece, float a, float b )
{
	float mid = 0.5f * ( a + b );
	float half = 0.5f * ( b - a );

	for ( int i = 0; i < B3_FLUID_QUADRATURE_COUNT; ++i )
	{
		float s = mid + half * b3_fluidNodes[i];
		float weight = half * b3_fluidWeights[i];
		float rho = b3GetCapsuleDiscRadius( integral, piece, s );

		// Distance from the disc center to the waterline chord, in the disc plane
		float h = b3ClampFloat( ( integral->distance1 + integral->cosine * s ) / integral->sine,
								-rho, rho );
		float q = b3MaxFloat( rho * rho - h * h, 0.0f );
		float root = sqrtf( q );

		float area = rho * rho * b3Atan2( root, h ) - h * root;
		integral->volume += weight * area;
		integral->axialMoment += weight * s * area;
		integral->lateralMoment += weight * ( 2.0f / 3.0f ) * q * root;
	}
}

// Splits a piece at its band ends and center crossing, integrating each range by its midpoint
static void b3IntegrateCapsulePiece( b3CapsuleIntegral* integral, int piece, float lower,
									 float upper )
{
	float radius = integral->radius;
	float cosine = integral->cosine;
	float sine = integral->sine;
	float distance1 = integral->distance1;

	float splits[3];
	int splitCount = 0;

	if ( cosine != 0.0f )
	{
		splits[splitCount++] = -distance1 / cosine;
	}

	if ( piece == 1 )
	{
		if ( cosine != 0.0f )
		{
			splits[splitCount++] = ( -distance1 - radius * sine ) / cosine;
			splits[splitCount++] = ( -distance1 + radius * sine ) / cosine;
		}
	}
	else
	{
		// The band on a cap sphere solves (d + cosine * t)^2 = (r^2 - t^2) * sine^2
		float center = piece == 0 ? 0.0f : integral->length;
		float distance = distance1 + cosine * center;
		float discriminant = radius * radius - distance * distance;
		if ( discriminant > 0.0f )
		{
			float root = sine * sqrtf( discriminant );
			splits[splitCount++] = center - distance * cosine - root;
			splits[splitCount++] = center - distance * cosine + root;
		}
	}

	for ( int i = 1; i < splitCount; ++i )
	{
		float value = splits[i];
		int j = i - 1;
		while ( j >= 0 && splits[j] > value )
		{
			splits[j + 1] = splits[j];
			j -= 1;
		}
		splits[j + 1] = value;
	}

	float a = lower;
	for ( int i = 0; i <= splitCount; ++i )
	{
		float b = i < splitCount ? b3ClampFloat( splits[i], lower, upper ) : upper;
		if ( ( b > a ) == false )
		{
			continue;
		}

		float mid = 0.5f * ( a + b );
		float reach = b3GetCapsuleDiscRadius( integral, piece, mid ) * sine;
		float distance = distance1 + cosine * mid;
		if ( distance + reach <= 0.0f )
		{
			b3AddWetCapsuleRange( integral, piece, a, b );
		}
		else if ( distance - reach < 0.0f )
		{
			b3AddCapsuleBand( integral, piece, a, b );
		}

		a = b;
	}
}

static float b3SubmergeCapsule( const b3Capsule* capsule, b3Plane plane, b3Vec3 localCenter,
								b3Vec3* moment, bool* crossed )
{
	b3Vec3 center1 = capsule->center1;
	b3Vec3 center2 = capsule->center2;
	float radius = capsule->radius;

	float distance1 = b3PlaneSeparation( plane, center1 );
	float distance2 = b3PlaneSeparation( plane, center2 );
	if ( b3MinFloat( distance1, distance2 ) >= radius )
	{
		*moment = b3Vec3_zero;
		return 0.0f;
	}

	if ( b3MaxFloat( distance1, distance2 ) <= -radius )
	{
		float volume = b3GetCapsuleVolume( capsule );
		b3Vec3 center = b3Lerp( center1, center2, 0.5f );
		*moment = b3MulSV( volume, b3Sub( center, localCenter ) );
		return volume;
	}

	float length;
	b3Vec3 axis = b3GetLengthAndNormalize( &length, b3Sub( center2, center1 ) );
	if ( length == 0.0f )
	{
		return b3SubmergeSphere( center1, radius, plane, localCenter, moment, crossed );
	}

	*crossed = true;

	float cosine = b3Dot( plane.normal, axis );
	b3Vec3 lateral = b3MulSub( plane.normal, cosine, axis );
	float sine = b3Length( lateral );

	b3CapsuleIntegral integral = { radius, length, cosine, sine, distance1, 0.0f, 0.0f, 0.0f };
	b3IntegrateCapsulePiece( &integral, 0, -radius, 0.0f );
	b3IntegrateCapsulePiece( &integral, 1, 0.0f, length );
	b3IntegrateCapsulePiece( &integral, 2, length, length + radius );

	float volume = b3ClampFloat( integral.volume, 0.0f, b3GetCapsuleVolume( capsule ) );
	if ( !( volume > 0.0f ) )
	{
		*moment = b3Vec3_zero;
		return 0.0f;
	}

	// The segments lie on the fluid side of the axis, opposite the lateral direction
	b3Vec3 m = b3MulSV( integral.axialMoment, axis );
	if ( integral.lateralMoment > 0.0f )
	{
		m = b3MulSub( m, integral.lateralMoment / sine, lateral );
	}

	*moment = b3MulAdd( m, integral.volume, b3Sub( center1, localCenter ) );
	return volume;
}

// Fan accumulator for clipped hull faces, with vertices relative to the apex
typedef struct b3FluidFan
{
	b3Vec3 first;
	b3Vec3 previous;
	int count;
	float volume;
	b3Vec3 moment;
} b3FluidFan;

static void b3AddFanPoint( b3FluidFan* fan, b3Vec3 p )
{
	if ( fan->count == 0 )
	{
		fan->first = p;
	}
	else if ( fan->count > 1 )
	{
		float det = b3ScalarTripleProduct( fan->first, fan->previous, p );
		fan->volume += det;
		fan->moment = b3MulAdd( fan->moment, det, b3Add( fan->first, b3Add( fan->previous, p ) ) );
	}

	fan->previous = p;
	fan->count += 1;
}

// Edge crossing computed from the vertex below to the vertex above, so both faces of an edge agree
static b3Vec3 b3GetHullCrossing( const b3Vec3* points, const float* distances, int index1,
								 int index2 )
{
	if ( distances[index1] >= 0.0f )
	{
		int temp = index1;
		index1 = index2;
		index2 = temp;
	}

	float distance1 = distances[index1];
	float distance2 = distances[index2];
	float t = distance1 / ( distance1 - distance2 );
	return b3MulAdd( points[index1], t, b3Sub( points[index2], points[index1] ) );
}

static float b3SubmergeHull( const b3HullData* hull, b3Plane plane, b3Vec3 localCenter,
							 b3Vec3* moment, bool* crossed )
{
	const b3Vec3* points = b3GetHullPoints( hull );
	int vertexCount = hull->vertexCount;
	B3_ASSERT( vertexCount <= B3_MAX_HULL_VERTICES );

	float distances[B3_MAX_HULL_VERTICES];
	int belowCount = 0;
	for ( int i = 0; i < vertexCount; ++i )
	{
		distances[i] = b3PlaneSeparation( plane, points[i] );
		belowCount += distances[i] < 0.0f ? 1 : 0;
	}

	if ( belowCount == 0 )
	{
		*moment = b3Vec3_zero;
		return 0.0f;
	}

	if ( belowCount == vertexCount )
	{
		*moment = b3MulSV( hull->volume, b3Sub( hull->center, localCenter ) );
		return hull->volume;
	}

	*crossed = true;

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );

	// Apex at the mean edge crossing, on the cap polygon, so the cap's tetrahedra vanish
	b3Vec3 apex = b3Vec3_zero;
	int crossingCount = 0;
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		const b3HullHalfEdge* edge = edges + i;
		if ( edge->twin < i )
		{
			continue;
		}

		int index1 = edge->origin;
		int index2 = edges[edge->twin].origin;
		if ( ( distances[index1] < 0.0f ) == ( distances[index2] < 0.0f ) )
		{
			continue;
		}

		apex = b3Add( apex, b3GetHullCrossing( points, distances, index1, index2 ) );
		crossingCount += 1;
	}

	if ( crossingCount == 0 )
	{
		*moment = b3Vec3_zero;
		return 0.0f;
	}

	apex = b3MulSV( 1.0f / (float)crossingCount, apex );

	// Clip each face polygon to the fluid side and fan it into tetrahedra with the apex
	b3FluidFan fan = { 0 };
	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		const b3HullHalfEdge* first = edges + faces[faceIndex].edge;
		const b3HullHalfEdge* edge = first;
		fan.count = 0;

		do
		{
			const b3HullHalfEdge* next = edges + edge->next;
			int index1 = edge->origin;
			int index2 = next->origin;
			bool below1 = distances[index1] < 0.0f;
			bool below2 = distances[index2] < 0.0f;

			if ( below1 )
			{
				b3AddFanPoint( &fan, b3Sub( points[index1], apex ) );
			}

			if ( below1 != below2 )
			{
				b3AddFanPoint(
					&fan, b3Sub( b3GetHullCrossing( points, distances, index1, index2 ), apex ) );
			}

			edge = next;
		}
		while ( edge != first );
	}

	float volume = fan.volume / 6.0f;
	float clampedVolume = b3ClampFloat( volume, 0.0f, hull->volume );
	if ( !( clampedVolume > 0.0f ) )
	{
		*moment = b3Vec3_zero;
		return 0.0f;
	}

	b3Vec3 apexMoment = b3MulSV( 1.0f / 24.0f, fan.moment );
	*moment = b3MulAdd( apexMoment, volume, b3Sub( apex, localCenter ) );
	return clampedVolume;
}

typedef struct b3FluidSample
{
	// Geometric volume of the buoyant shapes
	float volume;

	// Submerged volume and its first moment about the center of mass, in the body frame
	float submergedVolume;
	b3Vec3 moment;

	int crossedCount;
} b3FluidSample;

// Buoyant shapes are non-sensor spheres, capsules and hulls whose category bits match the mask
static void b3SampleFluidBody( const b3World* world, const b3Body* body, b3Plane plane,
							   b3Vec3 localCenter, uint64_t maskBits, b3FluidSample* sample )
{
	const b3Shape* shapes = world->shapes.data;
	int shapeId = body->headShapeId;
	while ( shapeId != B3_NULL_INDEX )
	{
		const b3Shape* shape = shapes + shapeId;
		shapeId = shape->nextShapeId;

		if ( shape->sensorIndex != B3_NULL_INDEX || ( shape->filter.categoryBits & maskBits ) == 0 )
		{
			continue;
		}

		float shapeVolume = 0.0f;
		float submergedVolume = 0.0f;
		b3Vec3 moment = b3Vec3_zero;
		bool crossed = false;

		if ( shape->type == b3_sphereShape )
		{
			shapeVolume = b3GetSphereVolume( shape->sphere.radius );
			b3Sphere sphere = shape->sphere;
			submergedVolume = b3SubmergeSphere( sphere.center, sphere.radius, plane, localCenter,
												&moment, &crossed );
		}
		else if ( shape->type == b3_capsuleShape )
		{
			shapeVolume = b3GetCapsuleVolume( &shape->capsule );
			submergedVolume =
				b3SubmergeCapsule( &shape->capsule, plane, localCenter, &moment, &crossed );
		}
		else if ( shape->type == b3_hullShape )
		{
			shapeVolume = shape->hull->volume;
			submergedVolume = b3SubmergeHull( shape->hull, plane, localCenter, &moment, &crossed );
		}
		else
		{
			continue;
		}

		sample->volume += shapeVolume;
		sample->submergedVolume += submergedVolume;
		sample->moment = b3Add( sample->moment, moment );
		sample->crossedCount += crossed ? 1 : 0;
	}
}

b3FluidWorld* b3GetOrCreateFluidWorld( b3World* world )
{
	if ( world->fluid == NULL )
	{
		b3FluidWorld* fluid = (b3FluidWorld*)b3AllocZero( sizeof( b3FluidWorld ) );
		fluid->volumeIdPool = b3CreateIdPool();
		world->fluid = fluid;
	}

	return world->fluid;
}

void b3DestroyFluidWorld( b3World* world )
{
	b3FluidWorld* fluid = world->fluid;
	if ( fluid == NULL )
	{
		return;
	}

	for ( int i = 0; i < fluid->taskContexts.count; ++i )
	{
		b3DestroyBitSet( &fluid->taskContexts.data[i].changedBits );
	}

	b3Array_Destroy( fluid->taskContexts );
	b3Array_Destroy( fluid->volumes );
	b3Array_Destroy( fluid->unbinnedVolumes );
	b3Array_Destroy( fluid->cellStarts );
	b3Array_Destroy( fluid->cellVolumes );
	b3Array_Destroy( fluid->bodies );
	b3Array_Destroy( fluid->beginEvents );
	b3Array_Destroy( fluid->endEvents[0] );
	b3Array_Destroy( fluid->endEvents[1] );
	b3DestroyIdPool( &fluid->volumeIdPool );

	b3Free( fluid, sizeof( b3FluidWorld ) );
	world->fluid = NULL;
}

int b3GetFluidByteCount( const b3World* world )
{
	b3FluidWorld* fluid = world->fluid;
	if ( fluid == NULL )
	{
		return 0;
	}

	int bytes = (int)sizeof( b3FluidWorld ) + b3GetIdBytes( &fluid->volumeIdPool );
	bytes += b3Array_ByteCount( fluid->volumes ) + b3Array_ByteCount( fluid->bodies );
	bytes += b3Array_ByteCount( fluid->unbinnedVolumes ) + b3Array_ByteCount( fluid->cellStarts );
	bytes += b3Array_ByteCount( fluid->cellVolumes );
	bytes += b3Array_ByteCount( fluid->taskContexts ) + b3Array_ByteCount( fluid->beginEvents );
	bytes += b3Array_ByteCount( fluid->endEvents[0] ) + b3Array_ByteCount( fluid->endEvents[1] );

	for ( int i = 0; i < fluid->taskContexts.count; ++i )
	{
		bytes += b3GetBitSetBytes( &fluid->taskContexts.data[i].changedBits );
	}

	return bytes;
}

// Grows the side table to cover count body ids. New entries are zero: default def, not in fluid.
static void b3GrowFluidBodies( b3FluidWorld* fluid, int count )
{
	int oldCount = fluid->bodies.count;
	if ( count <= oldCount )
	{
		return;
	}

	if ( count > fluid->bodies.capacity )
	{
		int capacity = b3MaxInt( count, fluid->bodies.capacity + fluid->bodies.capacity / 2 );
		b3Array_Reserve( fluid->bodies, capacity );
	}

	b3Array_Resize( fluid->bodies, count );
	memset( fluid->bodies.data + oldCount, 0,
			(size_t)( count - oldCount ) * sizeof( b3FluidBody ) );
}

static b3FluidBody* b3GetFluidBody( b3World* world, int bodyId )
{
	b3FluidWorld* fluid = world->fluid;
	if ( fluid == NULL || bodyId >= fluid->bodies.count )
	{
		return NULL;
	}

	return fluid->bodies.data + bodyId;
}

static b3BodyFluidDef b3GetEntryFluidDef( const b3FluidBody* entry )
{
	b3BodyFluidDef def = b3DefaultBodyFluidDef();
	if ( entry != NULL && ( entry->flags & b3_fluidHasDef ) )
	{
		def.displacedVolume = entry->displacedVolume;
		def.dragScale = entry->dragScale;
		def.enabled = ( entry->flags & b3_fluidDisabled ) == 0;
	}

	return def;
}

static b3FluidVolumeId b3MakeFluidVolumeId( b3World* world, int volumeIndex )
{
	b3FluidVolume* volume = b3Array_Get( world->fluid->volumes, volumeIndex );
	return (b3FluidVolumeId){ volumeIndex + 1, world->worldId, volume->generation };
}

static b3FluidVolume* b3GetFluidVolumeFullId( b3World* world, b3FluidVolumeId volumeId )
{
	B3_ASSERT( b3FluidVolume_IsValid( volumeId ) );
	return b3Array_Get( world->fluid->volumes, volumeId.index1 - 1 );
}

void b3FluidRemoveBody( b3World* world, int bodyId )
{
	b3FluidBody* entry = b3GetFluidBody( world, bodyId );
	if ( entry == NULL )
	{
		return;
	}

	if ( B3_IS_NON_NULL( entry->volumeId ) )
	{
		b3FluidEndTouchEvent event = { b3MakeBodyId( world, bodyId ), entry->volumeId };
		b3Array_Push( world->fluid->endEvents[world->endEventArrayIndex], event );
	}

	entry->volumeId = b3_nullFluidVolumeId;
	entry->fraction = 0.0f;
}

void b3FluidDestroyBody( b3World* world, int bodyId )
{
	b3FluidRemoveBody( world, bodyId );

	b3FluidBody* entry = b3GetFluidBody( world, bodyId );
	if ( entry != NULL )
	{
		memset( entry, 0, sizeof( b3FluidBody ) );
	}
}

void b3FluidSetBodyType( b3World* world, int bodyId, b3BodyType type )
{
	if ( type != b3_dynamicBody )
	{
		b3FluidRemoveBody( world, bodyId );
	}
}

typedef struct b3FluidStepContext
{
	b3World* world;
	b3FluidWorld* fluid;
	b3BodySim* sims;
	b3BodyState* states;
	float dt;
	float h;
	int subStepCount;

	// Predicted pose lead, beta * dt
	float lead;
} b3FluidStepContext;

static void b3ExitFluid( b3FluidTaskContext* taskContext, b3FluidBody* entry, int bodyId )
{
	entry->stepVolumeId = b3_nullFluidVolumeId;
	entry->fraction = 0.0f;
	b3SetBit( &taskContext->changedBits, (uint32_t)bodyId );
}

static void b3FluidTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b3TracyCZoneNC( fluid_task, "Fluid", b3_colorDodgerBlue, true );

	b3FluidStepContext* stepContext = (b3FluidStepContext*)context;
	b3World* world = stepContext->world;
	b3FluidWorld* fluid = stepContext->fluid;
	b3FluidTaskContext* taskContext = fluid->taskContexts.data + workerIndex;
	b3FluidBody* entries = fluid->bodies.data;
	b3Body* bodies = world->bodies.data;
	b3Vec3 gravity = world->gravity;
	float h = stepContext->h;
	float lead = stepContext->lead;

	for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
	{
		b3BodySim* sim = stepContext->sims + simIndex;
		const b3BodyState* state = stepContext->states + simIndex;
		int bodyId = sim->bodyId;
		b3FluidBody* entry = entries + bodyId;
		bool wasInFluid = B3_IS_NON_NULL( entry->volumeId );

		if ( ( sim->flags & b3_dynamicFlag ) == 0 || ( sim->invMass > 0.0f ) == false ||
			 ( entry->flags & b3_fluidDisabled ) )
		{
			if ( wasInFluid )
			{
				b3ExitFluid( taskContext, entry, bodyId );
			}
			continue;
		}

		// Predicted center of mass, see docs/fluid.md "The predicted pose"
		b3Vec3 v = state->linearVelocity;
		b3Pos center = b3OffsetPos( sim->center, b3MulSV( lead, v ) );
		float radius = b3Length( sim->maxExtent );

		float separation;
		int volumeIndex = b3FindFluidVolumeIndex( fluid, center, radius, &separation );
		if ( volumeIndex == B3_NULL_INDEX || separation >= radius )
		{
			if ( wasInFluid )
			{
				b3ExitFluid( taskContext, entry, bodyId );
			}
			continue;
		}

		const b3FluidVolume* volume = fluid->volumes.data + volumeIndex;
		b3Vec3 w = state->angularVelocity;
		b3Quat q = lead > 0.0f ? b3IntegrateRotation( sim->transform.q, b3MulSV( lead, w ) )
							   : sim->transform.q;

		// Surface plane in the body frame
		b3Vec3 normal = volume->surface.normal;
		b3Vec3 localNormal = b3InvRotateVector( q, normal );
		b3Plane localPlane = { localNormal, b3Dot( localNormal, sim->localCenter ) - separation };

		b3FluidSample sample = { 0 };
		b3SampleFluidBody( world, bodies + bodyId, localPlane, sim->localCenter, volume->maskBits,
						   &sample );
		taskContext->clippedShapeCount += sample.crossedCount;

		float displacedVolume = ( entry->flags & b3_fluidHasDef ) ? entry->displacedVolume : 0.0f;
		if ( displacedVolume == 0.0f )
		{
			displacedVolume = sample.volume;
		}

		float fraction = 0.0f;
		if ( sample.volume > 0.0f )
		{
			fraction = b3ClampFloat( sample.submergedVolume / sample.volume, 0.0f, 1.0f );
		}

		if ( !( fraction > 0.0f ) || !( displacedVolume > 0.0f ) )
		{
			if ( wasInFluid )
			{
				b3ExitFluid( taskContext, entry, bodyId );
			}
			continue;
		}

		float density = volume->density;
		float gravityScale = sim->gravityScale;
		float dragScale = ( entry->flags & b3_fluidHasDef ) ? entry->dragScale : 1.0f;
		float mass = 1.0f / sim->invMass;
		b3Vec3 flow = volume->flowVelocity;
		b3Vec3 userForce = sim->force;

		// Buoyancy, with the torque from the submerged first moment
		float buoyancyScale = -density * displacedVolume * gravityScale;
		b3Vec3 buoyancy = b3MulSV( buoyancyScale * fraction, gravity );
		b3Vec3 arm = b3RotateVector( q, sample.moment );
		b3Vec3 torque = b3Cross( arm, b3MulSV( buoyancyScale / sample.volume, gravity ) );

		// Substeps map torque through the start-of-step inertia, so give it the predicted response
		if ( lead > 0.0f )
		{
			b3Quat q0 = sim->transform.q;
			b3Vec3 acceleration = b3RotateVector(
				q, b3MulMV( sim->invInertiaLocal, b3InvRotateVector( q, torque ) ) );
			torque = b3RotateVector(
				q0, b3MulMV( sim->inertia, b3InvRotateVector( q0, acceleration ) ) );
		}

		// Drag rates, with area over mass for the quadratic term
		float side = b3FluidRoot( fraction * sample.volume, 3 );
		float linearRate = dragScale * fraction * volume->linearDrag;
		float quadraticScale =
			dragScale * 0.5f * density * volume->quadraticDrag * side * side * sim->invMass;
		float speed = b3Length( b3Sub( v, flow ) );
		float k = linearRate + quadraticScale * speed;

		// Substeps hold the quadratic rate, so use the speed whose held rate gives the step's decay
		int subStepCount = stepContext->subStepCount;
		if ( subStepCount > 1 && quadraticScale * speed > 0.0f )
		{
			float dt = stepContext->dt;
			b3Vec3 a = b3MulAdd( b3MulSV( gravityScale, gravity ), sim->invMass,
								 b3Add( userForce, buoyancy ) );
			b3Vec3 predicted = b3MulAdd( v, dt, b3MulAdd( a, k, flow ) );
			predicted = b3MulSV( 1.0f / ( 1.0f + dt * ( sim->linearDamping + k ) ), predicted );
			float predictedSpeed = b3Length( b3Sub( predicted, flow ) );
			float ratio = predictedSpeed > 0.0f
							  ? b3ClampFloat( speed / predictedSpeed, 1.0e-6f, 1.0e6f )
							  : 1.0e6f;

			float y = b3FluidRoot( ratio, subStepCount );
			float power = 1.0f;
			float sum = 1.0f;
			for ( int i = 1; i < subStepCount; ++i )
			{
				power *= y;
				sum += power;
			}

			speed = (float)subStepCount * speed / sum;
			k = linearRate + quadraticScale * speed;
		}

		float ka = dragScale * fraction * volume->angularDrag;

		// Backward Euler on all non-constraint accelerations, see docs/fluid.md
		float linearDamping = sim->linearDamping + k;
		float angularDamping = sim->angularDamping + ka;
		b3Vec3 heldForce = b3Add( userForce, b3MulSV( mass * gravityScale, gravity ) );
		b3Vec3 force = b3MulAdd( buoyancy, mass * k, flow );
		force = b3MulSub( force, h * linearDamping, heldForce );
		force = b3MulSV( 1.0f / ( 1.0f + h * linearDamping ), force );

		entry->linearDamping = sim->linearDamping;
		entry->angularDamping = sim->angularDamping;
		entry->flags |= b3_fluidDamped;

		sim->linearDamping = linearDamping;
		sim->angularDamping = angularDamping;
		sim->force = b3Add( userForce, force );
		sim->torque = b3MulSV( 1.0f / ( 1.0f + h * angularDamping ), b3Add( sim->torque, torque ) );

		b3FluidVolumeId volumeId = { volumeIndex + 1, world->worldId, volume->generation };
		entry->fraction = fraction;
		entry->stepVolumeId = volumeId;

		if ( B3_ID_EQUALS( entry->volumeId, volumeId ) == false )
		{
			// Begin payload: submerged centroid projected onto the surface, velocity before drag
			b3Vec3 centroid = b3MulSV( 1.0f / sample.submergedVolume, arm );
			float height = separation + b3Dot( normal, centroid );
			entry->point = b3OffsetPos( center, b3MulSub( centroid, height, normal ) );
			entry->velocity = v;
		}

		b3SetBit( &taskContext->changedBits, (uint32_t)bodyId );
		taskContext->bodyCount += 1;
	}

	b3TracyCZoneEnd( fluid_task );
}

void b3FluidPreSolve( b3World* world, b3StepContext* context )
{
	b3FluidWorld* fluid = world->fluid;
	if ( fluid == NULL )
	{
		return;
	}

	// Ensure the user does not see stale begin events if there is an early return
	b3Array_Clear( fluid->beginEvents );
	fluid->stats = (b3FluidStats){ 0 };
	fluid->stats.volumeCount = b3GetIdCount( &fluid->volumeIdPool );

	if ( fluid->stats.volumeCount == 0 || ( context->dt > 0.0f ) == false )
	{
		return;
	}

	uint64_t ticks = b3GetTicks();

	int bodyCapacity = b3GetIdCapacity( &world->bodyIdPool );
	b3GrowFluidBodies( fluid, bodyCapacity );

	int workerCount = world->workerCount;
	if ( fluid->taskContexts.count != workerCount )
	{
		for ( int i = 0; i < fluid->taskContexts.count; ++i )
		{
			b3DestroyBitSet( &fluid->taskContexts.data[i].changedBits );
		}

		b3Array_Resize( fluid->taskContexts, workerCount );
		memset( fluid->taskContexts.data, 0, (size_t)workerCount * sizeof( b3FluidTaskContext ) );
	}

	for ( int i = 0; i < workerCount; ++i )
	{
		b3FluidTaskContext* taskContext = fluid->taskContexts.data + i;
		b3SetBitCountAndClear( &taskContext->changedBits, (uint32_t)bodyCapacity );
		taskContext->bodyCount = 0;
		taskContext->clippedShapeCount = 0;
	}

	int subStepCount = context->subStepCount;
	float beta = (float)( subStepCount - 1 ) / (float)( 2 * subStepCount );

	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	b3FluidStepContext stepContext = {
		.world = world,
		.fluid = fluid,
		.sims = awakeSet->bodySims.data,
		.states = awakeSet->bodyStates.data,
		.dt = context->dt,
		.h = context->h,
		.subStepCount = subStepCount,
		.lead = beta * context->dt,
	};

	int minRange = 32;
	b3ParallelFor( world, b3FluidTask, awakeSet->bodySims.count, minRange, &stepContext, "fluid" );

	fluid->stepActive = true;
	fluid->stats.time = b3GetMilliseconds( ticks );
}

void b3FluidPostSolve( b3World* world )
{
	b3FluidWorld* fluid = world->fluid;
	if ( fluid == NULL || fluid->stepActive == false )
	{
		return;
	}

	uint64_t ticks = b3GetTicks();

	b3FluidTaskContext* taskContexts = fluid->taskContexts.data;
	b3BitSet* bitSet = &taskContexts[0].changedBits;
	int bodyCount = taskContexts[0].bodyCount;
	int clippedShapeCount = taskContexts[0].clippedShapeCount;
	for ( int i = 1; i < fluid->taskContexts.count; ++i )
	{
		b3InPlaceUnion( bitSet, &taskContexts[i].changedBits );
		bodyCount += taskContexts[i].bodyCount;
		clippedShapeCount += taskContexts[i].clippedShapeCount;
	}

	// Restore damping and publish events in body id order
	b3Array( b3FluidEndTouchEvent )* endEvents = fluid->endEvents + world->endEventArrayIndex;
	uint64_t* bits = bitSet->bits;
	uint32_t blockCount = bitSet->blockCount;
	for ( uint32_t k = 0; k < blockCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = b3CTZ64( word );
			int bodyId = (int)( 64 * k + ctz );

			b3FluidBody* entry = fluid->bodies.data + bodyId;
			if ( entry->flags & b3_fluidDamped )
			{
				// The body may have moved to a sleeping set
				b3Body* body = b3Array_Get( world->bodies, bodyId );
				b3BodySim* sim = b3GetBodySim( world, body );
				sim->linearDamping = entry->linearDamping;
				sim->angularDamping = entry->angularDamping;
				entry->linearDamping = 0.0f;
				entry->angularDamping = 0.0f;
				entry->flags &= ~b3_fluidDamped;
			}

			if ( B3_ID_EQUALS( entry->volumeId, entry->stepVolumeId ) == false )
			{
				b3BodyId fullId = b3MakeBodyId( world, bodyId );
				if ( B3_IS_NON_NULL( entry->volumeId ) )
				{
					b3FluidEndTouchEvent event = { fullId, entry->volumeId };
					b3Array_Push( *endEvents, event );
				}

				if ( B3_IS_NON_NULL( entry->stepVolumeId ) )
				{
					b3FluidBeginTouchEvent event = { fullId, entry->stepVolumeId, entry->point,
													 entry->velocity };
					b3Array_Push( fluid->beginEvents, event );
				}

				entry->volumeId = entry->stepVolumeId;
			}

			entry->stepVolumeId = b3_nullFluidVolumeId;
			entry->point = b3Pos_zero;
			entry->velocity = b3Vec3_zero;

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}

	fluid->stepActive = false;
	fluid->stats.bodyCount = bodyCount;
	fluid->stats.clippedShapeCount = clippedShapeCount;
	fluid->stats.time += b3GetMilliseconds( ticks );
}

void b3FluidSwapEvents( b3World* world )
{
	if ( world->fluid != NULL )
	{
		b3Array_Clear( world->fluid->endEvents[world->endEventArrayIndex] );
	}
}

// Wake query box: the bounds raised over their footprint to where the surface lies above them
static b3AABB b3GetFluidWakeBox( b3AABB bounds, b3Plane surface )
{
	int axis = b3GetFluidAxis( surface.normal );
	float nk = b3GetAxis( surface.normal, axis );
	if ( nk == 0.0f )
	{
		return bounds;
	}

	int axis1 = axis == 0 ? 1 : 0;
	int axis2 = axis == 2 ? 1 : 2;
	float n1 = b3GetAxis( surface.normal, axis1 );
	float n2 = b3GetAxis( surface.normal, axis2 );

	float level =
		nk > 0.0f ? b3GetAxis( bounds.upperBound, axis ) : b3GetAxis( bounds.lowerBound, axis );
	for ( int corner = 0; corner < 4; ++corner )
	{
		float p1 = ( corner & 1 ) ? b3GetAxis( bounds.upperBound, axis1 )
								  : b3GetAxis( bounds.lowerBound, axis1 );
		float p2 = ( corner & 2 ) ? b3GetAxis( bounds.upperBound, axis2 )
								  : b3GetAxis( bounds.lowerBound, axis2 );
		float cornerLevel = ( surface.offset - n1 * p1 - n2 * p2 ) / nk;
		level = nk > 0.0f ? b3MaxFloat( level, cornerLevel ) : b3MinFloat( level, cornerLevel );
	}

	b3AABB box = bounds;
	if ( nk > 0.0f )
	{
		b3SetAxis( &box.upperBound, axis, level );
	}
	else
	{
		b3SetAxis( &box.lowerBound, axis, level );
	}

	return box;
}

typedef struct b3FluidWakeContext
{
	b3World* world;
	int volumeIndex;

	// The volume before the change
	b3FluidVolume previous;

	b3FluidWakeRule rule;
} b3FluidWakeContext;

static bool b3ShouldWakeForFluid( b3FluidWakeContext* context, b3Body* body )
{
	b3World* world = context->world;
	b3FluidWorld* fluid = world->fluid;
	const b3FluidVolume* volume = fluid->volumes.data + context->volumeIndex;
	const b3FluidVolume* previous = &context->previous;

	b3BodySim* sim = b3GetBodySim( world, body );
	b3Pos center = sim->center;
	float radius = b3Length( sim->maxExtent );
	float separation = b3GetFluidSeparation( volume->surface, center );
	bool contains;

	if ( context->rule == b3_fluidWakeImmersed )
	{
		return b3IsFluidCandidate( volume, center, radius, &contains ) && separation < radius;
	}

	if ( b3IsFluidCandidate( volume, center, radius, &contains ) == false &&
		 b3IsFluidCandidate( previous, center, radius, &contains ) == false )
	{
		return false;
	}

	float previousSeparation = b3GetFluidSeparation( previous->surface, center );
	bool below = separation + radius <= 0.0f && previousSeparation + radius <= 0.0f;
	bool above = separation >= radius && previousSeparation >= radius;
	if ( below == false && above == false )
	{
		return true;
	}

	// The immersion is unchanged, but the surface height decides between overlapping volumes
	return b3HasOtherFluidCandidate( fluid, center, radius, context->volumeIndex );
}

static bool b3FluidWakeCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	b3FluidWakeContext* wakeContext = (b3FluidWakeContext*)context;
	b3World* world = wakeContext->world;

	int shapeId = (int)userData;
	b3Shape* shape = b3Array_Get( world->shapes, shapeId );
	b3Body* body = b3Array_Get( world->bodies, shape->bodyId );
	if ( body->setIndex < b3_firstSleepingSet )
	{
		return true;
	}

	if ( wakeContext->rule != b3_fluidWakeRegion &&
		 b3ShouldWakeForFluid( wakeContext, body ) == false )
	{
		return true;
	}

	b3WakeBody( world, body );
	return true;
}

// Wakes sleepers in the current and previous regions per the rule, like b3World_Explode
static void b3WakeFluidRegion( b3World* world, int volumeIndex, const b3FluidVolume* previous,
							   b3FluidWakeRule rule )
{
	b3FluidVolume* volume = b3Array_Get( world->fluid->volumes, volumeIndex );
	b3DynamicTree* tree = world->broadPhase.trees + b3_dynamicBody;

	b3FluidWakeContext context = { world, volumeIndex, *previous, rule };

	b3AABB box = b3GetFluidWakeBox( volume->aabb, volume->surface );
	b3DynamicTree_Query( tree, box, volume->maskBits, false, b3FluidWakeCallback, &context );

	b3AABB previousBox = b3GetFluidWakeBox( previous->aabb, previous->surface );
	if ( memcmp( &box, &previousBox, sizeof( b3AABB ) ) != 0 )
	{
		b3DynamicTree_Query( tree, previousBox, previous->maskBits, false, b3FluidWakeCallback,
							 &context );
	}
}

// Wakes a volume's recorded bodies in body id order, ending their touch first if asked
static void b3WakeFluidMembers( b3World* world, b3FluidVolumeId volumeId, bool end )
{
	b3FluidWorld* fluid = world->fluid;
	for ( int bodyId = 0; bodyId < fluid->bodies.count; ++bodyId )
	{
		b3FluidBody* entry = fluid->bodies.data + bodyId;
		if ( B3_ID_EQUALS( entry->volumeId, volumeId ) == false )
		{
			continue;
		}

		if ( end )
		{
			b3FluidEndTouchEvent event = { b3MakeBodyId( world, bodyId ), volumeId };
			b3Array_Push( fluid->endEvents[world->endEventArrayIndex], event );
			entry->volumeId = b3_nullFluidVolumeId;
			entry->fraction = 0.0f;
		}

		b3Body* body = b3Array_Get( world->bodies, bodyId );
		b3WakeBody( world, body );
	}
}

static void b3ValidateFluidVolumeDef( const b3FluidVolumeDef* def )
{
	B3_UNUSED( def );
	B3_ASSERT( b3IsValidAABB( def->bounds ) );
	B3_ASSERT( b3IsValidFloat( def->rotation ) );
	B3_ASSERT( b3IsValidPlane( def->surface ) );
	B3_ASSERT( b3IsValidFloat( def->density ) && def->density >= 0.0f );
	B3_ASSERT( b3IsValidFloat( def->linearDrag ) && def->linearDrag >= 0.0f );
	B3_ASSERT( b3IsValidFloat( def->angularDrag ) && def->angularDrag >= 0.0f );
	B3_ASSERT( b3IsValidFloat( def->quadraticDrag ) && def->quadraticDrag >= 0.0f );
	B3_ASSERT( b3IsValidVec3( def->flowVelocity ) );
}

b3FluidVolumeDef b3DefaultFluidVolumeDef( void )
{
	float lengthUnits = b3GetLengthUnitsPerMeter();
	float huge = B3_HUGE;

	b3FluidVolumeDef def = { 0 };
	def.bounds = (b3AABB){ { -huge, -huge, -huge }, { huge, huge, huge } };
	def.surface = (b3Plane){ { 0.0f, 1.0f, 0.0f }, 0.0f };
	def.density = 1000.0f / ( lengthUnits * lengthUnits * lengthUnits );
	def.linearDrag = 1.5f;
	def.angularDrag = 1.5f;
	def.quadraticDrag = 1.0f;
	def.maskBits = B3_DEFAULT_MASK_BITS;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3FluidVolumeId b3CreateFluidVolume( b3WorldId worldId, const b3FluidVolumeDef* def )
{
	B3_CHECK_DEF( def );
	b3ValidateFluidVolumeDef( def );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return b3_nullFluidVolumeId;
	}

	b3FluidWorld* fluid = b3GetOrCreateFluidWorld( world );

	int volumeIndex = b3AllocId( &fluid->volumeIdPool );
	if ( volumeIndex == fluid->volumes.count )
	{
		b3Array_Push( fluid->volumes, (b3FluidVolume){ 0 } );
	}

	b3FluidVolume* volume = b3Array_Get( fluid->volumes, volumeIndex );
	volume->bounds = def->bounds;
	volume->rotation = def->rotation;
	volume->surface = def->surface;
	volume->flowVelocity = def->flowVelocity;
	volume->density = def->density;
	volume->linearDrag = def->linearDrag;
	volume->angularDrag = def->angularDrag;
	volume->quadraticDrag = def->quadraticDrag;
	volume->maskBits = def->maskBits;
	volume->userData = def->userData;
	volume->id = volumeIndex;
	volume->axis = b3GetFluidAxis( def->surface.normal );
	volume->generation += 1;
	b3UpdateFluidVolumeAABB( volume );

	b3RebuildFluidLookup( fluid );

	b3FluidVolumeId id = b3MakeFluidVolumeId( world, volumeIndex );

	world->locked = true;
	b3WakeFluidRegion( world, volumeIndex, volume, b3_fluidWakeRegion );
	world->locked = false;

	B3_REC_CREATE( world, CreateFluidVolume, id, worldId, *def );

	return id;
}

void b3DestroyFluidVolume( b3FluidVolumeId volumeId )
{
	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, DestroyFluidVolume, volumeId );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	int volumeIndex = volume->id;

	world->locked = true;
	b3WakeFluidMembers( world, volumeId, true );
	b3WakeFluidRegion( world, volumeIndex, volume, b3_fluidWakeRegion );
	world->locked = false;

	volume->id = B3_NULL_INDEX;
	volume->userData = NULL;

	b3FreeId( &world->fluid->volumeIdPool, volumeIndex );
	b3RebuildFluidLookup( world->fluid );
}

bool b3FluidVolume_IsValid( b3FluidVolumeId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3_worlds + id.world0;
	if ( world->worldId != id.world0 || world->fluid == NULL )
	{
		return false;
	}

	b3FluidWorld* fluid = world->fluid;
	if ( id.index1 < 1 || fluid->volumes.count < id.index1 )
	{
		return false;
	}

	b3FluidVolume* volume = fluid->volumes.data + ( id.index1 - 1 );
	if ( volume->id == B3_NULL_INDEX )
	{
		return false;
	}

	return volume->generation == id.generation;
}

b3FluidVolumeDef b3FluidVolume_GetDef( b3FluidVolumeId volumeId )
{
	b3World* world = b3GetWorld( volumeId.world0 );
	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );

	b3FluidVolumeDef def = b3DefaultFluidVolumeDef();
	def.bounds = volume->bounds;
	def.rotation = volume->rotation;
	def.surface = volume->surface;
	def.density = volume->density;
	def.linearDrag = volume->linearDrag;
	def.angularDrag = volume->angularDrag;
	def.quadraticDrag = volume->quadraticDrag;
	def.flowVelocity = volume->flowVelocity;
	def.maskBits = volume->maskBits;
	def.userData = volume->userData;
	return def;
}

void b3FluidVolume_SetSurface( b3FluidVolumeId volumeId, b3Plane surface )
{
	B3_ASSERT( b3IsValidPlane( surface ) );

	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, FluidVolumeSetSurface, volumeId, surface );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	if ( memcmp( &volume->surface, &surface, sizeof( b3Plane ) ) == 0 )
	{
		return;
	}

	b3FluidVolume previous = *volume;
	volume->surface = surface;
	volume->axis = b3GetFluidAxis( surface.normal );

	if ( volume->axis == previous.axis )
	{
		world->locked = true;
		b3WakeFluidRegion( world, volume->id, &previous, b3_fluidWakeSurface );
		world->locked = false;
		return;
	}

	b3UpdateFluidVolumeAABB( volume );
	b3RebuildFluidLookup( world->fluid );

	// A rotated footprint turns about the new axis, so the region moves like a bounds change
	world->locked = true;
	if ( volume->rotation != 0.0f )
	{
		b3WakeFluidMembers( world, volumeId, false );
		b3WakeFluidRegion( world, volume->id, &previous, b3_fluidWakeRegion );
	}
	else
	{
		b3WakeFluidRegion( world, volume->id, &previous, b3_fluidWakeSurface );
	}
	world->locked = false;
}

void b3FluidVolume_SetBounds( b3FluidVolumeId volumeId, b3AABB bounds )
{
	B3_ASSERT( b3IsValidAABB( bounds ) );

	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, FluidVolumeSetBounds, volumeId, bounds );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	if ( memcmp( &volume->bounds, &bounds, sizeof( b3AABB ) ) == 0 )
	{
		return;
	}

	b3FluidVolume previous = *volume;
	volume->bounds = bounds;
	b3UpdateFluidVolumeAABB( volume );
	b3RebuildFluidLookup( world->fluid );

	world->locked = true;
	b3WakeFluidMembers( world, volumeId, false );
	b3WakeFluidRegion( world, volume->id, &previous, b3_fluidWakeRegion );
	world->locked = false;
}

void b3FluidVolume_SetRotation( b3FluidVolumeId volumeId, float rotation )
{
	B3_ASSERT( b3IsValidFloat( rotation ) );

	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, FluidVolumeSetRotation, volumeId, rotation );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	if ( volume->rotation == rotation )
	{
		return;
	}

	b3FluidVolume previous = *volume;
	volume->rotation = rotation;
	b3UpdateFluidVolumeAABB( volume );
	b3RebuildFluidLookup( world->fluid );

	world->locked = true;
	b3WakeFluidMembers( world, volumeId, false );
	b3WakeFluidRegion( world, volume->id, &previous, b3_fluidWakeRegion );
	world->locked = false;
}

void b3FluidVolume_SetFlowVelocity( b3FluidVolumeId volumeId, b3Vec3 flowVelocity )
{
	B3_ASSERT( b3IsValidVec3( flowVelocity ) );

	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, FluidVolumeSetFlowVelocity, volumeId, flowVelocity );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	if ( memcmp( &volume->flowVelocity, &flowVelocity, sizeof( b3Vec3 ) ) == 0 )
	{
		return;
	}

	volume->flowVelocity = flowVelocity;

	world->locked = true;
	b3WakeFluidRegion( world, volume->id, volume, b3_fluidWakeImmersed );
	world->locked = false;
}

void b3FluidVolume_SetDrag( b3FluidVolumeId volumeId, float linearDrag, float angularDrag,
							float quadraticDrag )
{
	B3_ASSERT( b3IsValidFloat( linearDrag ) && linearDrag >= 0.0f );
	B3_ASSERT( b3IsValidFloat( angularDrag ) && angularDrag >= 0.0f );
	B3_ASSERT( b3IsValidFloat( quadraticDrag ) && quadraticDrag >= 0.0f );

	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, FluidVolumeSetDrag, volumeId, linearDrag, angularDrag, quadraticDrag );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	if ( volume->linearDrag == linearDrag && volume->angularDrag == angularDrag &&
		 volume->quadraticDrag == quadraticDrag )
	{
		return;
	}

	volume->linearDrag = linearDrag;
	volume->angularDrag = angularDrag;
	volume->quadraticDrag = quadraticDrag;

	world->locked = true;
	b3WakeFluidRegion( world, volume->id, volume, b3_fluidWakeImmersed );
	world->locked = false;
}

void b3FluidVolume_SetDensity( b3FluidVolumeId volumeId, float density )
{
	B3_ASSERT( b3IsValidFloat( density ) && density >= 0.0f );

	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, FluidVolumeSetDensity, volumeId, density );

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	if ( volume->density == density )
	{
		return;
	}

	volume->density = density;

	world->locked = true;
	b3WakeFluidRegion( world, volume->id, volume, b3_fluidWakeImmersed );
	world->locked = false;
}

void b3FluidVolume_SetUserData( b3FluidVolumeId volumeId, void* userData )
{
	b3World* world = b3GetUnlockedWorld( volumeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	volume->userData = userData;
}

void* b3FluidVolume_GetUserData( b3FluidVolumeId volumeId )
{
	b3World* world = b3GetWorld( volumeId.world0 );
	b3FluidVolume* volume = b3GetFluidVolumeFullId( world, volumeId );
	return volume->userData;
}

b3BodyFluidDef b3DefaultBodyFluidDef( void )
{
	b3BodyFluidDef def = { 0 };
	def.displacedVolume = 0.0f;
	def.dragScale = 1.0f;
	def.enabled = true;
	return def;
}

static void b3ValidateBodyFluidDef( const b3BodyFluidDef* def )
{
	B3_UNUSED( def );
	B3_ASSERT( b3IsValidFloat( def->displacedVolume ) && def->displacedVolume >= 0.0f );
	B3_ASSERT( b3IsValidFloat( def->dragScale ) && def->dragScale >= 0.0f );
}

// Shared by the single and array setters
static void b3SetBodyFluidDefInternal( b3World* world, b3Body* body, const b3BodyFluidDef* def )
{
	b3FluidWorld* fluid = b3GetOrCreateFluidWorld( world );
	b3GrowFluidBodies( fluid, b3GetIdCapacity( &world->bodyIdPool ) );

	b3FluidBody* entry = fluid->bodies.data + body->id;
	b3BodyFluidDef oldDef = b3GetEntryFluidDef( entry );
	bool inFluid = B3_IS_NON_NULL( entry->volumeId );

	// A default def is stored as no def, so an entry out of fluid stays all zero
	entry->flags &= ~( b3_fluidHasDef | b3_fluidDisabled );
	entry->displacedVolume = 0.0f;
	entry->dragScale = 0.0f;

	if ( def->displacedVolume != 0.0f || def->dragScale != 1.0f || def->enabled == false )
	{
		entry->flags |= b3_fluidHasDef;
		entry->displacedVolume = def->displacedVolume;
		entry->dragScale = def->dragScale;

		if ( def->enabled == false )
		{
			entry->flags |= b3_fluidDisabled;
		}
	}

	if ( oldDef.enabled && def->enabled == false )
	{
		b3FluidRemoveBody( world, body->id );
	}

	bool wake = inFluid && ( oldDef.enabled != def->enabled ||
							 oldDef.displacedVolume != def->displacedVolume );

	// Turning fluids back on wakes a sleeper that may be in fluid
	if ( oldDef.enabled == false && def->enabled && body->type == b3_dynamicBody &&
		 body->setIndex >= b3_firstSleepingSet )
	{
		b3BodySim* sim = b3GetBodySim( world, body );
		float radius = b3Length( sim->maxExtent );
		float separation;
		wake = b3FindFluidVolumeIndex( fluid, sim->center, radius, &separation ) != B3_NULL_INDEX &&
			   separation < radius;
	}

	if ( wake )
	{
		b3WakeBodyWithLock( world, body );
	}
}

void b3Body_SetFluidDef( b3BodyId bodyId, const b3BodyFluidDef* def )
{
	b3ValidateBodyFluidDef( def );

	b3World* world = b3GetUnlockedWorld( bodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	B3_REC( world, BodySetFluidDef, bodyId, *def );

	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3SetBodyFluidDefInternal( world, body, def );
}

b3BodyFluidDef b3Body_GetFluidDef( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	return b3GetEntryFluidDef( b3GetFluidBody( world, body->id ) );
}

float b3Body_GetSubmergedFraction( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3FluidBody* entry = b3GetFluidBody( world, body->id );
	return entry != NULL ? entry->fraction : 0.0f;
}

b3FluidVolumeId b3Body_GetFluidVolume( b3BodyId bodyId )
{
	b3World* world = b3GetWorld( bodyId.world0 );
	b3Body* body = b3GetBodyFullId( world, bodyId );
	b3FluidBody* entry = b3GetFluidBody( world, body->id );
	return entry != NULL ? entry->volumeId : b3_nullFluidVolumeId;
}

// Tolerates a stale id like batch.c, skipping it instead of asserting
static b3Body* b3TryGetFluidBody( b3World* world, b3BodyId bodyId )
{
	if ( bodyId.world0 != world->worldId || b3Body_IsValid( bodyId ) == false )
	{
		return NULL;
	}

	return b3Array_Get( world->bodies, bodyId.index1 - 1 );
}

void b3World_SetBodyFluidDefs( b3WorldId worldId, const b3BodyId* bodyIds,
							   const b3BodyFluidDef* defs, int count )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	for ( int i = 0; i < count; ++i )
	{
		const b3BodyFluidDef* def = defs + i;
		b3ValidateBodyFluidDef( def );

		b3Body* body = b3TryGetFluidBody( world, bodyIds[i] );
		if ( body == NULL )
		{
			continue;
		}

		B3_REC( world, BodySetFluidDef, bodyIds[i], *def );

		b3SetBodyFluidDefInternal( world, body, def );
	}
}

b3FluidVolumeId b3World_FindFluidVolume( b3WorldId worldId, b3Pos point, float* depth )
{
	B3_ASSERT( b3IsValidPosition( point ) );

	if ( depth != NULL )
	{
		*depth = 0.0f;
	}

	b3World* world = b3GetWorldFromId( worldId );
	b3FluidWorld* fluid = world->fluid;
	if ( fluid == NULL )
	{
		return b3_nullFluidVolumeId;
	}

	float separation;
	int volumeIndex = b3FindFluidVolumeIndex( fluid, point, 0.0f, &separation );
	if ( volumeIndex == B3_NULL_INDEX || ( separation < 0.0f ) == false )
	{
		return b3_nullFluidVolumeId;
	}

	if ( depth != NULL )
	{
		*depth = -separation;
	}

	return b3MakeFluidVolumeId( world, volumeIndex );
}

b3FluidEvents b3World_GetFluidEvents( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL || world->fluid == NULL )
	{
		return (b3FluidEvents){ 0 };
	}

	b3FluidWorld* fluid = world->fluid;

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world->endEventArrayIndex;

	b3FluidEvents events = {
		.beginEvents = fluid->beginEvents.data,
		.endEvents = fluid->endEvents[endEventArrayIndex].data,
		.beginCount = fluid->beginEvents.count,
		.endCount = fluid->endEvents[endEventArrayIndex].count,
	};
	return events;
}

b3FluidStats b3World_GetFluidStats( b3WorldId worldId )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL || world->fluid == NULL )
	{
		return (b3FluidStats){ 0 };
	}

	return world->fluid->stats;
}
