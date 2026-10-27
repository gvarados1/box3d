// SPDX-License-Identifier: MIT

// MineMogul fork: fluid volumes, see docs/fluid.md. Buoyancy, drag and currents for dynamic bodies,
// computed inside b3World_Step over awake bodies only. Worlds without fluid volumes are unaffected.

#pragma once

#include "base.h"
#include "collision.h"
#include "id.h"
#include "math_functions.h"
#include "types.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup fluid Fluid
 * @brief Fluid volumes with buoyancy, drag, currents and enter/exit events (MineMogul fork).
 * @{
 */

/// Fluid volume id. Treat as an opaque handle.
typedef struct b3FluidVolumeId
{
	int32_t index1;
	uint16_t world0;
	uint16_t generation;
} b3FluidVolumeId;

static const b3FluidVolumeId b3_nullFluidVolumeId = B3_NULL_ID;

/// Describes a fluid volume. The fluid fills the bounds below the surface plane.
typedef struct b3FluidVolumeDef
{
	/// Region the fluid can occupy, in world space.
	b3AABB bounds;

	/// Fluid surface. The normal points out of the fluid and must be unit length.
	b3Plane surface;

	/// Fluid density. Default is water, 1000 / lengthUnits^3.
	float density;

	/// Linear damping rate (1/s) toward the flow when fully submerged, scaled by the fraction.
	float linearDrag;

	/// Angular damping rate (1/s) at full submersion. Scaled by the submerged fraction.
	float angularDrag;

	/// Drag coefficient for 0.5 * density * Cd * A * |v - flow|^2, where A = submergedVolume^(2/3).
	float quadraticDrag;

	/// Velocity of the current.
	b3Vec3 flowVelocity;

	/// Only shapes whose filter category bits match these bits are buoyant.
	uint64_t maskBits;

	/// Use this to store application specific volume data.
	void* userData;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3FluidVolumeDef;

/// How a body responds to fluid. Bodies without a def use b3DefaultBodyFluidDef().
typedef struct b3BodyFluidDef
{
	/// Volume of fluid displaced at full submersion. Zero uses the volume of the body's buoyant
	/// shapes (non-sensor spheres, capsules and hulls). Mass never enters buoyancy.
	float displacedVolume;

	/// Multiplies the volume's linear, angular and quadratic drag on this body.
	float dragScale;

	/// False makes fluids ignore this body.
	bool enabled;
} b3BodyFluidDef;

/// A body started touching a fluid volume.
typedef struct b3FluidBeginTouchEvent
{
	b3BodyId bodyId;
	b3FluidVolumeId volumeId;

	/// Submerged centroid projected onto the surface.
	b3Pos point;

	/// Center of mass velocity before fluid drag.
	b3Vec3 velocity;
} b3FluidBeginTouchEvent;

/// A body stopped touching a fluid volume. The body may have been destroyed: check b3Body_IsValid.
/// For an exit position and velocity, use the same step's body move event.
typedef struct b3FluidEndTouchEvent
{
	b3BodyId bodyId;
	b3FluidVolumeId volumeId;
} b3FluidEndTouchEvent;

/// Fluid events for the last time step. Events are keyed by (body, volume) and the two arrays have
/// no order relative to each other: when tracking one volume per body, handle all end events before
/// all begin events. b3Body_GetFluidVolume is authoritative.
typedef struct b3FluidEvents
{
	b3FluidBeginTouchEvent* beginEvents;
	b3FluidEndTouchEvent* endEvents;
	int beginCount;
	int endCount;
} b3FluidEvents;

/// Fluid stage statistics for the last time step.
typedef struct b3FluidStats
{
	/// Fluid stage time in milliseconds.
	float time;
	int volumeCount;

	/// Bodies in fluid this step.
	int bodyCount;

	/// Shapes that crossed a surface and needed clipping this step.
	int clippedShapeCount;
} b3FluidStats;

/// Default fluid volume definition: water below y = 0, unbounded, no current.
B3_API b3FluidVolumeDef b3DefaultFluidVolumeDef( void );

/// Create a fluid volume. Wakes the dynamic bodies it may affect.
B3_API b3FluidVolumeId b3CreateFluidVolume( b3WorldId worldId, const b3FluidVolumeDef* def );

/// Destroy a fluid volume. Pushes end events for the bodies in it and wakes them.
B3_API void b3DestroyFluidVolume( b3FluidVolumeId volumeId );

/// Fluid volume id validation.
B3_API bool b3FluidVolume_IsValid( b3FluidVolumeId volumeId );

/// Get the definition the volume currently uses.
B3_API b3FluidVolumeDef b3FluidVolume_GetDef( b3FluidVolumeId volumeId );

/// Set the surface plane. Wakes bodies whose immersion may change.
B3_API void b3FluidVolume_SetSurface( b3FluidVolumeId volumeId, b3Plane surface );

/// Set the bounds. Wakes bodies in the old and new region.
B3_API void b3FluidVolume_SetBounds( b3FluidVolumeId volumeId, b3AABB bounds );

/// Set the current. Wakes bodies in the fluid.
B3_API void b3FluidVolume_SetFlowVelocity( b3FluidVolumeId volumeId, b3Vec3 flowVelocity );

/// Set the drag. Wakes bodies in the fluid.
B3_API void b3FluidVolume_SetDrag( b3FluidVolumeId volumeId, float linearDrag, float angularDrag,
								   float quadraticDrag );

/// Set the density. Wakes bodies in the fluid.
B3_API void b3FluidVolume_SetDensity( b3FluidVolumeId volumeId, float density );

/// Set the user data pointer.
B3_API void b3FluidVolume_SetUserData( b3FluidVolumeId volumeId, void* userData );

/// Get the user data pointer.
B3_API void* b3FluidVolume_GetUserData( b3FluidVolumeId volumeId );

/// Default body fluid definition: enabled, buoyant shape volume, drag scale 1.
B3_API b3BodyFluidDef b3DefaultBodyFluidDef( void );

/// Set how a body responds to fluid. Disabling it ends its touch at once. Wakes the body when it is
/// in fluid and its buoyancy changed, or when enabling it may put a sleeping body in fluid.
B3_API void b3Body_SetFluidDef( b3BodyId bodyId, const b3BodyFluidDef* def );

/// Get how a body responds to fluid.
B3_API b3BodyFluidDef b3Body_GetFluidDef( b3BodyId bodyId );

/// Submerged fraction from the last step, in [0, 1].
B3_API float b3Body_GetSubmergedFraction( b3BodyId bodyId );

/// The fluid volume the body is in, or a null id.
B3_API b3FluidVolumeId b3Body_GetFluidVolume( b3BodyId bodyId );

/// Array form of b3Body_SetFluidDef. Stale ids are skipped.
B3_API void b3World_SetBodyFluidDefs( b3WorldId worldId, const b3BodyId* bodyIds,
									  const b3BodyFluidDef* defs, int count );

/// The fluid volume containing a point (by the same rule bodies use) and the point's depth below
/// the surface. Returns a null id when the point is not in fluid.
B3_API b3FluidVolumeId b3World_FindFluidVolume( b3WorldId worldId, b3Pos point, float* depth );

/// Get the fluid events for the last time step. The event data is transient.
B3_API b3FluidEvents b3World_GetFluidEvents( b3WorldId worldId );

/// Get fluid stage statistics for the last time step.
B3_API b3FluidStats b3World_GetFluidStats( b3WorldId worldId );

/** @} */
