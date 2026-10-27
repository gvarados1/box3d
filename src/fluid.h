// SPDX-License-Identifier: MIT

// MineMogul fork: fluid stage internals. See docs/fluid.md.

#pragma once

#include "bitset.h"
#include "container.h"
#include "id_pool.h"

#include "box3d/fluid.h"

typedef struct b3StepContext b3StepContext;
typedef struct b3World b3World;

enum b3FluidBodyFlags
{
	// The entry holds a user def. Without it the body uses b3DefaultBodyFluidDef.
	b3_fluidHasDef = 0x01,

	// The user def disables fluids for this body.
	b3_fluidDisabled = 0x02,

	// The pass raised the body damping this step. PostSolve restores it.
	b3_fluidDamped = 0x04,
};

typedef struct b3FluidVolume
{
	b3AABB bounds;
	b3Plane surface;
	b3Vec3 flowVelocity;
	float density;
	float linearDrag;
	float angularDrag;
	float quadraticDrag;
	uint64_t maskBits;
	void* userData;

	// B3_NULL_INDEX when the slot is free
	int id;

	// The bounds axis most aligned with the surface normal
	int axis;

	uint16_t generation;
} b3FluidVolume;

// Per-body fluid state indexed by body id. All zero means the default def and not in fluid.
typedef struct b3FluidBody
{
	// Begin event payload, valid between PreSolve and PostSolve
	b3Pos point;
	b3Vec3 velocity;

	// The volume the body is in, null when not in fluid
	b3FluidVolumeId volumeId;

	// The volume found this step, valid between PreSolve and PostSolve
	b3FluidVolumeId stepVolumeId;

	// Submerged fraction from the last step that evaluated this body
	float fraction;

	// User def, valid with b3_fluidHasDef
	float displacedVolume;
	float dragScale;

	// Damping saved while the pass raised it
	float linearDamping;
	float angularDamping;

	// b3FluidBodyFlags
	uint32_t flags;
} b3FluidBody;

typedef struct b3FluidTaskContext
{
	// Bodies in fluid this step or whose volume changed, keyed by body id
	b3BitSet changedBits;

	int bodyCount;
	int clippedShapeCount;

	// Prevent false sharing
	char cacheLine[64];
} b3FluidTaskContext;

b3DeclareArray( b3FluidVolume );
b3DeclareArray( b3FluidBody );
b3DeclareArray( b3FluidTaskContext );
b3DeclareArray( b3FluidBeginTouchEvent );
b3DeclareArray( b3FluidEndTouchEvent );

typedef struct b3FluidWorld
{
	b3IdPool volumeIdPool;

	// Sparse array indexed by volume id
	b3Array( b3FluidVolume ) volumes;

	// Volume lookup, see docs/fluid.md "Volume lookup"
	b3Array( int ) unbinnedVolumes;
	b3Array( int ) cellStarts;
	b3Array( int ) cellVolumes;
	float gridLowerX;
	float gridLowerZ;
	float gridUpperX;
	float gridUpperZ;
	float inverseCellSize;
	int cellCountX;
	int cellCountZ;

	// Side table indexed by body id
	b3Array( b3FluidBody ) bodies;

	// Sized to the world worker count by PreSolve
	b3Array( b3FluidTaskContext ) taskContexts;

	b3Array( b3FluidBeginTouchEvent ) beginEvents;

	// Double buffered on world->endEventArrayIndex like sensor end events
	b3Array( b3FluidEndTouchEvent ) endEvents[2];

	b3FluidStats stats;

	// PreSolve ran this step and PostSolve has work to finish
	bool stepActive;
} b3FluidWorld;

// Created by the first fluid mutator, freed with the world or by a restore without fluid
b3FluidWorld* b3GetOrCreateFluidWorld( b3World* world );
void b3DestroyFluidWorld( b3World* world );
int b3GetFluidByteCount( const b3World* world );
void b3RebuildFluidLookup( b3FluidWorld* fluid );

// Step hooks, see docs/fluid.md "Step placement and data flow"
void b3FluidPreSolve( b3World* world, b3StepContext* context );
void b3FluidPostSolve( b3World* world );
void b3FluidSwapEvents( b3World* world );

// Ends the body's touch with an end event and clears its fluid state, keeping its def.
void b3FluidRemoveBody( b3World* world, int bodyId );

// Like b3FluidRemoveBody but also resets the def. Call before the body id is freed.
void b3FluidDestroyBody( b3World* world, int bodyId );

// Ends the touch when the new type is not dynamic.
void b3FluidSetBodyType( b3World* world, int bodyId, b3BodyType type );
