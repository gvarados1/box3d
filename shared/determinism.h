// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/fluid.h"
#include "box3d/id.h"

#include "human.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define RAGDOLL_GROUP_SIZE 2
#define RAGDOLL_GRID_COUNT 2

typedef struct RagdollGroup
{
	Human humans[RAGDOLL_GROUP_SIZE];
} RagdollGroup;

typedef struct FallingRagdollData
{
	RagdollGroup groups[RAGDOLL_GRID_COUNT * RAGDOLL_GRID_COUNT];
	b3MeshData* gridMesh;
	b3MeshData* torusMesh;
	int columnCount;
	int columnIndex;
	int stepCount;
	int sleepStep;
	uint32_t hash;
} FallingRagdollData;

FallingRagdollData CreateFallingRagdolls( b3WorldId worldId );
bool UpdateFallingRagdolls( b3WorldId worldId, FallingRagdollData* data );
void DestroyFallingRagdolls( FallingRagdollData* data );

// Convex pile dropped on a wave height field. Spheres, capsules, boxes, and rocks with
// rolling resistance so the pile sleeps quickly.
#define WAVE_PILE_BODY_COUNT 100

typedef struct WavePileData
{
	b3BodyId bodies[WAVE_PILE_BODY_COUNT];
	b3HeightFieldData* heightField;
	int stepCount;
	int sleepStep;
	uint32_t hash;
} WavePileData;

WavePileData CreateWavePile( b3WorldId worldId );
bool UpdateWavePile( b3WorldId worldId, WavePileData* data );
void DestroyWavePile( WavePileData* data );

// Query driven spawning in an empty zero gravity world. Each step casts a ray, overlaps an
// AABB, and casts a sphere, then spawns a shape whose position, type, and size depend on the
// query results. Any query divergence cascades into the final state.
#define QUERY_SPAWN_COUNT 50
#define QUERY_SPAWN_CAST_RADIUS 0.5f

typedef struct QuerySpawnData
{
	b3BodyId bodies[QUERY_SPAWN_COUNT];
	int spawnCount;
	int queryHitCount;
	uint32_t queryHash;
	int stepCount;
	int sleepStep;
	uint32_t hash;

	// Last query cycle, recorded for visualization
	b3Pos rayOrigin;
	b3Vec3 rayTranslation;
	b3Pos rayPoint;
	b3Vec3 rayNormal;
	bool rayDidHit;
	b3AABB overlapBounds;
	int overlapCount;
	float castFraction;
	b3Pos lastSpawnPosition;
} QuerySpawnData;

QuerySpawnData CreateQuerySpawn( b3WorldId worldId );
bool UpdateQuerySpawn( b3WorldId worldId, QuerySpawnData* data );
void DestroyQuerySpawn( QuerySpawnData* data );

// Mixed shapes and specific gravities dropped into a walled pool. A higher volume with a current
// overlaps one end, so bodies switch volumes as they drift out of it or sink below it. Hashes the
// settled state and the ordered fluid event stream.
#define FLUID_POOL_GRID 5
#define FLUID_POOL_BODY_COUNT ( FLUID_POOL_GRID * FLUID_POOL_GRID )

typedef struct FluidPoolData
{
	b3BodyId bodies[FLUID_POOL_BODY_COUNT];
	b3FluidVolumeId poolId;
	b3FluidVolumeId channelId;
	int beginCount;
	int endCount;
	uint32_t eventHash;
	int stepCount;
	int sleepStep;
	uint32_t hash;
} FluidPoolData;

FluidPoolData CreateFluidPool( b3WorldId worldId );
bool UpdateFluidPool( b3WorldId worldId, FluidPoolData* data );
void DestroyFluidPool( FluidPoolData* data );

#ifdef __cplusplus
}
#endif
