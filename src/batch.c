// SPDX-License-Identifier: MIT

// MineMogul fork: array forms of the per-body write functions.
//
// The game drives thousands of conveyor items per step from C#, where every native call pays a
// managed-to-native transition on top of the body lookup. One call per array turns that into a
// plain loop. Each entry keeps the semantics of its single-body counterpart, including the
// recording op, so a recorded session replays through the ordinary per-body path.

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "recording.h"
#include "shape.h"
#include "solver_set.h"

#include "box3d/box3d.h"

// Tolerates a stale id (a body the caller destroyed after queuing it) instead of asserting like
// b3GetBodyFullId. Ids from another world are skipped the same way.
static b3Body* b3TryGetBody( b3World* world, b3BodyId bodyId )
{
	if ( bodyId.world0 != world->worldId || b3Body_IsValid( bodyId ) == false )
	{
		return NULL;
	}

	return b3Array_Get( world->bodies, bodyId.index1 - 1 );
}

void b3World_SetLinearVelocities( b3WorldId worldId, const b3BodyId* bodyIds, const b3Vec3* velocities, int count )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 linearVelocity = velocities[i];
		B3_ASSERT( b3IsValidVec3( linearVelocity ) );

		b3Body* body = b3TryGetBody( world, bodyIds[i] );
		if ( body == NULL || body->type == b3_staticBody )
		{
			continue;
		}

		B3_REC( world, BodySetLinearVelocity, bodyIds[i], linearVelocity );

		if ( b3LengthSquared( linearVelocity ) > 0.0f )
		{
			b3WakeBodyWithLock( world, body );
		}

		b3BodyState* state = b3GetBodyState( world, body );
		if ( state != NULL )
		{
			state->linearVelocity = linearVelocity;
		}
	}
}

void b3World_SetAngularVelocities( b3WorldId worldId, const b3BodyId* bodyIds, const b3Vec3* velocities, int count )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 angularVelocity = velocities[i];
		B3_ASSERT( b3IsValidVec3( angularVelocity ) );

		b3Body* body = b3TryGetBody( world, bodyIds[i] );
		if ( body == NULL || body->type == b3_staticBody )
		{
			continue;
		}

		B3_REC( world, BodySetAngularVelocity, bodyIds[i], angularVelocity );

		// Apply locks to avoid waking
		b3Vec3 w;
		w.x = ( body->flags & b3_lockAngularX ) ? 0.0f : angularVelocity.x;
		w.y = ( body->flags & b3_lockAngularY ) ? 0.0f : angularVelocity.y;
		w.z = ( body->flags & b3_lockAngularZ ) ? 0.0f : angularVelocity.z;

		if ( b3LengthSquared( w ) != 0.0f )
		{
			b3WakeBodyWithLock( world, body );
		}

		b3BodyState* state = b3GetBodyState( world, body );
		if ( state != NULL )
		{
			state->angularVelocity = w;
		}
	}
}

void b3World_ApplyForcesToCenter( b3WorldId worldId, const b3BodyId* bodyIds, const b3Vec3* forces, int count, bool wake )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 force = forces[i];
		B3_ASSERT( b3IsValidVec3( force ) );

		b3Body* body = b3TryGetBody( world, bodyIds[i] );
		if ( body == NULL )
		{
			continue;
		}

		B3_REC( world, BodyApplyForceToCenter, bodyIds[i], force, wake );

		if ( wake && body->setIndex >= b3_firstSleepingSet )
		{
			b3WakeBodyWithLock( world, body );
		}

		if ( body->setIndex == b3_awakeSet )
		{
			b3BodySim* bodySim = b3GetBodySim( world, body );
			bodySim->force = b3Add( bodySim->force, force );
		}
	}
}

void b3World_WakeBodies( b3WorldId worldId, const b3BodyId* bodyIds, int count )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	for ( int i = 0; i < count; ++i )
	{
		b3Body* body = b3TryGetBody( world, bodyIds[i] );
		if ( body == NULL )
		{
			continue;
		}

		B3_REC( world, BodySetAwake, bodyIds[i], true );

		if ( body->setIndex >= b3_firstSleepingSet )
		{
			b3WakeBodyWithLock( world, body );
		}
	}
}

// Sensor tree mask (box3d.h). Not recorded: a replay queries every tree, which yields the same events
// whenever the skipped trees hold no sensor visitors, the only case the mask exists for.
void b3World_SetSensorTreeMask( b3WorldId worldId, uint32_t mask )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}

	world->sensorTreeMask = mask & 0x7u;
}

uint32_t b3World_GetSensorTreeMask( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return world->sensorTreeMask;
}

// Contact census (box3d.h). Walks the same awake contacts the collide stage gathers each step: touching
// contacts from the constraint graph colors, then the non-touching ones from the awake set. Not recorded.
static void b3WriteAwakeContact( b3World* world, int contactId, b3AwakeContact* out )
{
	b3Contact* contact = b3Array_Get( world->contacts, contactId );
	b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	out->shapeIdA = (b3ShapeId){ shapeA->id + 1, world->worldId, shapeA->generation };
	out->shapeIdB = (b3ShapeId){ shapeB->id + 1, world->worldId, shapeB->generation };
	out->manifoldCount = contact->manifoldCount;
	out->touching = ( contact->flags & b3_contactTouchingFlag ) != 0 ? 1 : 0;
}

int b3World_GetAwakeContacts( b3WorldId worldId, b3AwakeContact* contacts, int capacity )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return 0;
	}

	int count = 0;
	for ( int i = 0; i < B3_GRAPH_COLOR_COUNT; ++i )
	{
		b3GraphColor* color = world->constraintGraph.colors + i;
		for ( int j = 0; j < color->convexContacts.count && count < capacity; ++j )
		{
			b3WriteAwakeContact( world, color->convexContacts.data[j], contacts + count );
			count += 1;
		}

		for ( int j = 0; j < color->contacts.count && count < capacity; ++j )
		{
			b3WriteAwakeContact( world, color->contacts.data[j].contactId, contacts + count );
			count += 1;
		}
	}

	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
	for ( int j = 0; j < awakeSet->contactIndices.count && count < capacity; ++j )
	{
		b3WriteAwakeContact( world, awakeSet->contactIndices.data[j], contacts + count );
		count += 1;
	}

	return count;
}
