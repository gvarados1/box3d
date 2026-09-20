// SPDX-License-Identifier: MIT

// MineMogul fork: Unity profiler hooks for the plugin build.

#include "unity_profiler.h"

#include "core.h"
#include "platform.h"

#include "IUnityInterface.h"
#include "IUnityProfiler.h"

#include <stdio.h>
#include <string.h>

// Set by UnityPluginLoad when the host is Unity with a profiler-capable player (editor or development
// build); NULL everywhere else, which reduces every hook to a null test.
static IUnityProfilerV2* s_profiler;

// One marker per distinct task name, matched by content: the solver formats its names into a stack
// buffer, so pointers are not identities. Entries are appended under the mutex and published through
// the count, so readers need no lock.
#define B3_MAX_PROFILER_MARKERS 64
#define B3_PROFILER_NAME_CAPACITY 32

typedef struct b3ProfilerMarkerEntry
{
	char name[B3_PROFILER_NAME_CAPACITY];
	const UnityProfilerMarkerDesc* desc;
} b3ProfilerMarkerEntry;

static b3ProfilerMarkerEntry s_markers[B3_MAX_PROFILER_MARKERS];
static b3AtomicInt s_markerCount;
static b3Mutex* s_markerMutex;

static const UnityProfilerMarkerDesc* b3FindMarker( const char* name, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( strncmp( s_markers[i].name, name, B3_PROFILER_NAME_CAPACITY ) == 0 )
		{
			return s_markers[i].desc;
		}
	}
	return NULL;
}

// Unity calls this when it loads the plugin dll.
UNITY_INTERFACE_EXPORT void UNITY_INTERFACE_API UnityPluginLoad( IUnityInterfaces* unityInterfaces )
{
	IUnityProfilerV2* profiler = UNITY_GET_INTERFACE( unityInterfaces, IUnityProfilerV2 );
	if ( profiler == NULL || profiler->IsAvailable() == 0 )
	{
		return;
	}

	s_markerMutex = b3CreateMutex();
	s_profiler = profiler;
}

UNITY_INTERFACE_EXPORT void UNITY_INTERFACE_API UnityPluginUnload( void )
{
	s_profiler = NULL;
	if ( s_markerMutex != NULL )
	{
		b3DestroyMutex( s_markerMutex );
		s_markerMutex = NULL;
	}
	b3AtomicStoreInt( &s_markerCount, 0 );
}

void b3UnityProfilerRegisterThread( const char* name )
{
	IUnityProfilerV2* profiler = s_profiler;
	if ( profiler == NULL )
	{
		return;
	}

	UnityProfilerThreadId threadId;
	profiler->RegisterThread( &threadId, "Box3D", name );
}

void b3UnityProfilerUnregisterThread( void )
{
	IUnityProfilerV2* profiler = s_profiler;
	if ( profiler == NULL )
	{
		return;
	}

	// 0 = the calling thread
	profiler->UnregisterThread( 0 );
}

b3ProfilerMarker* b3UnityProfilerMarker( const char* name )
{
	IUnityProfilerV2* profiler = s_profiler;
	if ( profiler == NULL || name == NULL )
	{
		return NULL;
	}

	const UnityProfilerMarkerDesc* desc = b3FindMarker( name, b3AtomicLoadInt( &s_markerCount ) );
	if ( desc != NULL )
	{
		return desc;
	}

	b3LockMutex( s_markerMutex );

	// Another thread may have added it while this one waited for the lock.
	int count = b3AtomicLoadInt( &s_markerCount );
	desc = b3FindMarker( name, count );
	if ( desc == NULL && count < B3_MAX_PROFILER_MARKERS )
	{
		b3ProfilerMarkerEntry* entry = s_markers + count;
		snprintf( entry->name, sizeof( entry->name ), "%s", name );
		if ( profiler->CreateMarker( &desc, entry->name, kUnityProfilerCategoryPhysics, kUnityProfilerMarkerFlagDefault, 0 ) == 0 )
		{
			entry->desc = desc;
			b3AtomicStoreInt( &s_markerCount, count + 1 );
		}
		else
		{
			desc = NULL;
		}
	}

	b3UnlockMutex( s_markerMutex );
	return desc;
}

void b3UnityProfilerBegin( b3ProfilerMarker* marker )
{
	IUnityProfilerV2* profiler = s_profiler;
	if ( marker != NULL && profiler != NULL )
	{
		profiler->EmitEvent( marker, kUnityProfilerMarkerEventTypeBegin, 0, NULL );
	}
}

void b3UnityProfilerEnd( b3ProfilerMarker* marker )
{
	IUnityProfilerV2* profiler = s_profiler;
	if ( marker != NULL && profiler != NULL )
	{
		profiler->EmitEvent( marker, kUnityProfilerMarkerEventTypeEnd, 0, NULL );
	}
}
