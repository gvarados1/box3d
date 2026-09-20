// SPDX-License-Identifier: MIT

// MineMogul fork: Unity profiler hooks for the plugin build.

#pragma once

#include <stddef.h>

// Unity profiler hooks for the Unity plugin build (BOX3D_UNITY_PROFILER). The scheduler registers its
// worker threads with Unity's profiler and brackets every task with a named sample, so the workers get
// rows in the Unity timeline next to the game's own threads. Without the option, or when the host never
// calls the plugin entry point (samples app, tests), every hook is a no-op.

#if defined( BOX3D_UNITY_PROFILER )

typedef const struct UnityProfilerMarkerDesc b3ProfilerMarker;

void b3UnityProfilerRegisterThread( const char* name );
void b3UnityProfilerUnregisterThread( void );

// The marker for a task name, created on first use. Names must be string literals: the cache is
// keyed by pointer. NULL when the profiler is not available.
b3ProfilerMarker* b3UnityProfilerMarker( const char* name );
void b3UnityProfilerBegin( b3ProfilerMarker* marker );
void b3UnityProfilerEnd( b3ProfilerMarker* marker );

#else

typedef const void b3ProfilerMarker;

static inline void b3UnityProfilerRegisterThread( const char* name )
{
	(void)name;
}

static inline void b3UnityProfilerUnregisterThread( void )
{
}

static inline b3ProfilerMarker* b3UnityProfilerMarker( const char* name )
{
	(void)name;
	return NULL;
}

static inline void b3UnityProfilerBegin( b3ProfilerMarker* marker )
{
	(void)marker;
}

static inline void b3UnityProfilerEnd( b3ProfilerMarker* marker )
{
	(void)marker;
}

#endif
