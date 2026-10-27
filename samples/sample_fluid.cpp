// SPDX-License-Identifier: MIT

// MineMogul fork: fluid volume samples. See docs/fluid.md.

#include "gfx/draw.h"
#include "imgui.h"
#include "sample.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <string.h>
#include <vector>

enum FluidShapeType
{
	e_fluidSphere,
	e_fluidBox,
	e_fluidCapsule,
	e_fluidRock,
};

// Sphere and rock: size.x is the radius. Box: half extents. Capsule: half length, radius.
struct FluidItemKind
{
	const char* name;
	FluidShapeType shapeType;
	b3Vec3 size;
	float specificGravity;
};

struct FluidItem
{
	b3BodyId bodyId;
	b3Vec3 localCenter;
	float specificGravity;
	int kind;
	int age;
};

static float GetWaterDensity()
{
	return b3DefaultFluidVolumeDef().density;
}
// Real mass from the specific gravity. Floaters are gold and sinkers red when color coded.
static b3BodyId CreateFluidBody( b3WorldId worldId, const FluidItemKind* kind, b3Pos position,
								 b3Quat rotation, bool colorCode )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = position;
	bodyDef.rotation = rotation;
	bodyDef.name = kind->name;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = kind->specificGravity * GetWaterDensity();
	if ( colorCode )
	{
		shapeDef.baseMaterial.customColor =
			kind->specificGravity < 1.0f ? b3_colorGold : b3_colorCrimson;
	}

	switch ( kind->shapeType )
	{
		case e_fluidSphere:
		{
			b3Sphere sphere = { b3Vec3_zero, kind->size.x };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
		break;

		case e_fluidBox:
		{
			b3BoxHull box = b3MakeBoxHull( kind->size.x, kind->size.y, kind->size.z );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
		break;

		case e_fluidCapsule:
		{
			b3Capsule capsule = {
				{ -kind->size.x, 0.0f, 0.0f }, { kind->size.x, 0.0f, 0.0f }, kind->size.y };
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
		break;

		case e_fluidRock:
		{
			b3HullData* hull = b3CreateRock( kind->size.x );
			b3CreateHullShape( bodyId, &shapeDef, hull );
			b3DestroyHull( hull );
		}
		break;
	}

	return bodyId;
}

// MineMogul's rule: the displaced volume that gives the body its specific gravity in water
static b3BodyFluidDef MakeSpecificGravityDef( b3BodyId bodyId, float specificGravity )
{
	b3BodyFluidDef fluidDef = b3DefaultBodyFluidDef();
	fluidDef.displacedVolume = b3Body_GetMass( bodyId ) / ( specificGravity * GetWaterDensity() );
	return fluidDef;
}

static FluidItem CreateFluidItem( b3WorldId worldId, const FluidItemKind* kinds, int kindIndex,
								  b3Pos position, b3Quat rotation )
{
	const FluidItemKind* kind = kinds + kindIndex;
	b3BodyId bodyId = CreateFluidBody( worldId, kind, position, rotation, true );

	b3BodyFluidDef fluidDef = MakeSpecificGravityDef( bodyId, kind->specificGravity );
	b3Body_SetFluidDef( bodyId, &fluidDef );

	FluidItem item = {};
	item.bodyId = bodyId;
	item.localCenter = b3Body_GetMassData( bodyId ).center;
	item.specificGravity = kind->specificGravity;
	item.kind = kindIndex;
	item.age = 0;
	return item;
}

// Walls around the interior on the ground box floor. An open end leaves out the +x wall.
static void CreateBasin( b3WorldId worldId, b3AABB interior, bool openEnd )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.name = "basin";
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();

	float thickness = 0.25f;
	b3Vec3 lower = interior.lowerBound;
	b3Vec3 upper = interior.upperBound;
	b3Vec3 extents = b3MulSV( 0.5f, b3Sub( upper, lower ) );
	b3Vec3 center = b3Add( lower, extents );

	b3BoxHull box = b3MakeOffsetBoxHull( thickness, extents.y, extents.z + 2.0f * thickness,
										 { lower.x - thickness, center.y, center.z } );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	if ( openEnd == false )
	{
		box = b3MakeOffsetBoxHull( thickness, extents.y, extents.z + 2.0f * thickness,
								   { upper.x + thickness, center.y, center.z } );
		b3CreateHullShape( bodyId, &shapeDef, &box.base );
	}

	box = b3MakeOffsetBoxHull( extents.x, extents.y, thickness,
							   { center.x, center.y, lower.z - thickness } );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	box = b3MakeOffsetBoxHull( extents.x, extents.y, thickness,
							   { center.x, center.y, upper.z + thickness } );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
}

static b3FluidVolumeId CreatePoolVolume( b3WorldId worldId, b3AABB bounds, float surfaceHeight,
										 b3Vec3 flowVelocity )
{
	b3FluidVolumeDef volumeDef = b3DefaultFluidVolumeDef();
	volumeDef.bounds = bounds;
	volumeDef.surface = { b3Vec3_axisY, surfaceHeight };
	volumeDef.flowVelocity = flowVelocity;
	return b3CreateFluidVolume( worldId, &volumeDef );
}

// Translucent water from the bounds bottom up to the surface, with the surface outline
static void DrawWater( b3AABB bounds, float surfaceHeight )
{
	b3Vec3 lower = bounds.lowerBound;
	b3Vec3 upper = bounds.upperBound;
	float top = b3MinFloat( surfaceHeight, upper.y );
	if ( top <= lower.y )
	{
		return;
	}

	b3Vec3 center = { 0.5f * ( lower.x + upper.x ), 0.5f * ( lower.y + top ),
					  0.5f * ( lower.z + upper.z ) };
	b3WorldTransform transform = { b3ToPos( center ), b3Quat_identity };
	b3Vec3 scale = { upper.x - lower.x, top - lower.y, upper.z - lower.z };
	DrawCubeEx( transform, scale, MakeColorAlpha( b3_colorDodgerBlue, 0.25f ), 0.0f, 0.1f,
				TRANSPARENT_SHADOW_NONE );

	Vec4 color = MakeColor( b3_colorLightSkyBlue );
	b3Pos p1 = { lower.x, top, lower.z };
	b3Pos p2 = { upper.x, top, lower.z };
	b3Pos p3 = { upper.x, top, upper.z };
	b3Pos p4 = { lower.x, top, upper.z };
	DrawLine( p1, p2, color );
	DrawLine( p2, p3, color );
	DrawLine( p3, p4, color );
	DrawLine( p4, p1, color );
}

static void DrawRing( b3Pos center, float radius, Vec4 color )
{
	constexpr int segmentCount = 16;
	b3Pos p1 = center + b3Vec3{ radius, 0.0f, 0.0f };
	for ( int i = 1; i <= segmentCount; ++i )
	{
		b3CosSin cs = b3ComputeCosSin( 2.0f * B3_PI * i / segmentCount );
		b3Pos p2 = center + b3Vec3{ radius * cs.cosine, 0.0f, radius * cs.sine };
		DrawLine( p1, p2, color );
		p1 = p2;
	}
}

static const FluidItemKind s_poolKinds[] = {
	{ "cork ball", e_fluidSphere, { 0.25f, 0.0f, 0.0f }, 0.25f },
	{ "wood ball", e_fluidSphere, { 0.3f, 0.0f, 0.0f }, 0.6f },
	{ "steel ball", e_fluidSphere, { 0.2f, 0.0f, 0.0f }, 7.8f },
	{ "pine box", e_fluidBox, { 0.3f, 0.3f, 0.3f }, 0.5f },
	{ "ice block", e_fluidBox, { 0.35f, 0.25f, 0.3f }, 0.92f },
	{ "stone block", e_fluidBox, { 0.25f, 0.25f, 0.25f }, 2.5f },
	{ "plank", e_fluidBox, { 0.8f, 0.05f, 0.2f }, 0.5f },
	{ "log", e_fluidCapsule, { 0.5f, 0.2f, 0.0f }, 0.7f },
	{ "pipe", e_fluidCapsule, { 0.4f, 0.12f, 0.0f }, 1.6f },
	{ "pumice", e_fluidRock, { 0.35f, 0.0f, 0.0f }, 0.7f },
	{ "rock", e_fluidRock, { 0.35f, 0.0f, 0.0f }, 2.6f },
};

// A pool with floaters and sinkers. The sliders drive every fluid volume setter.
class Buoyancy : public Sample
{
public:
	enum
	{
		e_maxItems = 64,
		e_maxSplashes = 32,
		e_splashSteps = 40,
	};

	struct Splash
	{
		b3Pos point;
		float speed;
		int age;
	};

	explicit Buoyancy( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -30.0f, 30.0f, 20.0f, { 0.0f, 1.0f, 0.0f } );
		}

		// Paused frames skip the world step, so a dt = 0 step never clears the events on screen
		m_stepWhilePaused = false;

		AddGroundBox( 20.0f );

		m_bounds = { { -5.0f, 0.0f, -4.0f }, { 5.0f, 3.0f, 4.0f } };
		CreateBasin( m_worldId, m_bounds, false );

		b3FluidVolumeDef defaultDef = b3DefaultFluidVolumeDef();
		m_density = defaultDef.density;
		m_linearDrag = defaultDef.linearDrag;
		m_angularDrag = defaultDef.angularDrag;
		m_quadraticDrag = defaultDef.quadraticDrag;
		m_surfaceHeight = 2.0f;
		m_flowVelocity = b3Vec3_zero;
		m_volumeId = CreatePoolVolume( m_worldId, m_bounds, m_surfaceHeight, m_flowVelocity );

		// Totals survive Reset, so begin minus end always counts the bodies in fluid
		m_beginCount = 0;
		m_endCount = 0;
		m_beginTotal = 0;
		m_endTotal = 0;

		m_showFraction = true;
		m_itemCount = 0;
		CreateItems();
	}

	void CreateItems()
	{
		for ( int i = 0; i < m_itemCount; ++i )
		{
			b3DestroyBody( m_items[i].bodyId );
		}

		m_itemCount = 0;
		m_splashIndex = 0;
		for ( int i = 0; i < e_maxSplashes; ++i )
		{
			m_splashes[i] = { b3Pos_zero, 0.0f, e_splashSteps };
		}

		g_randomSeed = RAND_SEED;

		int kindCount = IM_ARRAYSIZE( s_poolKinds );
		for ( int i = 0; i < kindCount; ++i )
		{
			int column = i % 4;
			int row = i / 4;
			b3Pos position = { -3.75f + 2.5f * column, m_surfaceHeight + 1.0f + 0.4f * row,
							   -2.5f + 2.5f * row };
			AddItem( i, position, RandomQuat() );
		}
	}

	void AddItem( int kindIndex, b3Pos position, b3Quat rotation )
	{
		if ( m_itemCount == e_maxItems )
		{
			// The oldest item makes room
			b3DestroyBody( m_items[0].bodyId );
			memmove( m_items, m_items + 1, ( e_maxItems - 1 ) * sizeof( FluidItem ) );
			m_itemCount -= 1;
		}

		m_items[m_itemCount] =
			CreateFluidItem( m_worldId, s_poolKinds, kindIndex, position, rotation );
		m_itemCount += 1;
	}

	void DropItem()
	{
		b3Vec3 lower = m_bounds.lowerBound;
		b3Vec3 upper = m_bounds.upperBound;
		int kindIndex = RandomInt() % IM_ARRAYSIZE( s_poolKinds );
		b3Pos position = { RandomFloatRange( lower.x + 1.0f, upper.x - 1.0f ), upper.y + 1.5f,
						   RandomFloatRange( lower.z + 1.0f, upper.z - 1.0f ) };
		AddItem( kindIndex, position, RandomQuat() );
	}

	bool DrawControls() override
	{
		ImGuiSliderFlags flags = ImGuiSliderFlags_AlwaysClamp;
		if ( ImGui::SliderFloat( "Density", &m_density, 0.0f, 3000.0f, "%.0f kg/m3", flags ) )
		{
			b3FluidVolume_SetDensity( m_volumeId, m_density );
		}

		bool dragChanged =
			ImGui::SliderFloat( "Linear Drag", &m_linearDrag, 0.0f, 5.0f, "%.2f", flags );
		dragChanged |=
			ImGui::SliderFloat( "Angular Drag", &m_angularDrag, 0.0f, 5.0f, "%.2f", flags );
		dragChanged |=
			ImGui::SliderFloat( "Quadratic Drag", &m_quadraticDrag, 0.0f, 2.0f, "%.2f", flags );
		if ( dragChanged )
		{
			b3FluidVolume_SetDrag( m_volumeId, m_linearDrag, m_angularDrag, m_quadraticDrag );
		}

		if ( ImGui::SliderFloat( "Surface", &m_surfaceHeight, 0.25f, m_bounds.upperBound.y,
								 "%.2f m", flags ) )
		{
			b3FluidVolume_SetSurface( m_volumeId, { b3Vec3_axisY, m_surfaceHeight } );
		}

		bool flowChanged =
			ImGui::SliderFloat( "Flow X", &m_flowVelocity.x, -3.0f, 3.0f, "%.2f m/s", flags );
		flowChanged |=
			ImGui::SliderFloat( "Flow Z", &m_flowVelocity.z, -3.0f, 3.0f, "%.2f m/s", flags );
		if ( flowChanged )
		{
			b3FluidVolume_SetFlowVelocity( m_volumeId, m_flowVelocity );
		}

		ImGui::Checkbox( "Show Fraction", &m_showFraction );

		if ( ImGui::Button( "Drop" ) )
		{
			DropItem();
		}

		ImGui::SameLine();

		if ( ImGui::Button( "Reset" ) )
		{
			CreateItems();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_didStep == false )
		{
			return;
		}

		for ( int i = 0; i < e_maxSplashes; ++i )
		{
			m_splashes[i].age = b3MinInt( m_splashes[i].age + 1, e_splashSteps );
		}

		b3FluidEvents events = b3World_GetFluidEvents( m_worldId );
		m_beginCount = events.beginCount;
		m_endCount = events.endCount;
		m_beginTotal += events.beginCount;
		m_endTotal += events.endCount;

		for ( int i = 0; i < events.beginCount; ++i )
		{
			const b3FluidBeginTouchEvent* event = events.beginEvents + i;
			m_splashes[m_splashIndex] = { event->point, b3Length( event->velocity ), 0 };
			m_splashIndex = ( m_splashIndex + 1 ) % e_maxSplashes;
		}
	}

	void Render() override
	{
		DrawWater( m_bounds, m_surfaceHeight );

		// Buoyancy / weight = density * fraction / (SG * water density), weight is a 0.5 m arrow
		Vec4 buoyancyColor = MakeColor( b3_colorAqua );
		Vec4 weightColor = MakeColor( b3_colorFuchsia );
		Vec4 textColor = MakeColor( b3_colorWhite );
		float waterDensity = GetWaterDensity();
		int inFluidCount = 0;
		for ( int i = 0; i < m_itemCount; ++i )
		{
			const FluidItem* item = m_items + i;
			if ( B3_IS_NULL( b3Body_GetFluidVolume( item->bodyId ) ) )
			{
				continue;
			}

			inFluidCount += 1;
			float fraction = b3Body_GetSubmergedFraction( item->bodyId );
			float ratio = m_density * fraction / ( item->specificGravity * waterDensity );
			b3Pos center = b3Body_GetWorldPoint( item->bodyId, item->localCenter );
			DrawArrowEx( center, center + b3Vec3{ 0.0f, 0.5f * ratio, 0.0f }, buoyancyColor, 2.0f,
						 OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM, DEFAULT_ARROW_HEAD_FRAC );
			DrawArrowEx( center, center + b3Vec3{ 0.0f, -0.5f, 0.0f }, weightColor, 2.0f,
						 OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM, DEFAULT_ARROW_HEAD_FRAC );

			if ( m_showFraction )
			{
				DrawString3D( center, textColor, "%.2f", fraction );
			}
		}

		for ( int i = 0; i < e_maxSplashes; ++i )
		{
			const Splash* splash = m_splashes + i;
			if ( splash->age >= e_splashSteps )
			{
				continue;
			}

			float t = float( splash->age ) / float( e_splashSteps );
			float radius = ( 0.2f + 0.1f * splash->speed ) * ( 0.25f + t );
			DrawRing( splash->point, radius, MakeColorAlpha( b3_colorWhite, 1.0f - t ) );
		}

		b3FluidStats stats = b3World_GetFluidStats( m_worldId );
		DrawTextLine( "fluid stage %.3f ms, volumes %d, awake in fluid %d, clipped shapes %d",
					  stats.time, stats.volumeCount, stats.bodyCount, stats.clippedShapeCount );
		DrawTextLine( "items %d, in fluid %d", m_itemCount, inFluidCount );
		DrawTextLine( "begin events %d (total %d), end events %d (total %d)", m_beginCount,
					  m_beginTotal, m_endCount, m_endTotal );
		DrawTextLine( "gold floats, red sinks, buoyancy (aqua) against weight (fuchsia)" );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Buoyancy( context );
	}

	b3AABB m_bounds;
	b3FluidVolumeId m_volumeId;
	float m_density;
	float m_linearDrag;
	float m_angularDrag;
	float m_quadraticDrag;
	float m_surfaceHeight;
	b3Vec3 m_flowVelocity;
	bool m_showFraction;

	FluidItem m_items[e_maxItems];
	int m_itemCount;

	Splash m_splashes[e_maxSplashes];
	int m_splashIndex;

	int m_beginCount;
	int m_endCount;
	int m_beginTotal;
	int m_endTotal;
};

static int sampleBuoyancy = RegisterSample( "Fluid", "Buoyancy", Buoyancy::Create );

static const FluidItemKind s_channelKinds[] = {
	{ "cork", e_fluidSphere, { 0.12f, 0.0f, 0.0f }, 0.25f },
	{ "plank", e_fluidBox, { 0.4f, 0.04f, 0.12f }, 0.5f },
	{ "log", e_fluidCapsule, { 0.4f, 0.15f, 0.0f }, 0.7f },
	{ "crate", e_fluidBox, { 0.2f, 0.2f, 0.2f }, 1.15f },
	{ "pebble", e_fluidRock, { 0.12f, 0.0f, 0.0f }, 2.6f },
	{ "boulder", e_fluidRock, { 0.35f, 0.0f, 0.0f }, 2.6f },
	{ "shot", e_fluidSphere, { 0.08f, 0.0f, 0.0f }, 7.8f },
};

// A channel with a current. Light and broad items drift with the flow, dense ones sink and lag.
class Current : public Sample
{
public:
	enum
	{
		e_maxItems = 300,
		e_kindCount = IM_ARRAYSIZE( s_channelKinds ),
	};

	explicit Current( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -25.0f, 30.0f, 40.0f, { 0.0f, 1.0f, 0.0f } );
		}

		m_stepWhilePaused = false;

		AddGroundBox( 30.0f );

		// Closed upstream at -x and open downstream at +x
		m_bounds = { { -20.0f, 0.0f, -2.0f }, { 20.0f, 3.0f, 2.0f } };
		CreateBasin( m_worldId, m_bounds, true );

		m_surfaceHeight = 2.0f;
		m_flowSpeed = 1.5f;
		m_volumeId =
			CreatePoolVolume( m_worldId, m_bounds, m_surfaceHeight, { m_flowSpeed, 0.0f, 0.0f } );

		m_spawnInterval = 8;
		m_maxCount = 120;
		m_maxAge = 40.0f;
		m_itemCount = 0;
		m_beginTotal = 0;
		m_endTotal = 0;
	}

	void SpawnItem()
	{
		int kindIndex = RandomInt() % e_kindCount;
		float x = m_bounds.lowerBound.x + RandomFloatRange( 1.0f, 2.5f );
		float z = RandomFloatRange( m_bounds.lowerBound.z + 0.5f, m_bounds.upperBound.z - 0.5f );
		b3Pos position = { x, m_surfaceHeight + 1.0f, z };
		m_items[m_itemCount] =
			CreateFluidItem( m_worldId, s_channelKinds, kindIndex, position, RandomQuat() );
		m_itemCount += 1;
	}

	bool DrawControls() override
	{
		ImGuiSliderFlags flags = ImGuiSliderFlags_AlwaysClamp;
		if ( ImGui::SliderFloat( "Flow", &m_flowSpeed, 0.0f, 5.0f, "%.1f m/s", flags ) )
		{
			b3FluidVolume_SetFlowVelocity( m_volumeId, { m_flowSpeed, 0.0f, 0.0f } );
		}

		ImGui::SliderInt( "Spawn Interval", &m_spawnInterval, 1, 60, "%d steps", flags );
		ImGui::SliderInt( "Max Items", &m_maxCount, 10, e_maxItems, "%d", flags );
		ImGui::SliderFloat( "Max Age", &m_maxAge, 5.0f, 120.0f, "%.0f s", flags );
		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_didStep == false )
		{
			return;
		}

		b3FluidEvents events = b3World_GetFluidEvents( m_worldId );
		m_beginTotal += events.beginCount;
		m_endTotal += events.endCount;

		// Recycle items that passed the open end, fell off the ground or lingered too long
		int maxAgeSteps = int( m_maxAge * m_context->hertz );
		float endX = m_bounds.upperBound.x + 0.5f;
		int count = 0;
		for ( int i = 0; i < m_itemCount; ++i )
		{
			FluidItem item = m_items[i];
			item.age += 1;

			b3Pos center = b3Body_GetWorldPoint( item.bodyId, item.localCenter );
			if ( center.x > endX || center.y < -5.0f || item.age > maxAgeSteps )
			{
				b3DestroyBody( item.bodyId );
				continue;
			}

			m_items[count] = item;
			count += 1;
		}

		m_itemCount = count;

		if ( m_stepCount % m_spawnInterval == 0 && m_itemCount < m_maxCount )
		{
			SpawnItem();
		}
	}

	void Render() override
	{
		DrawWater( m_bounds, m_surfaceHeight );

		Vec4 flowColor = MakeColor( b3_colorLightSkyBlue );
		for ( float x = m_bounds.lowerBound.x + 2.0f; x < m_bounds.upperBound.x; x += 6.0f )
		{
			b3Pos p = { x, m_surfaceHeight + 0.05f, 0.0f };
			DrawArrow( p, p + b3Vec3{ 0.5f * m_flowSpeed, 0.0f, 0.0f }, flowColor );
		}

		int counts[e_kindCount] = {};
		float sumX[e_kindCount] = {};
		float sumSpeed[e_kindCount] = {};
		for ( int i = 0; i < m_itemCount; ++i )
		{
			const FluidItem* item = m_items + i;
			if ( B3_IS_NULL( b3Body_GetFluidVolume( item->bodyId ) ) )
			{
				continue;
			}

			b3Pos center = b3Body_GetWorldPoint( item->bodyId, item->localCenter );
			counts[item->kind] += 1;
			sumX[item->kind] += float( center.x );
			sumSpeed[item->kind] += b3Body_GetLinearVelocity( item->bodyId ).x;
		}

		DrawTextLine( "flow %.1f m/s, light and broad items drift, dense ones sink and lag",
					  m_flowSpeed );
		for ( int i = 0; i < e_kindCount; ++i )
		{
			const FluidItemKind* kind = s_channelKinds + i;
			if ( counts[i] == 0 )
			{
				DrawTextLine( "%s (SG %.2f): none in water", kind->name, kind->specificGravity );
				continue;
			}

			float invCount = 1.0f / float( counts[i] );
			DrawTextLine( "%s (SG %.2f): %d in water, mean x %.1f m, mean speed %.2f m/s",
						  kind->name, kind->specificGravity, counts[i], invCount * sumX[i],
						  invCount * sumSpeed[i] );
		}

		b3FluidStats stats = b3World_GetFluidStats( m_worldId );
		DrawTextLine(
			"items %d, awake in fluid %d, fluid stage %.3f ms, begin events %d, end events %d",
			m_itemCount, stats.bodyCount, stats.time, m_beginTotal, m_endTotal );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Current( context );
	}

	b3AABB m_bounds;
	b3FluidVolumeId m_volumeId;
	float m_surfaceHeight;
	float m_flowSpeed;
	int m_spawnInterval;
	int m_maxCount;
	float m_maxAge;

	FluidItem m_items[e_maxItems];
	int m_itemCount;

	int m_beginTotal;
	int m_endTotal;
};

static int sampleCurrent = RegisterSample( "Fluid", "Current", Current::Create );

static const FluidItemKind s_stressKinds[] = {
	{ "ball", e_fluidSphere, { 0.2f, 0.0f, 0.0f }, 0.4f },
	{ "box", e_fluidBox, { 0.18f, 0.18f, 0.18f }, 0.6f },
	{ "capsule", e_fluidCapsule, { 0.15f, 0.12f, 0.0f }, 0.8f },
	{ "heavy ball", e_fluidSphere, { 0.15f, 0.0f, 0.0f }, 2.5f },
	{ "heavy box", e_fluidBox, { 0.15f, 0.15f, 0.15f }, 1.5f },
};

// Tiles per side of the pool, and dry decoy volumes beside it
static const int s_stressTileSides[] = { 1, 2, 4, 8 };
static const char* s_stressTileLabels[] = { "1", "4", "16", "64" };
static const int s_stressDecoyCounts[] = { 0, 10, 50 };
static const char* s_stressDecoyLabels[] = { "0", "10", "50" };
static const float s_stressDecoySurface = 0.6f;

// Survive the restart that a worker count change triggers
static int s_fluidStressCount = 2000;
static int s_fluidStressTileIndex = 0;
static int s_fluidStressDecoyIndex = 0;

// Thousands of bodies dropped into a big pool. The world uses the UI worker count. The pool water
// can be split into abutting tiles that share every setting, so only the volume count changes.
class FluidStress : public Sample
{
public:
	enum
	{
		e_windowSize = 60,
	};

	explicit FluidStress( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( -30.0f, 35.0f, 70.0f, { 0.0f, 2.0f, 0.0f } );
		}

		m_stepWhilePaused = false;
		m_shadowSplitFar = 150.0f;

		m_bounds = { { -20.0f, 0.0f, -20.0f }, { 20.0f, 4.0f, 20.0f } };
		m_baseHeight = 3.0f;
		m_tide = true;
		CreateScene();
	}

	void CreateScene()
	{
		ResetProfile();

		m_bodyCount = s_fluidStressCount;
		m_surfaceHeight = m_baseHeight;
		m_tideTime = 0.0f;
		m_fluidPeak = 0.0f;
		m_stepPeak = 0.0f;

		b3Capacity capacity = {};
		capacity.staticBodyCount = 4;
		capacity.staticShapeCount = 8;
		capacity.dynamicBodyCount = m_bodyCount;
		capacity.dynamicShapeCount = m_bodyCount;
		capacity.contactCount = 4 * m_bodyCount;
		CreateWorld( &capacity );

		AddGroundBox( 30.0f );
		CreateBasin( m_worldId, m_bounds, false );

		b3Vec3 lower = m_bounds.lowerBound;
		b3Vec3 upper = m_bounds.upperBound;

		// The tie-break puts a body on a seam in one tile, and every tile acts the same
		m_tileSide = s_stressTileSides[s_fluidStressTileIndex];
		m_tileIds.clear();
		for ( int i = 0; i < m_tileSide * m_tileSide; ++i )
		{
			b3AABB tile = GetTileBounds( i % m_tileSide, i / m_tileSide );
			m_tileIds.push_back(
				CreatePoolVolume( m_worldId, tile, m_surfaceHeight, b3Vec3_zero ) );
		}

		// Shallow pools in a column on each side of the basin
		int decoyCount = s_stressDecoyCounts[s_fluidStressDecoyIndex];
		float pitch = ( upper.z - lower.z ) / float( b3MaxInt( ( decoyCount + 1 ) / 2, 1 ) );
		m_decoyBounds.clear();
		for ( int i = 0; i < decoyCount; ++i )
		{
			float x = ( i % 2 == 0 ? 1.0f : -1.0f ) * ( upper.x + 4.0f );
			float z = lower.z + pitch * ( float( i / 2 ) + 0.5f );
			float halfDepth = 0.4f * pitch;
			b3AABB decoy = { { x - 1.0f, 0.0f, z - halfDepth }, { x + 1.0f, 1.0f, z + halfDepth } };
			m_decoyBounds.push_back( decoy );
			CreatePoolVolume( m_worldId, decoy, s_stressDecoySurface, b3Vec3_zero );
		}

		g_randomSeed = RAND_SEED;

		float spacing = 0.8f;
		int columnCount = int( ( upper.x - lower.x - 1.0f ) / spacing ) + 1;
		int rowCount = int( ( upper.z - lower.z - 1.0f ) / spacing ) + 1;
		int layerSize = columnCount * rowCount;
		int kindCount = IM_ARRAYSIZE( s_stressKinds );

		std::vector<b3BodyId> bodyIds( m_bodyCount );
		std::vector<b3BodyFluidDef> fluidDefs( m_bodyCount );
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			int layer = i / layerSize;
			int row = ( i - layer * layerSize ) / columnCount;
			int column = i - layer * layerSize - row * columnCount;
			b3Pos position = { lower.x + 0.5f + spacing * column,
							   m_surfaceHeight + 1.5f + spacing * layer,
							   lower.z + 0.5f + spacing * row };

			const FluidItemKind* kind = s_stressKinds + RandomInt() % kindCount;
			bodyIds[i] = CreateFluidBody( m_worldId, kind, position, RandomQuat(), false );
			fluidDefs[i] = MakeSpecificGravityDef( bodyIds[i], kind->specificGravity );
		}

		b3World_SetBodyFluidDefs( m_worldId, bodyIds.data(), fluidDefs.data(), m_bodyCount );

		m_windowIndex = 0;
		m_windowCount = 0;
	}

	// Abutting tiles share each edge exactly, and the outer ones end on the pool bounds
	b3AABB GetTileBounds( int ix, int iz ) const
	{
		b3Vec3 lower = m_bounds.lowerBound;
		b3Vec3 upper = m_bounds.upperBound;
		float width = ( upper.x - lower.x ) / float( m_tileSide );
		float depth = ( upper.z - lower.z ) / float( m_tileSide );

		b3AABB tile = m_bounds;
		tile.lowerBound.x = lower.x + width * float( ix );
		tile.lowerBound.z = lower.z + depth * float( iz );
		tile.upperBound.x = ix + 1 < m_tileSide ? lower.x + width * float( ix + 1 ) : upper.x;
		tile.upperBound.z = iz + 1 < m_tileSide ? lower.z + depth * float( iz + 1 ) : upper.z;
		return tile;
	}

	bool DrawControls() override
	{
		ImGui::SliderInt( "Bodies", &s_fluidStressCount, 500, 5000, "%d",
						  ImGuiSliderFlags_AlwaysClamp );
		if ( ImGui::IsItemDeactivatedAfterEdit() )
		{
			CreateScene();
		}

		if ( ImGui::Combo( "Volumes", &s_fluidStressTileIndex, s_stressTileLabels,
						   IM_ARRAYSIZE( s_stressTileLabels ) ) )
		{
			CreateScene();
		}

		if ( ImGui::Combo( "Decoys", &s_fluidStressDecoyIndex, s_stressDecoyLabels,
						   IM_ARRAYSIZE( s_stressDecoyLabels ) ) )
		{
			CreateScene();
		}

		ImGui::Checkbox( "Tide", &m_tide );

		if ( ImGui::Button( "Reset" ) )
		{
			CreateScene();
		}

		return true;
	}

	void Step() override
	{
		// The tide moves the surface, which wakes the floaters and keeps the stage busy
		bool shouldStep = m_context->pause == false || m_context->singleStep > 0;
		if ( shouldStep && m_tide && m_context->hertz > 0.0f )
		{
			m_tideTime += 1.0f / m_context->hertz;
			m_surfaceHeight =
				m_baseHeight + 0.25f * b3ComputeCosSin( 2.0f * B3_PI * m_tideTime / 6.0f ).sine;
			for ( b3FluidVolumeId tileId : m_tileIds )
			{
				b3FluidVolume_SetSurface( tileId, { b3Vec3_axisY, m_surfaceHeight } );
			}
		}

		Sample::Step();

		if ( m_didStep == false )
		{
			return;
		}

		float fluidTime = b3World_GetFluidStats( m_worldId ).time;
		float stepTime = m_profiles[m_currentProfileIndex].step;
		m_fluidTimes[m_windowIndex] = fluidTime;
		m_stepTimes[m_windowIndex] = stepTime;
		m_windowIndex = ( m_windowIndex + 1 ) % e_windowSize;
		m_windowCount = b3MinInt( m_windowCount + 1, e_windowSize );
		m_fluidPeak = b3MaxFloat( m_fluidPeak, fluidTime );
		m_stepPeak = b3MaxFloat( m_stepPeak, stepTime );
	}

	void Render() override
	{
		DrawWater( m_bounds, m_surfaceHeight );

		Vec4 seamColor = MakeColorAlpha( b3_colorLightSkyBlue, 0.5f );
		b3Vec3 lower = m_bounds.lowerBound;
		b3Vec3 upper = m_bounds.upperBound;
		for ( int i = 1; i < m_tileSide; ++i )
		{
			b3AABB tile = GetTileBounds( i, i );
			float x = tile.lowerBound.x;
			float z = tile.lowerBound.z;
			DrawLine( { x, m_surfaceHeight, lower.z }, { x, m_surfaceHeight, upper.z }, seamColor );
			DrawLine( { lower.x, m_surfaceHeight, z }, { upper.x, m_surfaceHeight, z }, seamColor );
		}

		for ( const b3AABB& decoy : m_decoyBounds )
		{
			DrawWater( decoy, s_stressDecoySurface );
		}

		float fluidSum = 0.0f;
		float stepSum = 0.0f;
		for ( int i = 0; i < m_windowCount; ++i )
		{
			fluidSum += m_fluidTimes[i];
			stepSum += m_stepTimes[i];
		}

		float scale = m_windowCount > 0 ? 1.0f / float( m_windowCount ) : 0.0f;
		b3FluidStats stats = b3World_GetFluidStats( m_worldId );
		b3Profile profile = b3World_GetProfile( m_worldId );
		int awakeCount = b3World_GetAwakeBodyCount( m_worldId );

		DrawTextLine( "bodies %d, awake %d, awake in fluid %d, clipped shapes %d", m_bodyCount,
					  awakeCount, stats.bodyCount, stats.clippedShapeCount );
		DrawTextLine( "fluid stage %.2f ms (average %.2f, peak %.2f)", stats.time, scale * fluidSum,
					  m_fluidPeak );
		DrawTextLine( "step %.2f ms (average %.2f, peak %.2f)", profile.step, scale * stepSum,
					  m_stepPeak );
		DrawTextLine( "volumes %d: %d pool tiles, %d dry decoys", stats.volumeCount,
					  int( m_tileIds.size() ), int( m_decoyBounds.size() ) );
		DrawTextLine( "workers %d, sub-steps %d, surface %.2f m", m_context->workerCount,
					  m_context->subStepCount, m_surfaceHeight );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FluidStress( context );
	}

	b3AABB m_bounds;
	std::vector<b3FluidVolumeId> m_tileIds;
	std::vector<b3AABB> m_decoyBounds;
	int m_tileSide;
	float m_baseHeight;
	float m_surfaceHeight;
	float m_tideTime;
	bool m_tide;
	int m_bodyCount;

	float m_fluidTimes[e_windowSize];
	float m_stepTimes[e_windowSize];
	int m_windowIndex;
	int m_windowCount;
	float m_fluidPeak;
	float m_stepPeak;
};

static int sampleFluidStress = RegisterSample( "Fluid", "Fluid Stress", FluidStress::Create );
