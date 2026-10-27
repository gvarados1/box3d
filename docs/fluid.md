# Fluid stage (MineMogul fork)

Buoyancy, drag and currents for dynamic bodies inside fluid volumes, computed natively inside
`b3World_Step`. Only awake bodies cost anything, and a world without fluid volumes runs exactly
as before (bit-identical).

## Goals

- Bodies float, sink and settle, with correct equilibrium depth and righting torque.
- Drag and currents that stay stable at water density for any body.
- Thousands of bodies in fluid: work is per awake body, multithreaded, and deterministic for any
  worker count and across platforms.
- Enter and exit events for splashes and gameplay.
- No layout change to any struct the Unity binding mirrors, and minimal edits to upstream files.

Not in v1 (the API leaves room): per-cell flow grids and surface heights (rivers), waves,
orientation-dependent per-face drag and lift.

## Public API (`include/box3d/fluid.h`, included by `box3d.h`)

```c
typedef struct b3FluidVolumeId { int32_t index1; uint16_t world0; uint16_t generation; } b3FluidVolumeId;

typedef struct b3FluidVolumeDef
{
	b3AABB bounds;          // region the fluid can occupy (world space)
	b3Plane surface;        // fluid fills bounds below this plane; normal points out of the fluid (unit length)
	float density;          // default 1000 / lengthUnits^3 (water)
	float linearDrag;       // linear damping rate (1/s) at full submersion, default 1.5
	float angularDrag;      // angular damping rate (1/s) at full submersion, default 1.5
	float quadraticDrag;    // Cd for 0.5 * density * Cd * A * |v - u|^2, A = submergedVolume^(2/3), default 1
	b3Vec3 flowVelocity;    // current
	uint64_t maskBits;      // only shapes whose filter.categoryBits match are buoyant
	void* userData;
	int internalValue;
} b3FluidVolumeDef;

typedef struct b3BodyFluidDef
{
	float displacedVolume;  // fluid displaced at full submersion; 0 = the body's buoyant shape volume
	float dragScale;        // multiplies the volume's drag on this body
	bool enabled;           // false: fluids ignore this body
} b3BodyFluidDef;

typedef struct b3FluidBeginTouchEvent { b3BodyId bodyId; b3FluidVolumeId volumeId; b3Pos point; b3Vec3 velocity; } b3FluidBeginTouchEvent;
typedef struct b3FluidEndTouchEvent { b3BodyId bodyId; b3FluidVolumeId volumeId; } b3FluidEndTouchEvent;
typedef struct b3FluidEvents { b3FluidBeginTouchEvent* beginEvents; b3FluidEndTouchEvent* endEvents; int beginCount; int endCount; } b3FluidEvents;
typedef struct b3FluidStats { float time; int volumeCount; int bodyCount; int clippedShapeCount; } b3FluidStats;
```

The functions are declared in the header: create, destroy, IsValid, setters and getters for
volumes; the body def setter and getter plus a batch setter; submerged fraction and current
volume queries; the point query; events; and stats. Everything is new: `b3WorldDef`,
`b3BodyDef`, `b3ShapeDef`, `b3Profile`, `b3Counters` and the existing event structs keep their
layout. All mutators require an unlocked world, like every other mutator.

## Semantics

**Fluid world.**
- `world->fluid` is created by the first fluid mutator: volume create, or a body def set.
- It is freed with the world, and by a snapshot restore whose image has no fluid section.
  Destroying the last volume keeps it, along with every body def and the volume id generations, so
  ids never repeat.

**Classification.** Each volume has a vertical axis `k`: the bounds axis most aligned with the
surface normal. A dynamic body is a candidate for a volume when its center of mass (at the
predicted pose, below):
- is inside the bounds on the two axes other than `k`;
- is at or above the bounds' bottom on `k`;
- is at or below the bounds' top plus `r = |maxExtent|` on `k`.

Among candidates:
1. Volumes whose bounds contain the center of mass win.
2. Then the highest surface above the center of mass.
3. Then the lowest volume index.

Within the chosen volume, the geometry is clipped by the surface plane only. A body whose center
of mass is outside every footprint gets no buoyancy. `b3World_FindFluidVolume` applies the same
rule with `r = 0`, so the player and items agree.

**Eligibility.** A body takes part only if all of these hold:
- it is dynamic;
- `sim->invMass > 0`;
- its def is enabled;
- the total geometric volume of its buoyant shapes is above 0;
- the resolved displaced volume is above 0.

Buoyant shapes are sphere, capsule and hull shapes that are not sensors and whose category bits
match `maskBits`. An ineligible body counts as not in fluid. The pass makes no write to its sim.

**Buoyancy.**
- `f` = submerged / total geometric buoyant volume, clamped to [0, 1]. The gate is
  `if (!(f > 0)) not in fluid`, so a NaN can never pass.
- `Fb = -density * Vd * f * gravityScale * gravity`, with `Vd` the def's `displacedVolume`, or the
  geometric volume when that is 0.
- The torque comes from the submerged first moment, so tiny volumes give tiny torques; there is
  no division by the volume.
- Mass never enters buoyancy. MineMogul sets `displacedVolume = mass / (specificGravity * waterDensity)`.

**Drag**, on the velocity relative to the flow `u`:
- `k = dragScale * (f * linearDrag + 0.5 * density * Cd * A * |v - u| * invMass)`, with
  `A = (f * Vgeom)^(2/3)`. Area over mass makes small dense items fall through faster and broad
  light ones drift.
- The substeps hold one rate, so with `n >= 2` a rate from the step-start speed would over-damp
  fast bodies. The pass predicts the end-of-step velocity `vp` with one full-dt backward-Euler step
  and replaces `|v - u|` by `n * s / (1 + y + ... + y^(n-1))`, with `s = |v - u|` and
  `y = (s / |vp - u|)^(1/n)`. Pure quadratic decay is then exact for any `n`, and at a terminal or
  steady-current speed `vp = v`, so those stay exact.
- Angular rate `ka = dragScale * f * angularDrag`.

## The predicted pose (stability)

The solver applies a held `sim->force` in every substep. With `n >= 2` substeps, a restoring force
held from the start-of-step pose adds energy every step: undamped bobbing grows, and floaters
never settle. All fluid geometry is therefore evaluated at a predicted pose:
- `center + beta * dt * v`;
- rotation advanced by `beta * dt * w`;
- `beta = (n - 1) / (2n)`, which is 0.25 for 2 substeps and 0 for 1.

That makes the held-force step map exactly energy-neutral for translation and for rotation about a
principal axis. Any drag then decays the motion. This covers classification, fast paths, `f`, the
centroid, the torque arm and the drag rates.

The solver also maps `sim->torque` through the start-of-step world inverse inertia in every
substep. For an anisotropic body rotating off a principal axis that inertia lags the body, and the
first-order error pumps energy into yaw (a thin plank at zero drag went from 1.5 J to 10 J in
60 s at 50 Hz with 2 substeps). So the pass converts the buoyancy torque to its angular
acceleration through the inertia at the predicted rotation, and writes the torque that the
start-of-step inertia maps to that acceleration. User torque is left as is.

The limit is `w * dt < 2`, with `w^2 = g / (SG * H)` for a body of height `H` and specific
gravity `SG`. At 50 Hz that is about 1 mm of `SG * H`; anything thinner needs more drag.

`f`, `b3Body_GetSubmergedFraction` and the events describe the predicted pose.

## Step placement and data flow

In `b3World_Step`:

1. `b3Collide` runs (the awake set is final after it).
2. **`b3FluidPreSolve(world, &context)`.**
   - When `world->fluid` exists, its first statement clears the begin events, before any early
     return. That is after `b3Collide`, which is safe because the world is locked.
   - It returns early when the fluid world is missing, there are no volumes, or `timeStep == 0`.
   - Otherwise it sizes the side table and the per-worker "changed" bitsets to the body id capacity.
   - It runs `b3ParallelFor` over the awake sims (task name "fluid"). Every awake dynamic body is
     classified through the volume lookup (below); a body with no candidate and no recorded volume
     gets no write.
   - For the rest, it computes the fluid terms at the predicted pose and writes:
     - the body's side-table entry (new volume, `f`, begin-event payload, saved damping);
     - when in fluid, the sim: damping raised to `C = c + k`, `Ca = ca + ka`;
     - when in fluid, a force `X = (Fb + m*k*u - h*C*(Fuser + m*gs*g)) / (1 + h*C)`;
     - when in fluid, the torque scaled to `(Tuser + Tb) / (1 + h*Ca)`.
   - With the solver's `v' = h*F/m + h*gs*g + v/(1 + h*c)`, this makes every substep a
     backward-Euler step on all non-constraint accelerations, with the drag rate held for the step:
     `v' = (v + h*(a + k*u)) / (1 + h*C)`. It is stable for any drag, terminal speeds are exact, and
     steady currents are exact.
   - It sets the body's bit in its worker's bitset when the body is in fluid this step or its
     volume changed.
3. `b3Solve` runs (unchanged).
4. **`b3FluidPostSolve(world)`.** It only runs when this step's PreSolve ran. It unions the
   per-worker bitsets serially, O(workers x bodyCapacity/64), then walks the union in body-id order
   (CTZ) and:
   - restores the saved damping (looked up by body id, because bodies may have moved to a
     sleeping set);
   - emits begin and end events from the entries;
   - clears the per-step flags.
5. **`b3FluidSwapEvents(world)`**, right after the end-event buffer swap, on every step while the
   fluid world exists. It clears the fluid end buffer that becomes current.

`solver.c` is untouched. A body not in fluid gets no write, so the existing goldens and the
bitwise mix tests stay identical.

## Per-body side table

The table is fork-owned, lives in the fluid world, and is indexed by body id.

**Zero means default.** An all-zero entry means "no def, not in fluid". A flags bit marks a stored
def, and the volume is stored as a full `b3FluidVolumeId`, where 0 means none. Every growth path
memsets the new range to zero: PreSolve, the def setters, and snapshot restore.

**Lifecycle.** The helper `b3FluidRemoveBody(world, bodyId)` pushes an end event when the body is
in fluid, then clears its fluid state but keeps the def. It is called from:
- `b3DestroyBody`, before the id is freed (it resets the whole entry);
- `b3Body_Disable`;
- `b3Body_SetType` when the new type is not dynamic;
- a def setter that sets `enabled = false`.

Because the destroy hook resets entries, no generation validation is needed. As a safety net, the
pass also ends an awake body that is non-dynamic, disabled for fluid or ineligible.

**Volume destroy** walks the table in body-id order. It pushes end events for the entries in that
volume, clears them, and wakes those bodies. Then the id is freed. The slot's generation is bumped
when it is reused, so ids never repeat.

## Volume lookup

Caves hold many volumes, so classification does not test every volume.
- Volumes whose axis `k` is y and whose x and z footprint is ordered, inside `B3_HUGE` and narrower
  than `B3_HUGE / 2` are binned. The rest (other axes, unbounded or inverted bounds) are tested for
  every body, where the exact test rejects an inverted footprint as before.
- The binned footprints share one x-z grid over their union, with square cells, about 1024 cells,
  and at most 256 along a side. Each cell lists the binned volumes whose footprint overlaps it, in
  ascending index order (compressed rows).
- A point's cell is `min((int)((p - lower) * inverseCellSize), count - 1)`, computed in double.
  That is monotonic in `p`, so a volume inserted from its footprint's end cells is listed in the
  cell of every point the exact footprint test accepts. The grid never drops a candidate.
- The candidates still go through the exact test, and the tie-break is independent of visiting
  order (containing bounds, then the highest surface, then the lowest index). Results are therefore
  bit-identical to testing every volume.
- The lookup is derived state: it is rebuilt on create, destroy, SetBounds, a SetSurface that
  changes the axis, and snapshot restore, all on the calling thread. It is never serialized.

## Geometry (deterministic math)

Plane distances are taken against the body frame, at the predicted pose. `fluid.c` uses only
`+ - * /`, `sqrtf`, `fabsf`, min/max, `b3Atan2` and a fixed-iteration Newton n-th root (the area
cube root and the substep drag root). It never calls another libm function, so results are
bit-identical across platforms and CRTs.

- **Sphere:** the spherical cap `V = pi h^2 (3r - h) / 3` with `h = clamp(r - d, 0, 2r)`, and its
  closed-form centroid.
- **Capsule:** one integral along the axis over `s` in `[-r, L + r]`, caps included.
  - **Discs:** each axis-perpendicular disc of radius `rho(s)` is fully wet, dry, or partly wet.
  - **Wet ranges** are closed-form: `pi r^2 * length` on the cylinder and `pi (r^2 t - t^3/3)` on the
    caps, with matching first moments.
  - **The transition band**, where `|d| < rho * sin(theta)`, is found analytically: linear on the
    cylinder, quadratic on the caps. It is integrated with Gauss-Legendre points, using the
    circular-segment area (half-angle via `b3Atan2`), its axial moment and its lateral moment.
  - The upright case is exact, and V(depth) is continuous.
- **Hull:** the vertices are classified first.
  - All below: `hull->volume` and `hull->center`.
  - All above: nothing.
  - Crossing: clip each face fan against the plane and sum the signed tetrahedra to an apex on the
    cap polygon (the mean of the edge-plane crossings). The cap lies on the plane, so it adds zero
    and is never built.
- **Clamping:** each shape's submerged volume is clamped to [0, its volume] before any use.

## Events

- **Keyed by (body, volume).** Begin events live until the next step. End events are
  double-buffered on `world->endEventArrayIndex`, like sensor end events.
- **No order between the arrays.** A consumer tracking one volume per body handles all ends before
  all begins. `b3Body_GetFluidVolume` is authoritative.
- **Begin payload:**
  - `velocity` is the center-of-mass velocity read by the pass before drag;
  - `point` is the submerged centroid projected onto the surface.
- **End payload:** for exit splashes, the same step's `b3BodyMoveEvent` carries the transform and
  velocity. End events may name destroyed bodies.

## Sleep and waking

- Sleeping bodies are not processed and keep their state. Floaters come to rest and sleep.
  Bodies in a current faster than their sleep threshold stay awake.
- One helper decides waking for every volume mutator. It queries the dynamic tree over the old and
  new bounds, each extended along `k` to the surface where the surface lies beyond them, then
  decides per body with `r = |maxExtent|`:
  - **SetSurface:** wake unless the body is fully below both the old and new planes, or fully above
    both, and touches no other volume.
  - **SetDensity, SetDrag, SetFlowVelocity:** wake bodies not fully above the surface.
  - **SetBounds, create and destroy:** wake the old and new regions, plus every body recorded in
    that volume.
- The body def setters wake a body that is in fluid when `enabled` or `displacedVolume` changes,
  and a sleeping body that is a candidate for a volume and not fully above its surface when
  `enabled` turns back on.
- **Cost:** for SetSurface, each sleeper that stays asleep is also checked against the other
  volumes that the lookup returns for its center, on the calling thread.

## Recording and snapshots

**Recording.**
- Every mutator except `b3FluidVolume_SetUserData` is a `B3_REC` op with a replay dispatcher, like
  upstream, which records no user-data setter. The ops are allocated downward from 0xFF, so an
  upstream merge that takes the same code fails to compile on a duplicate `case` in replay.c. The
  batch setter records one op per element, as in `batch.c`.
- No recording version bump: a minor mismatch already loads, and unknown opcodes are skipped.
- `b3HashWorldState` covers only body transforms and velocities, so lost fluid state that does not
  change motion (the in-fluid record that drives events) is not reported as a divergence. The
  recording tests compare fluid state themselves.

**Snapshots.**
- The fluid section is appended after the names section, only when the fluid world exists.
- It is flagged by a fork-reserved header bit (`0x80000000`). `B3_SNAP_VERSION` and the layout hash
  stay the same, so old recordings keep loading and fluid-free snapshots are byte-identical.
- It holds the volume id pool, the full volume array (free slots included, userData nulled) and the
  side table, with the struct sizes for its own check.

**Restore.**
- It always tears the fluid world down first (in `b3FreeLiveSimElements`), then rebuilds it when the
  flag is set. A rewind before the first volume therefore leaves no stale state.
- The side table stores full volume ids, so restore rewrites their `world0` to the restoring world.
  A replay usually runs in another world slot than the recording.
- An older build skips the section and the ops, and reports the divergence through the state hash.

## Profiling

`b3World_GetFluidStats` reports the stage time and counts. `b3Profile` is untouched, because C#
receives it by value.

## Files

- **New:** `include/box3d/fluid.h`, `src/fluid.c`, `src/fluid.h`, `test/test_fluid.c`,
  `samples/sample_fluid.cpp`.
- **One-line hooks:**
  - `physics_world.c`: PreSolve (with the begin clear), PostSolve, swap, world destroy, and the
    fluid bytes in the world byte count;
  - `physics_world.h`: a `b3FluidWorld* fluid` field;
  - `body.c`: destroy, disable and set-type hooks;
  - `box3d.h`: the include.
- **Also touched:** `recording_ops.inl`, `recording.h/.c`, `replay.c/.h` and `world_snapshot.c` for
  the ops and the section; `test_recording.c` for AllOps and the fluid round trips;
  `test_determinism.c` and `shared/determinism.c` for a fluid golden scene; the CMake lists.

## Tests

- **Equilibrium:** draft for a sphere, a box, a free capsule and a fixed-rotation upright capsule at
  several specific gravities; `displacedVolume` overrides; a plank floating flat; a bottom-heavy
  body righting itself; capsule V(depth) continuity at 0, 30, 60 and 90 degrees.
- **Stability:** at 50 Hz with 2 substeps, a 5 cm SG 0.5 box with zero drag gains no heave amplitude
  over 10 s, and a tilted plank gains no roll; with the default drag both fall asleep. A thin
  plank pitched off its principal axes gains no energy at zero drag, at 50 Hz/2 and 60 Hz/8, and
  with isotropicInertiaTolerance 1 at 30 Hz/2 and 50 Hz/2.
- **Motion:** sinking to the bottom; reaching the current's speed; terminal speeds against the
  analytic value, including gravityScale 2 and body damping; quadratic decay against the analytic
  value at 1, 2, 4 and 8 substeps, at rest and in a current.
- **Frames:** the surface normal along +x, +z and -y gives the same draft as +y; `gravityScale` 2
  gives the same draft as 1 and the same spin-up as doubled gravity; a `gravityScale` 0 plank,
  submerged or crossing the surface, stays at rest.
- **Guards:** shapes all failing `maskBits`, sensor-only, shapeless and zero-density bodies, and a
  hull vertex at -1e-6, all stay finite with no events.
- **Events:**
  - enter, leave and a volume switch;
  - destroy, disable, set type to kinematic and to static, and `enabled = false` on awake and
    sleeping bodies, with balanced begin and end counts;
  - no repeated begins after a dt == 0 step or after the last volume is destroyed;
  - the begin payload.
- **Defs:** a def set before the first volume takes effect; defs survive destroying the last
  volume; the batch setter, including a stale id; a destroyed body's def does not leak into the
  body that reuses its slot.
- **Waking:** a sleeping floater sinks after its density is halved; a submerged sleeper on the bed
  stays asleep through a surface change that keeps it submerged; volume create over a sleeper;
  bounds that shrink off a floater and grow over a sleeper; a `displacedVolume` change; `enabled`
  turned back on, which leaves a dry sleeper on a ledge above the surface asleep; a SetSurface
  that keeps a sunk sleeper below both planes still wakes it when another volume, binned or not,
  is a candidate, and it then moves to that volume when its surface is higher.
- **Classification:** a body outside a tank wall stays dry; abutting volumes at different levels;
  a body under the floor stays dry; the point query.
- **Volume lookup:** point queries and bodies match an all-volume scan of the documented rule in
  scenes of 8 to 200 volumes, at every bounds corner, edge and face and their float neighbours,
  far from the origin, and after every kind of create, destroy, SetBounds and axis change; 64
  abutting pools give bit-identical results with and without 30 dry decoys; inverted and extreme
  footprints neither crash nor match.
- **Determinism:**
  - a fluid golden scene (sleep step, plus a hash of transforms, velocities and the ordered event
    stream) with float and double goldens, for workers 1-5 and in SIMDWidthTest;
  - Ragdoll rerun with a containing volume whose surface is below the bodies, and with a
    created-then-destroyed volume, to show it still matches the golden.
- **Recording:** every new op in AllOps; a round trip with defs set before recording; a volume
  destroyed before recording plus a volume created during it; Restart, and a backward seek across
  a create; a replay in a different world slot than the recording; 20 volumes created, destroyed,
  re-bounded and re-axed during a recording, replayed at 1 and 4 workers with Restart and a seek.
- **Existing suite:** passes unchanged, goldens included.

## Sample

`Fluid` category in the samples app:

- **Buoyancy:** a pool with mixed shapes and specific gravities, and sliders for density, drag,
  surface height and flow.
- **Current:** items sorting in a channel.
- **Stress:** thousands of bodies, with the stage stats. A "Volumes" control splits the pool into 1,
  4, 16 or 64 abutting tiles and "Decoys" adds 0, 10 or 50 dry pools beside it, to show the stage
  cost staying flat as volumes are added.
