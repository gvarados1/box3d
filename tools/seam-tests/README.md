# Seam and contact tests

Headless programs that measure the conveyor-seam contact behaviour and guard the two rules added
for it. They complement the `Issues -> Conveyor Seam Ghost` sample, which shows the same thing
interactively; these give numbers you can diff between builds.

## The rules under test

Both live in `include/box3d/constants.h` and both can be overridden at compile time:

| constant | default | what it does |
| --- | --- | --- |
| `B3_CONVEX_REST_OFFSET` | `1 * B3_LINEAR_SLOP` (5 mm) | Convex contacts keep a small gap, matching what mesh contacts already did via `B3_MESH_REST_OFFSET`. Without it a body settles into the surface by the solver's residual penetration and then catches on the leading face of the next collider, so butted-together tiles act like a curb. |
| `B3_GRAZING_FACE_ALIGNMENT` | `0.9` (about 25 deg) | A contact whose normal is far off the supporting face of both shapes is riding a corner, not resting on a surface. While nothing is touching it is dropped; when the shapes really overlap it is kept but does not get the rest offset. Without it, sliding velocity gets rotated into vertical velocity and the body is launched at seams. |

Set `B3_GRAZING_FACE_ALIGNMENT` to `-1` to disable grazing handling entirely.

The grazing drop is skipped for a body flagged `b3_isFast`. A fast body covers more than half its
own inner radius in a step, so its speculative contact is the only thing keeping it out of the
geometry, and a body arriving corner first has no face pair to match yet - exactly the manifold
the rule would otherwise delete. `tunneling.c` measures what happens without that exemption.

Two related fixes in the continuous collision path are covered here as well. They are not behind
constants, so `-RulesOff` does not turn them off:

- `b3ShapeTimeOfImpact` retries against a sphere at the shape centroid when a convex-vs-convex
  query returns fraction 0, the same fallback the mesh path already used. A hull proxy carries
  radius 0, so `b3TimeOfImpact` reports overlap the instant the surfaces graze and reports a hit
  at t1 = 0 whenever the step starts within `B3_LINEAR_SLOP` of the other shape. Both come back as
  fraction 0, which `b3ContinuousQueryCallback` discards, and the body then advances the whole
  step. Rounded shapes were never affected because their proxy radius keeps the core clouds apart.

  That retry is guarded on the step's sweep translation exceeding the shape's inscribed radius. The
  guard is not decoration: "starts within the slop of a surface" is the resting state of anything
  sitting on something, so without it every settled body moving fast enough to be flagged
  `b3_isFast` pays for a second time-of-impact query on every step. On a conveyor that is the whole
  awake set, and it measured at **+15% step time**. A body that moves less than its own inscribed
  radius in a step cannot end up on the far side of anything, so skipping the retry there costs no
  coverage - `tunneling.c` still reports 0.

## Building

```
.\build.ps1                 # rules on, build and run everything
.\build.ps1 -RulesOff       # comparison build with both rules disabled
.\build.ps1 -GhostCullOff   # comparison build with B3_CULL_CONVEX_EDGE_GHOSTS=0 (see ledge_launch.c)
.\build.ps1 -NoRun          # build only
```

Output goes to `%TEMP%\box3d-seam-tests` by default (`-BuildRoot` to change it), so the repo
stays clean. The script finds `vcvars64.bat` itself; edit the candidate list if your Visual
Studio lives elsewhere.

To compare, run both flavors and diff the output. That is the point of the harness: almost every
number here should be *identical* between the two builds, except the seam numbers.

## The programs

### `seam_ghost.c`

The repro. A conveyor run of ten 1 m tiles, each its own static body, butted together with
coplanar top faces, driven by `tangentVelocity`. An item rides across the seams. Reports launch
count, worst upward speed, worst rise, and average speed along the belt.

Covers five belt constructions (separate hulls, one merged hull, separate meshes, one merged
mesh, hulls feeding a mesh corner) crossed with three item shapes (box, chamfered box, sphere),
then sweeps belt speed and item yaw over the seamed constructions and prints only failures.

A merged collider has no seam and is the control: it should never launch and should always run
at exactly belt speed.

Expected with the rules on: 0 launches everywhere, average speed equal to belt speed, and a
handful of swept configurations at worst showing sub-centimetre residual rise. With the rules
off, expect launches up to ~1 m/s, rises over 20 cm, and 10-20% of belt throughput lost.

`seam_ghost.exe -v` dumps manifolds at each launch. `seam_ghost.exe -c <speed> <yaw>` runs one
chamfered-box configuration verbosely.

### `regress.c`

Convex contact regression. Stacks of 5 and 10 boxes, drops from 1 m and 5 m, a box on a 10 deg
and a 40 deg ramp, and impacts into a static wall at 5, 20 and 50 m/s. Guards against the rest
offset destabilising stacking and against grazing rejection letting things tunnel.

The rest offset intentionally shows up here: resting gaps become +5 mm and a stack of N boxes
sits N-1 gaps higher. Everything else should match the rules-off build.

### `regress_mesh.c`

Mesh contact regression. The mesh code warns that a hull can tunnel if the time of impact lands
on a concave edge ("flat box sliding down a ramp to a flat bottom"), so that is checked directly
at two friction values, along with resting gap and jitter on a flat mesh, a vertical mesh wall at
5/20/50 m/s, sliding across internal edges at three yaw angles, and containment in a closed mesh
pit.

Every line here should be identical between the rules-on and rules-off builds.

### `lifecycle.c`

Exercises the build/teardown cycle the `ConveyorSeamGhost` sample performs when a control
changes: destroy every body, then destroy the meshes those bodies referenced, then rebuild.
Meshes are not cloned by `b3CreateMeshShape`, so the ordering matters. 45 cycles across every
belt and item combination, stepping in between so contacts, islands and the SAT cache are live
across each teardown. Run it against a build with validation on so misuse asserts.

### `rest_offset_latch.c`

Guards the grazing-flag latch. A contact that grazes once - which is what happens every time a
body crosses a seam - must not keep that classification after it settles into a flat face pair.
When it did, the rest offset stayed suppressed and the body rode *below* the deck, turning every
downstream seam into a step up. Four item shapes over six drop heights, 21 landing offsets each,
checking the settled gap stays positive and the item never stops.

### `tunneling.c`

Fast bodies dropped onto static ground: seven shapes (three boxes, two hulls, a sphere, a capsule)
against four grounds (a 1 m box hull, a 100 mm box hull, a 1 m closed box mesh, a 100 mm closed box
mesh), at seven impact speeds from 10 to 100 m/s, 24 orientations each. 4704 drops. Counts how many
finish below the ground.

This is the shape asymmetry that exposed both CCD bugs: hulls and boxes tunnelled while spheres and
capsules never did, and the mesh grounds never failed while the hull grounds did. Expect 0. Before
the fixes: 597 with the grazing drop applying to fast bodies, 20 with the drop gated but no TOI
fallback, 78 with the fallback but no gate.

The thin mesh ground is there to answer a specific question rather than to pad the matrix. The
grazing drop in `src/mesh_contact.c` is NOT gated on `isFast` the way the convex one is, so it can
still delete a fast body's speculative manifold - and a 1 m thick mesh is exactly the geometry that
would hide the consequence. It scores 0 in every variant including the unfixed baseline, which says
the per-triangle fallback in `b3MeshTimeOfImpactFcn` covers the mesh path on its own. That is a
measured answer, not an assumed one; keep the row so it stays measured.

### `toi_probe.c`

Calls `b3TimeOfImpact` directly, no world, to show the mechanism rather than the symptom. A box,
a sphere and a capsule sweep into a slab from a clean gap, from inside the penetration slop, and
while tumbling. Prints the TOI state, fraction and core distance, and whether
`b3ContinuousQueryCallback` would have used the result. The box rows that read `Overlapped
f=0.0000 DISCARDED` while the sphere rows read `Hit` are the whole bug in one line.

### `tunnel_trace.c`

Prints a step-by-step position and velocity trace for drops that still finish below a thin deck.
It separates real tunneling (one step crosses the slab with no contact) from a wrong-side pop (the
body is stopped inside the slab and then resolved out through the bottom face). Prints "no
failures found" when everything is healthy.

### `ledge_launch.c`

Repro for long items being launched (with a spin) as they ride off a triangle-mesh ledge. The
RollerLedge collision profile from the game (flat deck, three transition facets, a 31 deg ramp
that is a 7.6 cm slab with a sloped underside, an 8.9 cm drop face) is extruded into a mesh with
painted tangent velocities; a rod (capsule), plate and ingot (box hulls) and a sphere ride off it
over a sweep of 25 lateral offsets and yaws, with and without the game's belt velocity write, at
three `contactSpeed` caps and at 2 and 4 substeps. A control builds the same profile as one convex
slab hull per top segment.

Reports, per shape and configuration, how many of the 25 runs launched, the worst upward velocity
near the edge, and the most negative separation the solver was handed together with its push
direction and triangle. `-v` also traces the worst mesh run step by step.

What it shows: the moment the item's centre crosses the drop face's plane, that triangle is no
longer back-face culled and its face contact reports the item's rear as 0.25-0.35 m behind the
face's *infinite* plane, pushing +z at up to `contactSpeed`. For hulls SAT's correct hull-face
axis is rejected by the `pushingDown` guard in `b3CollideTriangleAndHull`; for capsules the
shallow branch takes the face path because the closest direction is within ~78 deg of the face
normal. Spheres work from closest features and never launch; neither does the slab control. The
item's rear is in front of the neighbouring ramp triangle across a convex edge - outside the solid
- which is the test a mesh-level fix can apply.

Expected before a fix: 25/25 launches for rod, plate and ingot on every mesh row (peak upward
velocity ~3-4.5 m/s at `contactSpeed` 10, ~2 at 3, none at 0.1 where the bogus contact still
exists but cannot push; ~7-8 m/s at 4 substeps), 0/25 for spheres and for the slab control.
Expected after: 0 everywhere.

## Performance

### `perf.c` and `perf-ab.ps1`

```
.\perf-ab.ps1                    # all four variants, alternating
.\perf-ab.ps1 -Reps 5 -Scene belt
.\perf-ab.ps1 -Workers 4 -NoBuild
```

A MineMogul-shaped workload, because these rules are not free by inspection: they act on bodies
flagged `b3_isFast`, and in a factory game that is the entire awake set. An IronIngot hull has
`innerRadius` 47.2 mm, so the continuous stage triggers above 23.6 mm of motion per step, which at
dt = 1/50 is **1.18 m/s** - and belts run 1.4 to 2.0 m/s. Every conveyor rider takes the fast path
on every step.

Two scenes: `belt` is belts and riders only, so a regression is not diluted; `mixed` adds a
triangle-mesh ground and sleeping hull piles, sized to the project's moderate profile capture
(7.8k bodies, ~1.1k awake, ~37k awake contacts).

`perf-ab.ps1` builds four engine variants - `base`, `contact` (grazing gate only), `shape` (TOI
fallback only), `fixed` (both) - then swaps `box3d.dll` under one `perf.exe`, so the benchmark
binary is identical for every variant and cannot itself be the difference.

Two traps this script exists to avoid, both of which produced confidently wrong numbers first:

- **`Copy-Item` preserves the source timestamp.** Restoring a file that way leaves it older than
  the object built from the other version, ninja decides nothing changed, and you benchmark two
  identical DLLs while believing they differ. Every source swap is followed by an explicit
  `LastWriteTime` stamp, and each variant gets its own build tree.
- **A hash proves nothing.** A PE file embeds a link timestamp, so identical sources hash
  differently while a build that did nothing hashes the *same* as the previous variant. Variants
  are fingerprinted by behaviour instead: each must report its expected `tunneling.c` count
  (base 597, contact 20, shape 78, fixed 0) or the script aborts before timing anything.

Also note the benchmark assigns recirculating riders a per-rider deterministic yaw rather than
drawing from a shared random stream. Two variants recirculate on slightly different steps, and a
shared stream would then hand them different values and drift the scenes apart - which shows up as
a performance difference that is really just a different scene.

## Note on the determinism unit test

Changing contact behaviour moves the golden sleep steps and hashes in
`test/test_determinism.c`. Those are per-precision and reproducible on a given machine, so
re-run `test.exe DeterminismTest` and paste the printed values back into the `#define` block for
both the float and double branches.
