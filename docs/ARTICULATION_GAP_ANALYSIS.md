# AVBD Articulation Support Analysis

> **Analysis Date**: February 5, 2026 (original); **Last Updated**: March 15, 2026  
> **Status**: Pure AVBD Articulation — **PhysX-side ACCEPTED (29/29 tests)**

## Executive Summary

After careful analysis of the PhysX Articulation API and comparison with TGS solver architecture, we have determined that **AVBD should NOT attempt to replace Featherstone for articulation internal constraints**. Instead, the correct approach is a **hybrid architecture** similar to TGS.

**Decision**: Removed the incorrect `AvbdArticulationAdapter` implementation (~740 lines deleted).

---

## 1. Why the Previous Approach Was Wrong

The deleted `AvbdArticulationAdapter` attempted to:
- Treat each Articulation Link as an independent AVBD rigid body
- Connect links with AVBD SphericalJoint constraints
- Implement PD drives and joint limits inside AVBD

### Problems with This Approach

| Issue | Impact |
|-------|--------|
| **Coordinate Space Mismatch** | Articulation uses reduced coordinates (joint angles), AVBD uses Cartesian coordinates |
| **Poor Convergence** | Chain structures need O(chain_length) iterations to propagate constraints; AVBD assumes independent constraints |
| **DOF Mismatch** | Articulation joints have 1-6 DOF; SphericalJoint is always 3 DOF |
| **Joint Space Control Lost** | PD drives should operate on `jointPosition`/`jointVelocity`, not Cartesian velocity |
| **Algorithm Incompatibility** | Inverse dynamics requires analytical solutions; AVBD is iterative |

---

## 2. Correct Architecture (Following TGS Pattern)

TGS solver handles Articulation correctly using this pattern:

```
┌─────────────────────────────────────────────────────────────────┐
│                        Solver Loop                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  for each iteration:                                            │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ 1. Articulation Internal Constraints (Featherstone)     │  │
│    │    articulation->solveInternalConstraints(...)          │  │
│    │    - Joint drives, joint limits, tendons, mimic joints  │  │
│    │    - 100% handled by FeatherstoneArticulation           │  │
│    └─────────────────────────────────────────────────────────┘  │
│                          ↓                                      │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ 2. External Constraints (TGS/AVBD Solver)               │  │
│    │    - Link ↔ RigidBody collisions                        │  │
│    │    - Link ↔ RigidBody joints                            │  │
│    │    - Uses SolverExtBody abstraction for Link access     │  │
│    │    - Calls articulation->getImpulseResponse()           │  │
│    └─────────────────────────────────────────────────────────┘  │
│                          ↓                                      │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ 3. RigidBody Constraints (TGS/AVBD Solver)              │  │
│    │    - Body ↔ Body collisions                             │  │
│    │    - Body ↔ Body joints                                 │  │
│    └─────────────────────────────────────────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key TGS Components for Articulation

| Component | Purpose |
|-----------|---------|
| `SolverExtBodyStep` | Unified abstraction for RigidBody OR Articulation Link |
| `getImpulseResponse()` | Computes velocity change from impulse; delegates to `articulation->getImpulseResponse()` for links |
| `solveInternalConstraints()` | Featherstone handles all internal articulation constraints |
| Alternating solve | Internal constraints first, then external constraints |

---

## 3. What AVBD Needs to Support Articulation

### 3.1 Required New Components (~400 lines)

```cpp
// Unified body abstraction (similar to TGS SolverExtBodyStep)
class AvbdSolverExtBody {
    union {
        const FeatherstoneArticulation* mArticulation;
        const AvbdSolverBody* mBody;
    };
    PxU32 mLinkIndex;
    
    // Impulse response (delegates to articulation for links)
    PxReal getImpulseResponse(const Cm::SpatialVector& impulse, 
                               Cm::SpatialVector& deltaV);
    
    // Velocity accessors
    PxVec3 getLinVel() const;
    PxVec3 getAngVel() const;
    void applyImpulse(const Cm::SpatialVector& impulse);
};
```

### 3.2 Solver Loop Modification

```cpp
void AvbdSolver::solveWithArticulations(...) {
    for (PxU32 iter = 0; iter < iterations; ++iter) {
        // Step 1: Featherstone solves internal constraints
        for (PxU32 i = 0; i < numArticulations; ++i) {
            articulations[i]->solveInternalConstraints(dt, invDt, ...);
        }
        
        // Step 2: AVBD solves external constraints (Link-Body, Body-Body)
        solveExternalConstraints(bodies, extBodies, constraints);
    }
}
```

### 3.3 Constraint Preparation Changes

- Support `Link ↔ RigidBody` collision constraints
- Support `Link ↔ RigidBody` joint constraints  
- Use `AvbdSolverExtBody` for unified body access

---

## 4. Implementation Effort Estimate

| Task | Lines | Complexity | Time |
|------|-------|------------|------|
| ~~Delete incorrect adapter~~ | ~~-740~~ | ~~Low~~ | ~~Done~~ |
| `AvbdSolverExtBody` class | +100 | Medium | 2-3 days |
| Constraint prep modification | +200 | High | 1 week |
| Solver loop integration | +100 | Medium | 2-3 days |
| Testing & debugging | - | High | 1 week |
| **Total** | **~400 net** | | **2-4 weeks** |

---

## 5. What Will Work After Implementation

| Feature | Status | Notes |
|---------|--------|-------|
| Articulation internal dynamics | ✅ Via Featherstone | Joint drives, limits, tendons, mimic joints |
| Articulation ↔ RigidBody collision | 🔧 Needs implementation | Uses `getImpulseResponse()` |
| Articulation ↔ RigidBody joints | 🔧 Needs implementation | External joint constraints |
| Link self-collision | 🔧 Needs implementation | Uses `getImpulseSelfResponse()` |
| Inverse dynamics API | ✅ Via Featherstone | `computeMassMatrix()`, `computeJointForce()`, etc. |

---

## 6. What Will NOT Work

| Feature | Reason |
|---------|--------|
| AVBD solving articulation internal constraints | Algorithm mismatch (position-based vs reduced coordinates) |
| Custom AVBD joint drives for articulation | Should use Featherstone's native implementation |
| Pure AVBD articulation (no Featherstone) | Fundamentally incompatible architectures |

---

## 7. Conclusion

The correct path forward is:

1. ✅ **Keep Featherstone** for all articulation internal constraints
2. ✅ **Deleted incorrect adapter** that tried to replace Featherstone
3. 🔧 **Implement hybrid architecture** following TGS pattern
4. 🔧 **Add `AvbdSolverExtBody`** for unified Link/Body access
5. 🔧 **Modify constraint prep** to support Link-Body interactions

This approach:
- Leverages Featherstone's O(n) optimal algorithm for articulations
- Uses AVBD's strengths for rigid body constraints
- Follows proven architecture from TGS solver
- Minimizes code changes while maximizing compatibility

---

## 8. Deleted Files

The following files were removed as they implemented an incorrect approach:

- `physx/source/lowleveldynamics/src/DyAvbdArticulationAdapter.cpp` (~450 lines)
- `physx/source/lowleveldynamics/src/DyAvbdArticulationAdapter.h` (~290 lines)

Updated:
- `physx/source/compiler/cmake/LowLevelDynamics.cmake` — Removed file references

---

## References

- `physx/source/lowleveldynamics/src/DyTGSDynamics.cpp` — TGS articulation handling
- `physx/source/lowleveldynamics/src/DyTGSContactPrep.cpp` — `SolverExtBodyStep` implementation
- `physx/source/lowleveldynamics/src/DyFeatherstoneArticulation.cpp` — `solveInternalConstraints()`
- `physx/include/PxArticulationReducedCoordinate.h` — Full PhysX Articulation API

---

# Current Project Status & PhysX Gap Analysis (Updated 2026-03-14)

> **Date**: 2026-03-14  
> **Standalone Tests**: 95/95 PASS (Pure AVBD AL articulation, Phase 1+2 complete)  
> **PhysX Side**: 3 bugs fixed, build successful

---

## 8.1 Updated Architecture Findings (2026-03-11)

The original hybrid decision from Sections 2-7 is still correct: AVBD should
not replace Featherstone for articulation internal constraints. However, the
latest AVBD-vs-TGS debugging found that the current hybrid implementation still
has a missing stage.

### Confirmed findings from the latest investigation

| Finding | Status | Consequence |
|---------|--------|-------------|
| Final AVBD articulation projection had a stale-pose overwrite bug | ✅ Fixed | Solved joint coordinates were being discarded before reaching link world poses |
| Low-level drive target synchronization was suspected broken | ❌ Ruled out | Drive targets are live and consumed correctly by internal articulation solve |
| Remaining articulation mismatch comes from missing loop-coupling stage | ✅ Confirmed | External constraints touching articulation child links are not coupled correctly |
| Final internal articulation solve inside the last projection pass can erase deep-artic external coupling gains | ✅ Confirmed | Child-link external loop corrections are materially undone late in the solve |
| IK-only final projection is valid only for solves that actually used deep-artic external coupling | ✅ Confirmed | Making it global regresses ordinary articulation scenes |

### What this means

The current implementation effectively has:

1. Body-centric AVBD primal solve
2. Reduced-coordinate articulation solve / projection
3. Dual update

That is not enough for external loop constraints acting on articulation child
links. Those constraints are neither:

- pure internal articulation constraints, nor
- ordinary independent rigid-body constraints.

They require an explicit **articulation-aware external coupling** stage.

### New long-term architecture decision

The project should evolve toward a four-stage hybrid pipeline:

1. **Rigid-body primal solve** for ordinary rigid bodies and articulation roots
2. **Articulation-aware external coupling** for child-link external contacts /
    D6 / loop rows assembled in generalized coordinates
3. **Internal articulation solve / manifold projection** for native reduced-
    coordinate articulation constraints
4. **Dual / acceleration updates**

### Important implementation lesson

Do not solve this by simply letting all articulation child links re-enter the
ordinary AVBD per-body primal loop. A direct experiment showed that this does
restore some coupling, but it also introduces strong drift and wrong-direction
behavior. The correct fix is a dedicated coupling layer, not a full rollback to
Cartesian child-link solving.

Another important lesson from the latest validation is that the final
articulation projection policy must also remain layered. When deep-artic
external coupling runs successfully, using an IK-only final projection preserves
the externally corrected child-link state. But using that same IK-only policy as
the global default is incorrect, because non-deep-artic articulation scenes rely
on the legacy internal-solve projection semantics for stable drive and lift
behavior.

### Refactor plan now in effect

- Use the standalone solver as the architecture prototype.
- Introduce a generalized-coordinate coupling layer in standalone first.
- Once validated, port the same layered design back into the PhysX AVBD path.

### Standalone refactor milestone (2026-03-11)

The first standalone refactor milestone is now in place:

| Milestone | Status | Notes |
|-----------|--------|-------|
| Q-space external coupling path for articulation child-link contacts | ✅ Implemented | Contact rows are projected into articulation generalized coordinates before final projection |
| Q-space external coupling path for articulation child-link D6 / loop rows | ✅ Implemented | D6 rows touching articulation child links are assembled into the same generalized system |
| Coupling-row assembly extracted behind a dedicated helper boundary | ✅ Implemented | Standalone now separates row construction from the main Deep BCD solve flow |
| Dedicated regression for child-link external loop coupling | ✅ Implemented | New standalone `test104_articulationChildLoopD6` pins the minimal static-world loop case |
| Standalone validation after refactor step | ✅ 104/104 PASS | Full standalone suite passes after the coupling-layer extraction |

This does not finish the whole architecture migration, but it establishes the
right boundary for the next step: porting the same articulation-aware external
coupling layer into the PhysX AVBD path without dragging the entire standalone
solver structure over wholesale.

### Latest PhysX-side validation update (2026-03-11)

The current PhysX AVBD status after the latest projection follow-up is:

| Validation Item | Result | Notes |
|-----------------|--------|-------|
| Fixed-base child-link D6 loop with deep-artic off | ⚠️ Poor | Baseline remains large-error (`finalTipError ~= 26.89`) |
| Fixed-base child-link D6 loop with deep-artic on + legacy final projection | ⚠️ Limited gain | In-loop deep-artic coupling helps, but most of the gain is lost by final internal projection |
| Fixed-base child-link D6 loop with deep-artic on + IK-only final projection | ✅ Major gain | Error improves to about `finalTipError = 0.178667`, `maxTipError = 0.204957` |
| Full `SnippetAvbdArticulation` suite with IK-only projection forced globally | ❌ Regressed | Suite drops from `25 PASSED, 4 FAILED` to `22 PASSED, 7 FAILED` |
| Full `SnippetAvbdArticulation` suite with scoped IK-only projection | ✅ Restored | Default suite returns to `25 PASSED, 4 FAILED` while preserving the child-loop gain |

This leads to a concrete short-term rule for the PhysX port:

1. Keep the stale-pose overwrite fix.
2. Keep the deep-artic in-loop external coupling path.
3. Use IK-only final articulation projection only when deep-artic external
    coupling actually ran successfully in the current solve.
4. Keep legacy internal-solve final projection for ordinary non-deep-artic
    articulation scenes.

In other words, the projection fix is real, but its scope is conditional rather
than global.

### Current remaining PhysX-side failures

After applying the scoped projection rule, the default full regression suite is
back to its previous baseline of `25 PASSED, 4 FAILED`. The remaining failures
are the pre-existing scissor-lift baseline issues rather than new regressions
introduced by the projection fix:

- Scissor lift platform travels through a large lift stroke
- Scissor lift cycles up and down without load
- No explosion (positions bounded) in 10s
- Articulation base remains stable

That means the projection-policy question is resolved for now. The next solver
work item is the residual scissor-lift stability / stroke gap, not another
global projection-policy change.

### Generalized-coordinate entry refactor status (2026-03-12)

The next refactor step has now started on the PhysX side. The articulation-aware
external coupling stage is no longer wired only as an ad-hoc deep-artic helper.
Instead, the solver now has an explicit **generalized-coordinate coupling
entry** with separate:

- policy
- request
- workspace
- per-solve state
- backend kind selection

At the moment, this new entry hosts two concrete backends:

| Backend | Status | Scope |
|---------|--------|-------|
| Fixed-base articulation generalized-coordinate coupling | ✅ Wired through generic entry | Covers the validated child-link loop path |
| Non-fixed-base generalized-coordinate coupling | ✅ Wired through generic entry | Admits the scissor-lift articulation path and runs per iteration |

The fixed-base path is also no longer inlined directly inside the stage entry.
It has been pulled behind a dedicated backend helper boundary, so the generic
stage now does three separate things more cleanly:

1. Gate whether generalized-coordinate coupling is even enabled for the solve.
2. Select which backend kind is allowed to run.
3. Dispatch to a backend-specific build/apply helper and record the per-solve
    result in generic state.

That refactor does not change the validated fixed-base behavior. It also proved
that backend #2 can be added as a local extension of the generalized-coordinate
dispatch path rather than another rewrite of the main solver loop.

This is an architectural milestone rather than a final behavior fix. It means:

1. The solver loop boundary is now generic enough to host more than one
    generalized-coordinate backend without another main-loop rewrite.
2. The current fixed-base deep-artic path remains behaviorally intact, but is
    now treated as backend #1 behind the generic entry.
3. Final projection policy can continue to depend on whether a generalized-
    coordinate backend actually applied corrections, rather than depending on a
    fixed deep-artic implementation detail.

### Latest non-fixed-base backend conclusion (2026-03-12)

The remaining scissor-lift work is no longer an admission problem. With isolated
`Test 16` runs and first-frame stage logs, the current generalized-coordinate
backend now reports for every first-frame iteration:

- `policyGc=1`
- `attempt=1`
- `eligible=1`
- `built=1`
- `applied=1`
- `backend=non-fixed-base-articulation`

So the current situation is now:

- the scissor-lift path is admitted into backend #2 and generalized-coordinate
  corrections are applied every iteration

The active issue has shifted to backend #2 stability tuning. Two concrete
findings are now validated:

1. Reusing the fixed-base projection policy for backend #2 was too aggressive.
    If non-fixed-base coupling is allowed to trigger IK-only final projection,
    the floating-base scissor-lift becomes much less stable. The current best
    behavior keeps internal projection solve enabled for backend #2.
2. Letting backend #2 drive floating-base root angular generalized DOFs directly
    is also too aggressive. The current best variant freezes only the floating-
    base root angular correction block while still allowing root translation and
    internal articulation DOFs to participate.

With that narrower backend #2 policy, isolated `Test 16` improved from the
earlier `5 PASSED, 4 FAILED` state to `7 PASSED, 2 FAILED`.

The remaining failures are still the loaded boundedness / base-stability pair:

- `FAIL: No explosion (positions bounded) in 10s`
- `FAIL: Articulation base remains stable`

The current loaded failure mode is no longer an immediate frame-11 blow-up.
Instead, the best current backend #2 variant keeps the articulation stable in
the unloaded phase and through meaningful loaded cyclic motion, but still shows
a later whole-base drift/explosion around frame 21 in isolated `Test 16`.

### Root warmstart source diagnosis update (2026-03-12)

The current non-fixed-base gap is no longer best explained as a missing drive-
target sync problem. The latest stage-level tracing shows that the first-frame
root mismatch is introduced earlier, inside the floating-root Stage 2 adaptive
warmstart path.

#### Confirmed source-side findings

| Finding | Status | Consequence |
|---------|--------|-------------|
| Floating articulation root does enter Stage 2 adaptive warmstart | ✅ Confirmed | The root is not bypassing the warmstart path |
| Frame-1 root warmstart uses `accelWeight = 0` | ✅ Confirmed | Warmstarted root `position` remains at `prevPosition` |
| Stage 1 prediction already moved `predictedPosition` / `inertialPosition` downward by `g * dt^2` | ✅ Confirmed | A root-only inertial mismatch already exists at `postInitPositions` |
| Directly snapping root inertial targets to the warmstarted pose | ❌ Rejected | This regresses stability and worsens isolated `Test 16` |

At frame 1, the validated baseline behavior is:

- root warmstart is applied with non-zero inverse mass
- `position` stays on the previous pose because `accelWeight = 0`
- `predictedPosition` and `inertialPosition` are already lower by one gravity
  prediction step

That produces the first root-only `position - inertialPosition` gap before the
generalized-coordinate backend gets a chance to correct anything.

#### Mitigation experiments and current best state

The safe direction is not an inertial-target snap. The best tested mitigation so
far is a narrow source-side floor on the floating root's Stage 2 gravity
warmstart weight.

| Experiment | Result | Notes |
|-----------|--------|-------|
| Root inertial target snap to warmstarted pose | ❌ Regressed | Isolated `Test 16` falls to `6 PASSED, 3 FAILED`; loaded failure moves earlier |
| `rootWarmstartAccelFloor = 0.25f` | ❌ Too aggressive | Root gap shrinks, but unloaded articulation stability regresses; result `6 PASSED, 3 FAILED` |
| `rootWarmstartAccelFloor = 0.10f` | ❌ Too aggressive | Regresses badly to `5 PASSED, 4 FAILED` |
| `rootWarmstartAccelFloor = 0.05f` | ✅ Best validated | Keeps isolated `Test 16` at `7 PASSED, 2 FAILED` and delays first loaded failure from frame 21 to frame 43 |

With `rootWarmstartAccelFloor = 0.05f`, the frame-1 root Y gap improves from
about `2.725005e-03` to `2.588749e-03` without reintroducing the unloaded
phase-1 explosion seen at larger floor values.

#### Current interpretation

This does not close the remaining scissor-lift loaded-stability gap, but it
changes the immediate diagnosis:

1. The remaining non-fixed-base issue is now strongly localized to floating-root
    warmstart / inertial-reference consistency rather than drive-target sync.
2. The admissible tuning window is narrow; larger root gravity injection quickly
    damages unloaded stability.
3. The current best live state for isolated `Test 16` is no longer the old
    frame-21 baseline. It is the `rootWarmstartAccelFloor = 0.05f` state with
    the same `7 PASSED, 2 FAILED` count but first loaded failure delayed to
    frame 43.

---

## 9. Current AVBD Articulation Implementation Status

### 9.1 Implemented Feature Checklist

| Feature | File | Status | Test Coverage |
|---------|------|--------|---------------|
| **Joint Types** | | | |
| Revolute (1-DOF) | `avbd_articulation.h` | ✅ Implemented | test74, test77, test79 |
| Spherical (3-DOF) | `avbd_articulation.h` | ✅ Implemented | test78 |
| Fixed (0-DOF) | `avbd_articulation.h` | ✅ Implemented | test75 |
| Prismatic (1-DOF) | `avbd_articulation.h` | ✅ Implemented | test89, test90 |
| **Core Features** | | | |
| Fixed Base (fixedBase) | `avbd_articulation.h` | ✅ Implemented | All artic tests |
| Floating Base (fixedBase=false) | `avbd_articulation.h` | ✅ Implemented | test96, test97 |
| Joint Limits | `avbd_articulation.h:applyJointCorrections` | ✅ Implemented | test77, test90 |
| Velocity Drive | `avbd_articulation.h:applyJointCorrections` | ✅ Implemented | test79 |
| Position Drive (PD) | `avbd_articulation.h:applyJointCorrections` | ✅ Implemented | test91, test92 |
| Joint Friction | `avbd_articulation.h:applyJointCorrections` | ✅ Implemented | test93, test94 |
| Armature (Rotor Inertia) | `avbd_articulation.h:computeGeneralizedMassMatrix` | ✅ Implemented | test95 |
| Forward Kinematics (FK) | `avbd_articulation.h:forwardKinematics` | ✅ Implemented | All artic tests |
| Inverse Kinematics (extractJointAngles) | `avbd_articulation.h:extractJointAngles` | ✅ Implemented | All artic tests |
| **Computational Dynamics API** | | | |
| RNEA (Recursive Newton-Euler) | `avbd_articulation.h:rneaCore` | ✅ Implemented | test81, test85 |
| CRBA (Composite Rigid Body Mass Matrix) | `avbd_articulation.h:computeMassMatrix` | ✅ Implemented | test82 |
| Inverse Dynamics (ID) | `avbd_articulation.h:inverseDynamics` | ✅ Implemented | test83 |
| Forward Dynamics (FD) | `avbd_articulation.h:forwardDynamics` | ✅ Implemented | test83, test84 |
| Bias Force (Coriolis + Gravity) | `avbd_articulation.h:computeBiasForce` | ✅ Implemented | test81 |
| Gravity Compensation Force | `avbd_articulation.h:computeGravityForce` | ✅ Implemented | test81 |
| Joint Reaction Forces | `avbd_articulation.h:computeJointReactionForces` | ✅ Implemented | test85 |
| Dense Jacobian | `avbd_articulation.h:computeDenseJacobians` | ✅ Implemented | test86 |
| Generalized Mass Matrix (J^T M J) | `avbd_articulation.h:computeGeneralizedMassMatrix` | ✅ Implemented | test86 |
| **ADMM Hybrid Solve** | | | |
| Featherstone Projection (inside AVBD loop) | `avbd_solver.cpp` | ✅ Implemented | test74-80, test86 |
| Deep BCD (J^T M J q-space solve) | `avbd_solver.cpp` | ✅ Implemented | test86 |
| External Contact Handling (Artic link ↔ Ground) | `avbd_solver.cpp` | ✅ Implemented | test76 |
| **Convergence Acceleration** | | | |
| Safeguarded Anderson Acceleration (m=3) | `avbd_solver.cpp` | ✅ Implemented, ON by default | test87 |
| Chebyshev Semi-Iterative | `avbd_solver.cpp` | ✅ Implemented, ON by default | test88 |

### 9.2 Test Coverage (test74–test88)

| Test | Name | Validates |
|------|------|-----------|
| test74 | `articulationPendulum` | Single pendulum gravity swing + FK correctness |
| test75 | `articulationChain5` | 5-link chain constraint propagation |
| test76 | `articulationOnGround` | Articulation + ground collision hybrid solve |
| test77 | `articulationWithLimits` | Joint limits (rotation angle clamping) |
| test78 | `articulationSpherical` | Spherical joint 3-DOF correctness |
| test79 | `articulationRC` | Reduced-coordinate drive (mirrors PhysX SnippetArticulationRC) |
| test80 | `articulationConstraintAccuracy` | Constraint accuracy regression |
| test81 | `rnea_staticGravity` | RNEA static gravity compensation verification |
| test82 | `massMatrix_symmetry` | Mass matrix symmetric positive definite check |
| test83 | `forwardInverse_roundtrip` | FD→ID round-trip consistency |
| test84 | `pendulumDynamics` | Single pendulum dynamics integration |
| test85 | `jointReactionForces` | Joint reaction force correctness |
| test86 | `articulationHybridStability` | Deep BCD hybrid solve stability |
| test87 | `andersonAcceleration` | AA convergence speed verification |
| test88 | `chebyshevDual` | Chebyshev dual acceleration verification |
| test89 | `articulationPrismatic` | Prismatic joint slides under gravity |
| test90 | `articulationPrismaticLimits` | Prismatic joint with displacement limits |
| test91 | `positionDrive` | PD position drive tracking |
| test92 | `positionDriveGravComp` | PD drive gravity compensation comparison |
| test93 | `jointFrictionDamping` | Joint friction dampens oscillation |
| test94 | `viscousFriction` | Viscous friction decays joint velocity |
| test95 | `armature` | Armature/rotor inertia correctness + mass matrix |
| test96 | `floatingBase` | Floating base free-fall under gravity |
| test97 | `floatingBaseDynamics` | Floating base CoM conservation in zero gravity |

---

## 10. Detailed Gap vs PhysX Articulation

### 10.1 Joint Type Gaps

| PhysX Joint Type | DOF | AVBD Status | Priority | Difficulty |
|-----------------|-----|-------------|----------|------------|
| `eFIX` | 0 | ✅ Available | — | — |
| `eREVOLUTE` | 1 | ✅ Available | — | — |
| `eREVOLUTE_UNWRAPPED` | 1 | ⚠️ Partial (no angle wrapping distinction) | Low | Low |
| `eSPHERICAL` | 2-3 | ✅ Available | — | — |
| `ePRISMATIC` | 1 | ✅ Available | — | — |
| `eUNDEFINED` | N/A | Not needed | — | — |

**Prismatic Joint Implementation Guide**:
- Add `eARTIC_PRISMATIC` to `ArticulationJointType`
- `q[0]` represents linear displacement along axis (not angle)
- `extractJointAngles()` → compute projected distance along axis
- `forwardKinematics()` → child position = parent anchor + axis × q[0] (no rotation change)
- Deep BCD Jacobian: linear velocity column = axis (angular velocity column = 0)

### 10.2 Drive System Gaps

| PhysX Drive Feature | AVBD Status | Priority | Notes |
|---------------------|-------------|----------|-------|
| Velocity Drive | ✅ Available | — | `targetVelocity + driveDamping` |
| **Position Drive** | ✅ Available | — | PD controller: `stiffness*(target-q) + damping*(targetVel-qd)` |
| Acceleration Drive (`eACCELERATION`) | ❌ Missing | Medium | Inertia-invariant mode, divides by effective mass |
| Force/Torque Limit (`maxForce`) | ✅ Available (via driveMaxForce) | — | Clamp on drive output |
| Performance Envelope (`envelope`) | ❌ Missing | Low | Motor characteristics: maxEffort, speedEffortGradient |

**Position Drive Implementation Guide**:
```cpp
// Add to ArticulationJointData:
float targetPosition[3];    // Target position/angle
float driveStiffness;       // PD controller Kp
float driveMaxForce;        // Torque/force limit

// Add to applyJointCorrections():
float posError = targetPosition[d] - q[d];
float velError = targetVelocity[d] - qd[d];
float correction = (driveStiffness * posError + driveDamping * velError) * dt;
if (driveMaxForce > 0) correction = clamp(correction, -driveMaxForce*dt, driveMaxForce*dt);
joint.q[d] += correction;
```

### 10.3 Joint Physics Gaps

| PhysX Feature | AVBD Status | Priority | Notes |
|---------------|-------------|----------|-------|
| **Joint Friction** (Coulomb + Viscous) | ✅ Available | — | `staticFriction`, `dynamicFriction`, `viscousFriction` |
| **Armature (Rotor Inertia)** | ✅ Available | — | Diagonal extra inertia added to generalized mass matrix |
| Joint Velocity Limit | ❌ Missing | Medium | `maxJointVelocity` per-axis |
| CFM Scale | ❌ Missing | Low | Per-link stabilization coefficient |

**Joint Friction Implementation Guide**:
```cpp
// Add to ArticulationJointData:
float staticFriction;       // Static friction torque
float dynamicFriction;      // Dynamic friction torque
float viscousFriction;      // Viscous friction coefficient

// Add to applyJointCorrections():
float frictionTorque = viscousFriction * qd[d];
if (fabsf(qd[d]) > 1e-6f)
    frictionTorque += dynamicFriction * (qd[d] > 0 ? 1.0f : -1.0f);
else
    frictionTorque = clamp(frictionTorque, -staticFriction, staticFriction);
joint.q[d] -= frictionTorque * dt / effectiveInertia;
```

**Armature Implementation Guide**:
```cpp
// Add to ArticulationJointData:
float armature[3];          // Per-DOF extra inertia

// Add to computeGeneralizedMassMatrix():
for (int i = 0; i < totalDof; i++)
    M[i][i] += armature_for_dof[i];

// Similarly add diagonal terms in Deep BCD J^T M J
```

### 10.4 Base Configuration Gaps

| PhysX Feature | AVBD Status | Priority | Notes |
|---------------|-------------|----------|-------|
| Fixed Base (`eFIX_BASE`) | ✅ Available | — | `fixedBase = true` |
| **Floating Base** | ✅ Available | — | Root link with 6 DOF free motion, `fixedBase = false` |

**Floating Base Implementation Guide**:
- When `fixedBase = false`, root link gains 6 extra DOFs (3 translation + 3 rotation)
- `getGeneralizedDof()` already returns `(fixedBase ? 0 : 6) + getTotalDof()` correctly
- `computeDenseJacobians()` already handles first 6 columns for floating base (root_v, root_w)
- Remaining work:
  - `extractJointAngles()`: extract root position and rotation as q[0..5]
  - `forwardKinematics()`: reconstruct root link pose from root q[0..5]
  - `rneaCore()`: complete floating root branch (skeleton exists, needs finishing)
  - Deep BCD: extend Jacobian dimension from `getTotalDof()` to `getGeneralizedDof()`

### 10.5 Advanced Feature Gaps

| PhysX Feature | AVBD Status | Priority | Notes |
|---------------|-------------|----------|-------|
| **Spatial Tendons** | ❌ Missing | Low | Spring tendons connecting different links |
| **Fixed Tendons** | ❌ Missing | Low | Couple multiple joint DOFs via tendon |
| **Mimic Joints** | ✅ Available (standalone AL) | — | `q_A + ratio * q_B + offset = 0` as AL bilateral constraint |
| Cache API (state read/write) | ❌ Missing | Medium | `createCache()`, `applyCache()`, `copyInternalStateToCache()` |
| Sleep/Wake Management | ❌ Missing | Low | Energy threshold sleeping |
| Self-Collision Control | ❌ Missing | Low | `eDISABLE_SELF_COLLISION` flag |
| COM Computation (`computeArticulationCOM`) | ❌ Missing | Low | Whole-body center of mass |
| Centroidal Momentum Matrix | ❌ Missing | Low | For balance control |
| Coordinate Conversion (`packJointData/unpackJointData`) | ❌ Missing | Low | Maximal ↔ reduced coordinates |

**Mimic Joint Implementation Guide**:
```cpp
struct MimicJoint {
    uint32_t linkA, axisA;  // Leader joint
    uint32_t linkB, axisB;  // Follower joint
    float gearRatio;        // q_A + gearRatio * q_B + offset = 0
    float offset;
};

// Append after applyJointCorrections():
for (auto& mimic : mimicJoints) {
    float error = q_A + mimic.gearRatio * q_B + mimic.offset;
    // Distribute correction by inertia ratio
    q_A -= error * (I_B / (I_A + I_B));
    q_B -= error * mimic.gearRatio * (I_A / (I_A + I_B));
}
```

---

## 11. Priority Ranking & Implementation Recommendations

### 11.1 High Priority (All Implemented)

| # | Feature | Status | Tests |
|---|---------|--------|-------|
| 1 | Prismatic Joint | ✅ Implemented | test89, test90 |
| 2 | Position Drive (PD) | ✅ Implemented | test91, test92 |
| 3 | Joint Friction | ✅ Implemented | test93, test94 |
| 4 | Armature / Rotor Inertia | ✅ Implemented | test95 |
| 5 | Floating Base | ✅ Implemented | test96, test97 |

### 11.2 Medium Priority (Feature Completeness)

| # | Feature | Difficulty | Est. LOC | Status |
|---|---------|-----------|----------|--------|
| 6 | Mimic Joints | Medium | +60 | ✅ Implemented (standalone AL) |
| 7 | Joint Velocity Limit | Low | +10 | ❌ Missing |
| 8 | Cache API | Medium | +100 | ❌ Missing |
| 9 | Acceleration Drive | Low | +20 | ❌ Missing |

### 11.3 Low Priority (Full Parity)

| # | Feature | Difficulty | Est. LOC |
|---|---------|-----------|----------|
| 10 | Spatial/Fixed Tendons | High | +200 |
| 11 | Sleep/Wake | Medium | +80 |
| 12 | Self-Collision Control | Medium | +40 |
| 13 | COM / Momentum Matrix | Low | +50 |

### 11.4 Overall Coverage Estimate

```
Current █████████████████░░░ ~85-88%  (Joint types 4/5, Drives 2/3, Dynamics API 7/10, Friction+Armature+FloatBase+Mimic+ID+IK)
+Medium ██████████████████░░ ~92%     (+VelLimit, Cache, AccelDrive)
+Low    ████████████████████ ~98%     (+Tendons, Sleep, SelfCollision, COM)
```

> **Updated 2026-03-14**: Mimic joints moved from Medium→Done. ID extraction
> and end-effector IK added via pure AVBD AL constraints in standalone.

---

## 12. PhysX-Side AVBD Integration Status

### 12.1 Fixed Bugs

| Bug | File | Fix |
|-----|------|-----|
| BUG#1 | `DyAvbdDynamics.cpp` | `solverBody.initialize()` replaces partial field init |
| BUG#2 | `DyAvbdSolver.cpp` | Call `saveVelocityTGS` before each Featherstone solve |
| BUG#3 | `DyAvbdSolver.cpp` | Added final Featherstone projection pass after iteration loop |

### 12.2 Pending Port to PhysX

| Feature | Current Status | Priority |
|---------|---------------|----------|
| Deep BCD (J^T M J) | Standalone only, not in PhysX | High |
| Safeguarded AA | Standalone only, not in PhysX | Medium |
| Chebyshev Dual | Standalone only, not in PhysX | Medium |
| Jacobian Cache Optimization | Standalone only, not in PhysX | Medium |

---

> **For AI Agents**: This section is the latest status snapshot of AVBD Articulation.  
> Before implementing new features, check the status tables above for current progress.  
> When adding a new joint type or feature, update both the §9.1 status table and the §10 gap tables.  
> New test numbers start from test98 (test74-97 are occupied).  
> The standalone AL-based articulation (§13.7) uses the same test74-97 range but tests
> different content than the Featherstone hybrid tests listed in §9.2.  
> **Note**: Test numbers may conflict with `OGC_AVBD_SOFTBODY_ROADMAP.md` —  
> if the soft-body roadmap is implemented first, articulation tests should start after the soft-body range.

---

## 13. Featherstone Coupling Conclusion & MuJoCo Migration Plan (2026-03-13)

### 13.1 Featherstone Coupling — Final Assessment

After extensive iteration on the Featherstone hybrid architecture, the following
represents the **validated stability ceiling** for the AVBD ↔ Featherstone
coupling approach:

| Metric | Value |
|--------|-------|
| Test Suite | **28 PASSED, 1 FAILED** (out of 29) |
| Scissor Lift Stroke | 1.2523 |
| Explosion Frame | 446 (t=7.43s) |
| Base Stability | PASS |
| TGS Reference (similar scene) | ~1800 frames (~30s) |

#### Key fixes that achieved this ceiling

| Fix | Effect |
|-----|--------|
| C0 depth-adaptive clamping (both solve paths) | Deep penetrations get immediate full correction instead of 95% alpha blending |
| Post-FK single-pass contact repair | Push non-artic bodies out of post-FK artic-link overlap |
| D6 α=0.2 simple bilateral correction | Modest D6 loop closure |
| Phase 3 Export: full FK + prevPosition velocity fix | Correct velocity derivation after FK export |

#### Approaches tested and rejected

| Approach | Result | Reason |
|----------|--------|--------|
| Post-FK multi-pass repair (3 passes, full) | 27/2, frame 43 | Over-correction cascades through coupled contacts |
| Post-FK multi-pass repair (3 passes, ω=0.6) | 27/2, frame 116 | Still over-corrects, worse than single-pass |
| Velocity cap (15 m/s) | 28/1, frame 161 | Trapped boxes in penetration, energy feedback to links |
| Post-FK velocity injection (pos only, no prevPos) | 27/2, frame 454 | Base drift from reaction forces |

#### Root cause of the stability ceiling

The fundamental limitation is **architectural, not numerical**:

1. **Semantic mismatch**: AVBD operates at position level; Featherstone operates
   at velocity/acceleration level. The conversion between them loses information
   at every frame boundary.
2. **Sequential coupling**: AVBD solves contacts → Featherstone projects to
   joint manifold → positions change → new contact violations appear. This
   alternating solve cannot converge to a simultaneous solution.
3. **Closed kinematic loops**: The scissor lift has D6 loop closures +
   ground contacts + box loads forming a closed kinematic chain. Featherstone's
   tree-based recursion cannot handle closed loops natively; the D6 bilateral
   correction is a workaround with limited authority.
4. **TGS has the same problem**: Even TGS with full joint-space contact
   projection (`getImpulseResponse`) explodes at ~30s on scissor + chain hammer.
   This confirms the problem is intrinsic to Featherstone coupling, not specific
   to AVBD.

### 13.2 Strategic Decision: MuJoCo as Articulation Backend

**Decision**: Freeze the Featherstone coupling path at the current 28/1 baseline.
Future articulation development will use **MuJoCo as the articulation backend**.

#### Rationale

1. **Newton (Disney Research + Google DeepMind + NVIDIA) already validated this
   choice**: MuJoCo is Newton's "primary backend" for articulation; Featherstone
   is downgraded to a "kinematic integrator" for one-way coupling with VBD.
2. **MuJoCo has a unified constraint solver**: Contacts, joint limits, equality
   constraints, and closed loops are all handled in one KKT system — no
   alternating-solve instability.
3. **Diminishing returns**: Every approach beyond the C0 fix + single-pass repair
   either regressed or showed no improvement. The marginal cost of further
   Featherstone tuning far exceeds the expected gain.
4. **AVBD's core value points forward, not sideways**: Block descent + Augmented
   Lagrangian delivers unconditionally stable position-level constraint solving.
   Rather than outsourcing articulation to another solver (MuJoCo, Featherstone),
   the maximum-value path is to build native AVBD articulation that exploits
   this stability — making AVBD the first solver to unify FK/IK/ID/FD in a
   single position-level energy minimization framework.

#### Architecture: Pure AVBD Articulation (target)

```
┌─────────────────────────────────────────────────────────────────┐
│                     Per-Frame Pipeline                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ AVBD Block Descent (unified energy minimization)          │  │
│  │                                                           │  │
│  │   Articulation constraints (joint limits, drives, etc.)   │  │
│  │      → joint constraint rows in AL, same as contacts      │  │
│  │      → λ* at convergence = inverse dynamics output (free) │  │
│  │                                                           │  │
│  │   Contact constraints (rigid-rigid, artic-rigid, artic-   │  │
│  │      artic self-collision) — all in one energy function    │  │
│  │                                                           │  │
│  │   D6 / loop closure — bilateral AL constraints, no        │  │
│  │      special closed-loop handling needed                   │  │
│  │                                                           │  │
│  │   IK —末端约束 = just another constraint row              │  │
│  │   ID — λ* after convergence = joint forces (zero cost)    │  │
│  │                                                           │  │
│  │   Soft body / cloth / particles (future)                  │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
│  No Featherstone. No MuJoCo. No solver coupling boundary.       │
│  One energy function. One solve. All constraints unified.       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 13.3 Why Pure AVBD Articulation > MuJoCo Integration

The initial plan (§13.2 original) was to use MuJoCo as an articulation backend.
After deeper analysis of AVBD's mathematical properties, the pure AVBD path is
strictly more valuable:

| Dimension | MuJoCo Integration | Pure AVBD Articulation |
|-----------|--------------------|------------------------|
| Coupling boundary | Still exists (AVBD ↔ MuJoCo state exchange) | **Eliminated** (one solver) |
| Closed-loop ID | MuJoCo RNEA = open-chain only | AVBD λ* = **arbitrary topology** |
| IK | Separate algorithm needed | **Solver-is-IK** (position-level) |
| Stability guarantee | MuJoCo: velocity-level, conditionally stable | AVBD: **position-level, unconditionally stable** |
| Contact + joint coupling | Two solvers, sequential | **One energy function, simultaneous** |
| Dependency | External C library (licensing, build complexity) | **Zero dependency** |
| Novelty / research value | Integration engineering | **Novel contribution to physics simulation** |

The key mathematical insight: AVBD's AL framework already computes everything
needed for articulation. The constraint force $\lambda^*$ at convergence is
precisely the inverse dynamics output, and the position-level solve is precisely
IK. These are not new features to add — they are **inherent properties of the
solver that need to be exposed through the right articulation primitive**.

### 13.4 Implementation Roadmap (branch: `feature/avbd-articulation`)

#### Phase 1: Articulation as AL Constraints (standalone prototype) ✅ COMPLETE

Core idea: model each joint as a set of bilateral AL constraint rows in the
existing AVBD block descent, operating in **maximal (Cartesian) coordinates**.
No Featherstone, no generalized coordinates.

- [x] Define `ArticulationJoint` / `Articulation` primitives with constraint
      functions for: REVOLUTE (5 bilateral + 1 free), PRISMATIC (5 bilateral +
      1 free along axis), SPHERICAL (3 bilateral + 3 free), FIXED (6 bilateral)
- [x] Integrate joint constraint rows into the existing block descent Hessian
      (6×6 per body, same as contact/D6 rows) — `addArticulationContribution()`
- [x] Implement joint limits as inequality AL constraints (same mechanism as
      contact non-penetration) — revolute + prismatic limits with sign-correct Jacobians
- [x] Implement PD drives as soft AL constraints (target position/velocity with
      stiffness/damping → penalty term in energy) — revolute (PD controller) + prismatic (direct AL)
- [x] Implement joint friction as viscous damping AL constraints
- [x] Validate in standalone: single pendulum, 5-link chain, chain on ground,
      branching tree, mixed joint types, floating base
- [x] All 88 Phase 1 tests pass (test74–test90)

**Bug fixes during Phase 1**:
- Prismatic limit Jacobian sign: `limitSign = -sign` because `disp = (child-parent)·axis`
  has opposite convention to `C_lin = parent - child`
- Prismatic drive: rewrote from PD-controller intermediary to direct AL soft constraint
  (`C = disp - target`, with dedicated `lambdaDrive` dual variable)

#### Phase 2: ID / IK Extraction ✅ COMPLETE

- [x] Extract converged λ* for joint constraints → joint torques (ID)
      — `getJointForce()`, `getJointTorque()`, `getJointAxisForce()` API
- [x] Add end-effector position constraint primitive → solver-is-IK
      — `EndEffectorTarget` struct, `addIKTarget()` / `setIKTarget()` API,
        primal (`addIKTargetContribution`) + dual (`updateIKTargetDual`) updates
- [x] Add mimic joint constraint → linear DOF coupling
      — `MimicJointDef` struct, `addMimicJoint()` API,
        primal (`addMimicJointContribution`) + dual (`updateMimicDual`) updates
- [x] Validate: test91 (ID extraction), test92 (IK), test97 (mimic)
- [x] Additional stress tests: test93 (20-link chain), test94 (prismatic drive),
      test95 (multi-articulation), test96 (floating base momentum)
- [x] All 95 tests pass (test74–test97)

#### Phase 3: Convergence & Performance ✅ **COMPLETE**

- [x] Benchmark iteration count for articulation convergence (long chains)
      vs Featherstone O(n) single-pass
- [x] Apply existing convergence acceleration: Safeguarded Anderson
      Acceleration + Chebyshev semi-iterative
- [x] Evaluate block structure: tree-structured sweep order in block
      descent (symmetric Gauss-Seidel on articulation topology)
- [ ] Evaluate reduced-coordinate block descent as optional acceleration
      (generalized coordinates within AVBD energy, not as external coupling)

**Phase 3 Results** (20-link horizontal chain stress test, first-frame cold start):

| Method | 5 iters | 10 iters | 20 iters | Description |
|--------|---------|----------|----------|-------------|
| Baseline BCD | 0.146 | 0.099 | 0.066 | Max positional constraint violation (meters) |
| Tree-sweep | 0.874× | 0.821× | 0.833× | Symmetric GS on chain topology (~13–18% gain) |
| Anderson Accel (m=3) | 0.904× | **0.541×** | **0.529×** | Position-only AA; ~46–47% gain at 10+ iters |
| Chebyshev (ρ=0.92) | 0.815× | 0.755× | 0.712× | Position over-relaxation; ~19–29% gain |

Key findings:
1. **Warm-start scenario** (vertical chain, near-equilibrium): Converges to < 1e-3 in 1 iteration for all chain lengths (N=5–50). AVBD's per-body 6×6 block solve is extremely effective.
2. **Cold-start scenario** (horizontal chain, ~0.7m violation/joint): Needs 46+ iterations for N=5, fails to reach 1e-3 within 50 iters for N≥10. This is the realistic stress test.
3. **AA wins at higher iteration counts**: Position-only AA with constraint-violation safeguard gives ~47% reduction. Quaternion AA is ill-conditioned (rejected).
4. **Chebyshev is most robust**: Consistent 15–29% improvement with no instability risk.
5. **Tree-sweep is modest**: ~13–18% improvement (default body order already happens to be root→leaf for chains created sequentially).

All 99 tests pass (test74–test101).

#### Phase 4: Scissor Lift Validation

- [ ] Port standalone prototype to PhysX AVBD path
- [ ] Run Test 16 scissor lift with pure AVBD articulation (no Featherstone)
- [ ] Compare against Featherstone ceiling (28/1, frame 446) and TGS (~30s)
- [ ] **Target: exceed TGS stability** (closed loops + contacts are native)

#### Phase 5: Advanced Features

- [x] Joint friction (viscous damping) as AL constraints — implemented in Phase 1
- [ ] Armature / rotor inertia as diagonal mass augmentation
- [x] Mimic joints as linear equality AL constraints — implemented in Phase 2
- [ ] Tendon constraints as multi-body AL constraints
- [x] Floating base (free root = unconstrained DOFs, trivially supported via `fixedBase = false`)

### 13.5 Featherstone Path — Archived Configuration

The following configuration achieves the **28/1 baseline** and should be
preserved as the archived Featherstone reference on `feature/newton-mode-rework`:

| Parameter | Value |
|-----------|-------|
| avbdAlpha | 0.95 |
| avbdBeta | 1000 |
| avbdGamma | 0.99 |
| avbdPenaltyMin | 1000 |
| avbdPenaltyMax | 1e9 |
| baumgarte | 0.3 |
| omega (SOR) | 1.3 |
| innerIterations | 4 |
| outerIterations | 1 |
| d6Alpha | 0.2 |
| C0 depth-adaptive clamp | threshold=5mm, maxDepth=50mm |
| Post-FK repair | single-pass, full correction, pos+prevPos |
| Branch | `feature/newton-mode-rework` |

### 13.6 Theoretical Advantage of AVBD-Native ID over RNEA

Traditional RNEA computes $\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})$
via a single $O(n)$ recursive sweep. This is optimal for open kinematic chains.

AVBD-native ID extracts $\boldsymbol{\tau}$ from the AL dual variable $\lambda^*$
at convergence:

$$\boldsymbol{\tau}_j = J_j^T \lambda^*_j$$

where $J_j$ is the constraint Jacobian for joint $j$ and $\lambda^*_j$ is the
converged AL multiplier. This is $O(k \cdot n)$ where $k$ is iteration count,
but has two unique advantages:

1. **Arbitrary topology**: RNEA requires a tree. AVBD-ID works on closed loops,
   contact-loaded chains, and any constraint graph. For a scissor lift under
   load, RNEA cannot compute joint torques at all; AVBD-ID can.

2. **Zero marginal cost in simulation**: During simulation, $\lambda^*$ is
   already computed as part of the constraint solve. Reading it out as ID is
   free. RNEA requires a separate $O(n)$ call.

For real-time 1kHz control loops outside of simulation (where you need ID on
demand), RNEA remains faster for open chains. The standalone AVBD solver already
has a complete RNEA implementation (§9.1) that can serve this use case.

### 13.7 Pure AVBD Articulation — Standalone Implementation Status (2026-03-14)

The pure AVBD articulation path described in §13.3–§13.4 is now **implemented
and validated** in the standalone solver (`avbd_standalone/`). This section
records the current state.

#### Standalone Test Suite: **99/99 PASS**

All tests (test1–test101, with test48/test50 skipped) pass on MSVC 2026 x64.

#### Phase Status

| Phase | Status | Notes |
|-------|--------|-------|
| Phase 1: AL Constraints | ✅ **COMPLETE** | REVOLUTE, PRISMATIC, SPHERICAL, FIXED joint types; limits; PD drives; friction |
| Phase 2: ID / IK / Mimic | ✅ **COMPLETE** | ID extraction via λ*; end-effector IK target; mimic joint coupling |
| Phase 3: Convergence | ✅ **COMPLETE** | Tree-sweep (18%), AA (47%), Chebyshev (29%); warm-start converges in 1 iter |
| Phase 4: Scissor Lift | ❌ Not started | Port to PhysX, validate closed-loop stability |
| Phase 5: Advanced | 🔧 Partial | Joint friction ✅, mimic ✅, floating base ✅; tendon ❌, armature ❌ |

#### Standalone AL-Based Articulation Tests (test74–test97)

| Test | Name | Validates |
|------|------|-----------|
| test74 | `articulationPendulum` | Single revolute pendulum: gravity swing + constraint hold |
| test75 | `articulationChain5` | 5-link revolute chain stability |
| test76 | `articulationOnGround` | Articulation + ground contact hybrid |
| test77 | `articulationWithLimits` | Revolute joint limits (angle clamping) |
| test78 | `articulationSpherical` | Spherical (ball-socket) joint |
| test79 | `articulationFixed` | Fixed (welded) joint |
| test80 | `articulationPrismatic` | Prismatic (slider) joint |
| test81 | `articulationPrismaticLimits` | Prismatic joint with displacement limits |
| test82 | `articulationPDDrive` | PD position drive tracking (revolute) |
| test83 | `articulationJointFriction` | Viscous joint friction damps oscillation |
| test84 | `articulationConstraintAccuracy` | Positional constraint error measurement |
| test85 | `articulationMixedJoints` | Mixed joint types in one articulation |
| test86 | `articulationFloatingBase` | Floating base free-fall |
| test87 | `articulationBranching` | Branching (tree) topology |
| test88 | `articulationVelocityDrive` | Velocity drive produces rotation |
| test89 | `articulationMassRatio` | Extreme mass ratio (1000:1) stability |
| test90 | `articulationDriveGravComp` | PD drive gravity compensation |
| test91 | `articulationIDExtraction` | ID extraction: λ* encodes joint torques |
| test92 | `articulationEndEffectorIK` | End-effector IK: 2-link chain tip converges to target |
| test93 | `articulationLongChain` | 20-link deep chain stability stress test |
| test94 | `articulationPrismaticDriveTracking` | Prismatic PD drive tracks target displacement |
| test95 | `articulationMultiArticulation` | Two independent articulations coexist |
| test96 | `articulationFloatingBaseMomentum` | Floating base momentum conservation |
| test97 | `articulationMimicJoint` | Mimic joint: coupled revolutes with gear ratio |

#### Phase 3: Convergence & Performance Tests (test98–test101)

| Test | Name | Validates |
|------|------|-----------|
| test98 | `convergenceBenchmark` | Convergence curves for N=5–50 chains (warm + cold scenarios) |
| test99 | `treeSweepConvergence` | Symmetric GS tree-sweep ordering vs default linear sweep |
| test100 | `andersonAcceleration` | Position-only AA(m=3) with constraint-violation safeguard |
| test101 | `chebyshevSemiIterative` | Chebyshev position over-relaxation (ρ=0.92) |

#### Key Implementation Files

| File | Content |
|------|---------|
| `avbd_articulation.h` | All articulation structs, primal/dual inline functions |
| `avbd_solver.cpp` | Solver loop integration (primal + dual for joints, mimic, IK) |
| `avbd_solver.h` | `Solver` struct with `std::vector<Articulation>` |
| `avbd_tests_articulation.cpp` | Test cases test74–test97 |

#### Key Bug Fixes

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Prismatic limit explosion (test81) | Jacobian sign wrong: `disp = (child-parent)·axis` opposite to `C_lin = parent-child` | `limitSign = -sign` |
| Prismatic drive explosion (test94) | PD-controller intermediary produces wrong force direction for prismatic | Rewrote as direct AL soft constraint with `lambdaDrive` dual variable |

#### Architecture Validation

The pure AVBD AL approach confirms the §13.3 theoretical predictions:

1. **No Featherstone needed**: All joint types work as position-level bilateral
   AL constraints in the existing per-body 6×6 block descent.
2. **λ* is ID**: `getJointForce()` / `getJointTorque()` read converged dual
   variables directly — zero marginal cost.
3. **Solver-is-IK**: Adding `EndEffectorTarget` as another AL constraint row
   turns the solver into an IK solver with no separate algorithm.
4. **Unified constraint treatment**: Contacts, joint bilaterals, joint limits,
   drives, mimic coupling, and IK targets all live in one energy function.
5. **Deep chains stable**: 20-link chain under gravity converges without
   explosion (test93), though iteration count may need analysis in Phase 3.

---

## 14. PhysX-Side Pure AVBD Articulation — ACCEPTED (2026-03-15)

### 14.1 Final Status

| Metric | Value |
|--------|-------|
| PhysX Test Suite | **29 PASSED, 0 FAILED** (out of 29) |
| Deterministic Runs | 15/15 consecutive runs all 29/29 |
| Total Bugs Fixed | **12** (Bugs 1–12) |
| Architecture | Pure AVBD penalty-based (no Featherstone coupling) |
| Default Inner Iterations | 8 (contact-only islands) |
| Articulation Iterations | Per-articulation via `setSolverIterationCounts()` |

### 14.2 Architecture: Pure AVBD Penalty-Based Articulation

The Featherstone hybrid architecture (§2, §8, §13.1) has been **superseded**.
The PhysX AVBD solver now handles articulation internal joints entirely via
penalty constraints in the same block descent loop as contacts and external D6
joints. No Featherstone `solveInternalConstraints()` is called.

Key design points:

1. **Unified constraint loop**: Articulation internal joints (eFIX, eREVOLUTE,
   eSPHERICAL, ePRISMATIC), external D6 joints, and contacts are all penalty
   constraints solved in one ADMM iteration.
2. **Per-body 6×6 block descent**: Each articulation link is a standard AVBD
   body with mass/inertia. Joint constraints between links contribute to the
   6×6 Hessian via `J^T ρ J`.
3. **No alternating-solve lag**: Because all constraints are in one loop, there
   is no Featherstone↔AVBD coupling boundary that loses information per frame.
4. **Closed kinematic loops native**: D6 loop closures (e.g., scissor lift
   cross-links) are just more penalty rows — no special treatment needed.

### 14.3 Per-Island Adaptive Iteration Override

Implemented per-island iteration budgets to avoid penalizing simple contact
islands with the high iteration count required for articulations:

| Island Type | Iteration Count | Source |
|-------------|----------------|--------|
| Contact-only (no joints) | 8 | `AvbdSolverConfig.innerIterations` |
| Contains articulation | Per-articulation setting | `artic->setSolverIterationCounts(N)` |
| Joint-containing minimum | max(override, 8) | Ensures joint island floor |

Flow: `FeatherstoneArticulation::getIterationCounts()` → low byte = position
iters → per-island max → `AvbdIslandBatch.iterationOverride` → solver uses
override if > 0, else falls back to global default.

### 14.4 Bugs Fixed (PhysX-Side, Cumulative)

| Bug # | Category | Root Cause | Fix |
|-------|----------|-----------|-----|
| 1 | Crash | `totalBodyCount` used instead of island body count | Use `info.bodyCount` |
| 2 | Index | Body index mapping off by island offset | Add `bodyStartIdx` offset |
| 3 | Angular | D6 angular drive coupling mixed up axes | Correct axis indexing |
| 4 | Drive | Velocity drive used wrong target variable | Use `targetVelocity` |
| 5 | Explosion | Box stacking explosion from penalty imbalance | Tune `penScale` for body-body |
| 6 | Filter | `eHAS_TOUCH` flag filtering excluded valid contacts | Remove flag check |
| 7 | Namespace | `bodyRemapTable` offset collision between contact/joint prep | Separate remap ranges |
| 8 | Balance | Contact-joint penalty imbalance destabilized mixed scenes | Rebalance rho scaling |
| 9 | Encoding | 1-bit-per-axis motion encoding vs 2-bit getter | Use 2-bit `(value << axis*2)` encoding |
| 10 | Drive | Position drive used raw target instead of error | `posVel = (targetP - currentQ) * invDt` |
| 11 | Penalty | eFIX joints had lower rho than external D6 | Boost all-locked to `max(rho, 1e6)` |
| 12 | Byte order | `getIterationCounts()` high byte read as posIters | Use `iterWord & 0xFF` (low byte) |

### 14.5 Phase 4 Scissor Lift — PASSED

The scissor lift (SnippetArticulationRC scene) now passes all 9 sub-tests:

- Scissor lift stable alone (5s, no boxes)
- Scissor lift platform travels through a large lift stroke
- Scissor lift cycles up and down without load
- No NaN in 10s simulation
- No explosion (positions bounded) in 10s
- Boxes rest above ground after 10s
- Scissor lift retains meaningful lift stroke under load
- Scissor lift continues cyclic motion under load
- Articulation base remains stable

This exceeds the Featherstone hybrid ceiling (28/1, §13.1) and validates the
pure AVBD approach for closed-loop articulation systems.

### 14.6 Updated Phase Status

| Phase | Status | Notes |
|-------|--------|-------|
| Phase 1: AL Constraints | ✅ Standalone + PhysX | All joint types, limits, drives, friction |
| Phase 2: ID / IK / Mimic | ✅ Standalone | λ* extraction, IK targets, mimic joints |
| Phase 3: Convergence | ✅ Standalone | AA (47%), Chebyshev (29%), tree-sweep (18%) |
| Phase 4: Scissor Lift | ✅ **PhysX ACCEPTED** | 29/29 tests, closed-loop stable |
| Phase 5: Advanced | 🔧 Partial | Joint friction ✅, mimic ✅, floating base ✅ |

### 14.7 Theoretical Significance

The pure AVBD articulation solver demonstrates a result that contradicts the
original §2 analysis ("AVBD should NOT attempt to replace Featherstone"):

For **strongly coupled multi-articulation systems** (closed kinematic loops,
external contacts, cross-body D6 joints), a unified penalty-based block descent
solver achieves better convergence than a hybrid Featherstone + iterative
external solver. The alternating-solve lag in the hybrid architecture is the
dominant error source, exceeding the per-joint precision advantage of
Featherstone's direct recursion.

This does not invalidate Featherstone for single open-chain articulations, but
it establishes AVBD as the preferred solver for the robotics and mechanism
scenarios that matter most in practice.
