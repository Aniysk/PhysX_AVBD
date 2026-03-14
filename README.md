# NVIDIA PhysX + AVBD Solver

> 🔬 **Research Fork**: Experimental AVBD (Augmented Variable Block Descent) constraint solver integrated into NVIDIA PhysX SDK.

Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved. BSD-3-Clause License.

## ⚠️ Project Status

Status Legend: `Integrated` = merged into main code path; `Accepted` = integrated and fully validated by current acceptance gates; `Pending` = not complete or acceptance not closed.

| Feature | Status | Notes |
|---------|--------|-------|
| Rigid Body Solver | ✅ Accepted | Contacts + unified AVBD local solve |
| **D6 Unified Joint** | ✅ Accepted | All joint types (Spherical, Fixed, Revolute, Prismatic) unified into single D6 constraint path |
| Joint Limits | ✅ Accepted | Revolute angle, Prismatic linear, Spherical cone, D6 per-axis |
| Motor Drive | ✅ Accepted | Post-solve torque motor for revolute; SLERP drive for D6 |
| Gear Joint | ✅ Accepted | Velocity-ratio constraint with post-solve motor |
| Standalone Alignment | ✅ Accepted | PhysX and avbd_standalone share identical algorithm |
| Regression Baseline | ✅ Accepted | Standalone: 99/99; PhysX: 29/29 articulation tests |
| O(M) Constraint Lookup | ✅ Accepted | Eliminates O(N²) complexity |
| Multi-threaded Islands | ✅ Accepted | Per-island constraint mappings |
| Friction Model | ✅ Accepted | Coulomb cone, per-material coefficients from PxContactPatch |
| Custom Joint | ⏳ Pending | Custom constraint callbacks unsupported |
| Rack & Pinion | ⏳ Pending | RackAndPinionJoint unsupported |
| Mimic Joint | ⏳ Pending | MimicJoint unsupported |
| Articulation | ✅ Accepted | Pure AVBD penalty-based, 29/29 tests, per-island iterations |
| Sleep / Wake | ⏳ Pending | Not implemented |

**For research and evaluation only. Not production-ready.**

## Recent Progress (2026-03)

### Articulation Solver (NEW)

Articulation support is now fully implemented using a **pure AVBD penalty-based architecture** — no Featherstone dependency. All articulation internal joints are solved as AL constraint rows in the same block descent loop as contacts and external D6 joints.

Key achievements:
- **29/29 PhysX tests pass** including scissor lift with closed kinematic loops, loaded boxes, and 10s stability.
- **12 bugs fixed** during integration: motion encoding (2-bit-per-axis), position drive error computation, eFIX penalty boost, iteration count byte order, and more.
- **Per-island adaptive iterations**: Articulations use `setSolverIterationCounts(N)` for higher iteration budgets; contact-only islands default to 8 iterations.
- **Exceeds Featherstone hybrid ceiling**: The alternating-solve lag in Featherstone coupling was the dominant error source for strongly coupled systems. Unified penalty solving eliminates this boundary.
- **Standalone**: 99/99 tests including convergence acceleration (Anderson Acceleration 47%, Chebyshev 29%), ID extraction via λ*, solver-is-IK, mimic joints.

### D6 Unification

All joint types have been unified into a single D6 constraint path. Per-type independent solvers (Spherical, Fixed, Revolute, Prismatic) have been replaced by one shared `addD6Contribution()` / `updateD6Dual()` pipeline, with joint behavior determined entirely by motion masks (LOCKED/FREE/LIMITED per DOF).

Key changes:
- **Architecture**: ~400 lines of redundant per-type constraint code removed; all joints route through unified D6 primal + dual path.
- **Angular constraints**: Cross-product axis alignment for revolute-pattern D6 joints, replacing quaternion tangent-space error. Immune to twist-angle amplification at large rotations.
- **Angular error**: Axis-angle decomposition (`2·acos(w)·axis`) replaces tangent-space `2·vec(errQ)`, accurate at large angles.
- **Motor**: Post-solve torque motor decoupled from ADMM constraint Hessian, replacing in-iteration AL velocity drive.
- **Gear joint**: Dual update moved inside ADMM iteration loop; NaN from driveForceLimit overflow fixed.
- **Cone limit**: Per-body joint frame axes derived from `localFrameA`/`localFrameB`, replacing shared axis.
- **Joint frames**: `localFrameB` derived from initial relative rotation at joint creation. All factory methods updated.
- **Standalone sync**: `avbd_standalone` algorithm fully aligned with PhysX AVBD implementation.

### Friction Integration

Friction was already fully implemented in the AVBD solver (3-DOF contact model: 1 normal + 2 tangent), but PhysX contact preparation hardcoded `friction = 0.5f` and `restitution = 0.0f` instead of reading from materials.

Key changes:
- **Material read-through**: `constraint.friction` and `constraint.restitution` now read from `PxContactPatch::dynamicFriction` / `restitution` (combined by narrowphase).
- **Tangent basis**: Aligned with standalone — `PxAbs(normal.y) > 0.9f` branch for robustness.
- **Standalone tests**: 18 friction-specific tests (slope sliding, anisotropy, Coulomb cone, geometric mean combining, warmstart, penalty growth, etc.).

### Current Validation Snapshot

- ✅ Standalone regression baseline passes (99/99 suite).
- ✅ PhysX articulation regression passes (29/29 tests, 15 consecutive deterministic runs).
- ✅ PhysX debug build succeeds with all Snippets.
- ✅ `SnippetChainmail` remains integrated for extreme impact and dense-joint stress regression.
- ✅ All joint types validated: Spherical chain, breakable Fixed, damped D6, limited Prismatic, limited Revolute.
- ✅ Gear joint stable (no NaN, no oscillation, no twist amplification).
- ✅ Friction reads per-material coefficients; Coulomb cone + augmented Lagrangian validated.
- ✅ Scissor lift with closed kinematic loops and loaded boxes: stable 10s.
- ✅ Per-island adaptive iteration override: articulations get high iterations, contacts get low.

## SnippetChainmail Demo

https://github.com/user-attachments/assets/2ab299c7-8f7f-4bf2-b8b5-7de8033b17f8

## Why AVBD?

PhysX's built-in TGS/PGS are **velocity-level** iterative solvers that hit fundamental limits in several scenarios:

| Problem | TGS/PGS Limitation | AVBD Direction |
|---------|---------------------|----------------|
| **High mass-ratio joints** | Condition number explosion, rubber-banding | Augmented Lagrangian + local Hessian solve |
| **Multiplayer sync** | Velocity integration accumulates FP error | Position-level solve with stronger state consistency |
| **Cloth & soft body** | Requires separate solver pipelines | Position-level framework is more naturally extensible |

AVBD introduces a **unified position-level constraint solving framework** targeting:

1. Stable high mass-ratio interaction chains.
2. Whole-scene robustness under mixed contact/joint constraints.
3. Better deterministic behavior for server-authoritative simulation.
4. Future rigid/soft-body unification on a common optimization-style solver structure.

### Roadmap Snapshot

```
Contact AL stability (DONE)         D6 Unified Joint System (DONE)
  Rigid body contacts stable      ->  All joints unified into D6 path
  AVBD usable as whole-scene solver   Spherical/Fixed/Revolute/Prismatic/D6/Gear: accepted
            |                                    |
  Lambda warm-starting (DONE)        Articulation Solver (DONE)
  Iteration-efficiency tuning        Pure AVBD penalty-based, 29/29 PhysX tests
            |                        Per-island adaptive iterations
            |                                    |
                Soft body / performance / GPU path
                SOA refactoring, multiplayer determinism
```

## Solver Architecture

### Unified AVBD Hessian Approach

The solver accumulates **contacts and joints** into a per-body local system (typically 6x6), then solves via LDLT:

```
For each body i:
	H = M/h^2 * I_6x6
	g = M/h^2 * (x_i - x_tilde)

	For each contact/joint row:
		H += rho_eff * J^T J
		g += J * (rho_eff * C + lambda)

	Dual update (stabilized AL):
		rhoDual = min(Mh^2, rho^2/(rho + Mh^2))
		lambda  = decay * lambda + rhoDual * C

	delta = LDLT_solve(H, g)
	x_i -= delta
```

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **Unified D6 joint path** | All joint types (Spherical, Fixed, Revolute, Prismatic) map to a single D6 constraint with motion masks. |
| **Cross-product axis alignment** | Revolute-pattern angular constraints use `twistA x twistB` instead of quaternion error, avoiding twist amplification. |
| **Post-solve motor** | Motor torque applied after ADMM iterations, decoupled from constraint Hessian for stability. |
| **Stabilized AL dual for joints** | Bounded dual step + decay (`rhoDual`, `lambdaDecay`) reduces overshoot while retaining AL memory. |
| **Prismatic force-6x6 on touch** | Prevents instability from 3x3 decoupling under strong position-rotation coupling. |
| **Standalone/PhysX algorithm parity** | Both codebases share identical constraint formulation, solve pipeline, and dual update logic. |

## AVBD Solver Overview

AVBD is a position-based constraint solver using:
- **Block Coordinate Descent** - Per-body 6x6 local system solve
- **Augmented Lagrangian** - Multiplier updates for constraint satisfaction
- **Island-level Parallelism** - Independent islands solve concurrently

### Comparison with TGS/PGS

| Property | PGS | TGS | AVBD |
|----------|-----|-----|------|
| Solve Level | Velocity | Velocity | **Position** |
| Convergence | Linear | Sublinear | Quadratic |
| Stack Stability | Fair | Good | **Excellent** |
| Cost per Iteration | Low | Medium | Medium-High |

## Quick Start

### Build

```bash
cd physx
./generate_projects.bat  # Windows
./generate_projects.sh   # Linux
```

### Enable AVBD

```cpp
PxSceneDesc sceneDesc(physics->getTolerancesScale());
sceneDesc.solverType = PxSolverType::eAVBD;
```

## Source Structure

```
physx/source/lowleveldynamics/src/
├── DyAvbdSolver.h/cpp            # Core solver (contact-only path)
├── DyAvbdSolverJointPath.cpp     # Solver joint path (solveWithJoints entry)
├── DyAvbdJointProjection.h/cpp   # Per-joint-type constraint projection & multiplier update
├── DyAvbdDynamics.h/cpp          # PhysX integration & frame orchestration
├── DyAvbdDynamicsPrep.cpp        # Contact & joint constraint preparation
├── DyAvbdTasks.h/cpp             # Multi-threading
├── DyAvbdTypes.h                 # Config & data structures
├── DyAvbdConstraint.h            # Constraint definitions
└── DyAvbdSolverBody.h/cpp        # Body state
```

## Profiling

PVD Profile Zones available:
- `AVBD.update` - Total update time
- `AVBD.solveWithJoints` - Main solver loop
- `AVBD.blockDescentWithJoints` - Constraint iterations
- `AVBD.updateLambda` - Multiplier updates

## Known Limitations

1. **No Sleep/Wake** - Bodies remain active
2. **CPU only** - No GPU acceleration
3. **Articulation cold-start** - Long open chains (N>20) need higher iteration counts for cold-start convergence

## Original PhysX Documentation

- [PhysX User Guide](https://nvidia-omniverse.github.io/PhysX/physx/index.html)
- [API Documentation](https://nvidia-omniverse.github.io/PhysX)

## License

NVIDIA PhysX BSD-3-Clause. See [LICENSE.md](LICENSE.md).
