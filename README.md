# NVIDIA PhysX + AVBD Solver

> 🔬 **Research Fork**: Experimental AVBD (Augmented Variable Block Descent) constraint solver integrated into NVIDIA PhysX SDK.

Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved. BSD-3-Clause License.

## ⚠️ Project Status

Status Legend: `Integrated` = merged into main code path; `Accepted` = integrated and fully validated by current acceptance gates; `Pending` = not complete or acceptance not closed.

| Feature | Status | Notes |
|---------|--------|-------|
| Rigid Body Solver | ✅ Accepted | Contacts + unified AVBD local solve |
| Joint System | ⚠️ Integrated | Revolute, Prismatic, Spherical, Fixed, D6, Gear (Revolute acceptance pending) |
| Motor Drive | ⚠️ Integrated | Torque-based RevoluteJoint motor (needs scenario acceptance coverage) |
| Joint Limits | ⚠️ Integrated | Revolute, Prismatic, Spherical cone, D6 |
| Prismatic Hessian Path | ✅ Integrated | Position/rotation/limit rows accumulated in local Hessian |
| Prismatic 3x3 Fallback | ✅ Accepted | Bodies touching Prismatic are forced to 6x6 local solve |
| Standalone Alignment | ✅ Accepted | Prismatic limit sign/dual update aligned with PhysX path |
| Regression Baseline | ✅ Accepted | Default standalone suite runs 48 aligned cases |
| Custom Joint | ⏳ Pending | Custom constraint callbacks unsupported |
| Rack & Pinion | ⏳ Pending | RackAndPinionJoint unsupported |
| Mimic Joint | ⏳ Pending | MimicJoint unsupported |
| O(M) Constraint Lookup | ✅ Accepted | Eliminates O(N²) complexity |
| Multi-threaded Islands | ✅ Accepted | Per-island constraint mappings |
| Articulation | ⏳ Pending | Currently unsupported |
| Sleep / Wake | ⏳ Pending | Not implemented |
| Friction Model | ⚠️ Integrated | Coulomb approximation |

**For research and evaluation only. Not production-ready.**

## Recent Progress (2026-03)

- Prismatic joint has been migrated to the AVBD Hessian path in PhysX (primal accumulation + dual multiplier update).
- Debug defaults were cleaned up (joint debug disabled by default; named limit-flag constant used in parsing).
- Prismatic constraint comments were aligned with implementation semantics (3 world-axis projected rows).
- `avbd_standalone` was synchronized with PhysX behavior for prismatic limit handling and solve-path selection.
- Default standalone regression now excludes non-PhysX baseline cases (`prismatic drive`, `prismatic 3x3 chain`).

### Current Validation Snapshot

- ✅ Standalone aligned regression baseline passes (48/48 default suite).
- ✅ `SnippetChainmail` remains integrated for extreme impact and dense-joint stress regression.
- ✅ Prismatic baseline scenes validated under Hessian path + 6x6-on-touch policy.
- ⚠️ Revolute is integrated in solver code path, but full completion is pending SnippetJoint visual validation and acceptance checks.

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
Contact AL stability (DONE)         Joint Hessian integration (IN PROGRESS)
	Rigid body contacts stable      ->    Spherical/Fixed/D6/Gear/Prismatic: integrated
	AVBD usable as whole-scene solver     Revolute: integrated, acceptance pending
						|                                    |
	Lambda warm-starting                 Soft body / articulation / performance
	Iteration-efficiency tuning          Unified solver architecture
						|                                    |
							Multiplayer determinism across all the above
```

> See [docs/AVBD_SOLVER_README.md](docs/AVBD_SOLVER_README.md) and [docs/SOLVER_ALGORITHM_ANALYSIS.md](docs/SOLVER_ALGORITHM_ANALYSIS.md) for details.

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
| **Stabilized AL dual for joints** | Bounded dual step + decay (`rhoDual`, `lambdaDecay`) reduces overshoot while retaining AL memory. |
| **Prismatic force-6x6 on touch** | Prevents instability from 3x3 decoupling under strong position-rotation coupling. |
| **Standalone/PhysX semantic alignment** | Keep limit violation sign and dual clamp policy consistent to ensure parity. |

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
├── DyAvbdSolver.h/cpp       # Core solver
├── DyAvbdDynamics.h/cpp     # PhysX integration
├── DyAvbdTasks.h/cpp        # Multi-threading
├── DyAvbdTypes.h            # Config & data structures
├── DyAvbdConstraint.h       # Constraint definitions
├── DyAvbdJointSolver.h/cpp  # Joint solving
└── DyAvbdSolverBody.h       # Body state
```

## Profiling

PVD Profile Zones available:
- `AVBD.update` - Total update time
- `AVBD.solveWithJoints` - Main solver loop
- `AVBD.blockDescentWithJoints` - Constraint iterations
- `AVBD.updateLambda` - Multiplier updates

## Known Limitations

1. **No Articulation support** - Articulated bodies not implemented
2. **No Sleep/Wake** - Bodies remain active
3. **CPU only** - No GPU acceleration

## Original PhysX Documentation

- [PhysX User Guide](https://nvidia-omniverse.github.io/PhysX/physx/index.html)
- [API Documentation](https://nvidia-omniverse.github.io/PhysX)

## License

NVIDIA PhysX BSD-3-Clause. See [LICENSE.md](LICENSE.md).
