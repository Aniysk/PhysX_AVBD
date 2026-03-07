# AVBD Solver Algorithm Analysis

> **Analysis Date**: March 7, 2026  
> **Author**: Code review and reverse engineering  
> **Status**: Accepted (D6 Unification complete)

Status Legend: `Integrated` = merged into main code path; `Accepted` = integrated and validated by acceptance checks; `Pending` = not complete or acceptance gate not closed.

## Executive Summary

The current PhysX_AVBD path is a **unified AL + local Hessian solve** framework:

1. **Primal step**: per-body local system accumulates contacts and D6 joint rows into a local matrix/RHS.
2. **Local solve**: 3x3 decoupled or 6x6 coupled solve depending on runtime policy.
3. **Dual step**: multipliers are updated with stabilized `rhoDual` and decay.

All joint types (Spherical, Fixed, Revolute, Prismatic) have been **unified into a single D6 constraint path** ("万物皆D6"). Per-type solvers have been removed; joint behavior is determined entirely by per-DOF motion masks (LOCKED/FREE/LIMITED). Both PhysX and avbd_standalone share the same algorithm.

---

## 1. Paper Framework vs. Current Implementation

Standard AVBD in paper form can be summarized as:

$$\min_x \; \frac{1}{2h^2}\|x-\tilde x\|_M^2 + \sum_j \left(\frac{\rho_j}{2} C_j(x)^2 + \lambda_j C_j(x)\right)$$

with per-iteration local linearization and multiplier updates.

Current implementation remains consistent with this structure:

- **Local matrix accumulation**: inertia + $\sum (\rho J^T J)$
- **Local RHS accumulation**: inertial term + $\sum J(\rho C + \lambda)$
- **Dual update**: decayed AL multiplier update with inequality-aware clamping for one-sided limits

So this is best characterized as **paper-aligned framework with engineering approximations**, not a fully symbolic one-to-one reproduction.

---

## 2. Active Solve Paths and Policy

### 2.1 Global mode

- `enableLocal6x6Solve = false` remains the global default (cost-oriented mode).
- `enableLocal6x6Solve = true` enables always-coupled local solves.

### 2.2 Prismatic runtime override

- If a body touches any Prismatic joint, local solve is forced to 6x6.
- Reason: prismatic position projection introduces strong position-rotation coupling; 3x3 decoupling is less stable in these scenarios.

This means the practical behavior is **hybrid by policy**: mostly 3x3 where safe, 6x6 where coupling risk is known.

---

## 3. Joint Integration Snapshot (Current)

| Joint Type | Primal (Hessian/RHS) | Dual Update | Acceptance Status |
|------------|----------------------|-------------|-------------------|
| Spherical  | Unified D6 path | Unified D6 path | Accepted |
| Fixed      | Unified D6 path | Unified D6 path | Accepted |
| D6         | Unified D6 path | Unified D6 path | Accepted |
| Gear       | Velocity-ratio + post-solve motor | Inside iteration loop | Accepted |
| Prismatic  | Unified D6 path (pos/rot/limit) | Unified D6 path (signed limit clamp) | Accepted |
| Revolute   | Unified D6 path (cross-product axis alignment) | Unified D6 path | Accepted |

All joint types share one `addD6Contribution()` primal function and one `updateD6Dual()` dual function. The per-type behavior (which DOFs are constrained, limited, or free) is configured via `linearMotion`/`angularMotion` masks at joint creation time.

### Key Algorithmic Choices

- **Revolute angular constraint**: Cross-product axis alignment (`worldTwistA × worldTwistB`) projected onto perpendicular swing basis. Immune to twist-angle amplification.
- **Angular error computation**: Axis-angle decomposition (`2·acos(w)·axis`), accurate at large angles.
- **Motor drive**: Post-solve torque application after all ADMM iterations, decoupled from constraint Hessian.
- **Cone limit**: Per-body joint frame X-axis (`rotA * localFrameA` / `rotB * localFrameB`), not shared world axis.
- **Joint frames**: `localFrameB` derived from initial relative rotation at creation; both primal and dual use per-body frames.

---

## 4. Where the Approximation Lives

Main engineering approximations relative to idealized paper derivation:

- 3x3 decoupled path is still used in many bodies for throughput.
- Some Jacobian/Hessian terms are implemented in practical projected forms (especially around constrained-axis handling), prioritizing robustness and maintainability.
- Stabilization policy (decay, clamps, force-6x6 triggers) is tuned for game-scene behavior rather than strict symbolic minimality.

These are healthy as long as they remain measurable and regression-gated.

---

## 5. Risks and Control Points

### 5.1 Primary risks

1. **Documentation drift**: code integration state and acceptance state can diverge.
2. **Path divergence**: 3x3/6x6 behavior mismatch in edge cases.
3. **Drive/limit coupling sensitivity** in mixed joint scenes.

### 5.2 Controls

- Keep explicit status tiers: `integrated`, `accepted`, `pending`.
- Maintain PhysX↔standalone parity scenes for new joint semantics.
- Track residual/iteration/drift metrics in recurring benchmarks.

---

## 6. Recommended Next Steps

1. Add Distance joint to unified D6 path.
2. Implement hybrid Featherstone architecture for articulation support.
3. Continue low-iteration convergence tuning (toward paper-style 1x4 targets).
4. SOA refactoring for cache efficiency and GPU-readiness.
5. Add numerical precision regression tests (compare against analytical solutions).

---

## 7. Conclusion

The solver is a **unified AVBD-style AL framework with single D6 constraint path**, runtime path selection (3x3/6x6), and explicit stabilization policies. All joint types route through the same primal/dual pipeline. The D6 unification eliminates per-type code duplication, reduces maintenance surface, and ensures algorithmic improvements benefit all joint types simultaneously. PhysX and avbd_standalone implementations are fully aligned.

---

## References

- AVBD Paper: "Augmented Variable Block Descent for Rigid Body Simulation"
- XPBD Paper: Macklin et al., "XPBD: Position-Based Simulation of Compliant Constrained Dynamics"
- Source: `physx/source/lowleveldynamics/src/DyAvbdSolver.cpp`
