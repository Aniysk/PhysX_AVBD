# AVBD Solver Algorithm Analysis

> **Analysis Date**: March 2, 2026  
> **Author**: Code review and reverse engineering  
> **Status**: Accepted (state-aligned as of current implementation)

Status Legend: `Integrated` = merged into main code path; `Accepted` = integrated and validated by acceptance checks; `Pending` = not complete or acceptance gate not closed.

## Executive Summary

The current PhysX_AVBD path is a **unified AL + local Hessian solve** framework:

1. **Primal step**: per-body local system accumulates contacts and supported joint rows into a local matrix/RHS.
2. **Local solve**: 3x3 decoupled or 6x6 coupled solve depending on runtime policy.
3. **Dual step**: multipliers are updated with stabilized `rhoDual` and decay.

Compared with the older state, **Prismatic is now integrated into Hessian accumulation**, and prismatic-touching bodies are forced to 6x6 local solve for stability. Revolute rows are integrated in the solver path as well, but completion should be treated as **acceptance-pending** until SnippetJoint visual verification is closed.

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
| Spherical  | Integrated | Integrated | Accepted |
| Fixed      | Integrated | Integrated | Accepted |
| D6         | Integrated | Integrated | Accepted (with scene-dependent tuning) |
| Gear       | Integrated | Integrated | Accepted |
| Prismatic  | Integrated (pos/rot/limit) | Integrated (signed limit clamp) | Accepted |
| Revolute   | Integrated in code path | Integrated | **Pending visual acceptance** |

Note: “integrated” != “fully accepted.” Acceptance must include scenario-level validation, not only code-path presence.

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

1. Close Revolute acceptance with SnippetJoint visual checklist.
2. Add parity harness for mixed-joint chains (revolute + prismatic + fixed).
3. Continue low-iteration convergence tuning (toward paper-style 1x4 targets).
4. Keep architecture documentation synchronized with code comments and regression outputs.

---

## 7. Conclusion

The solver is no longer accurately described as “3x3 + GS fallback” only. The current state is a **unified AVBD-style AL framework with local Hessian accumulation**, runtime path selection (3x3/6x6), and explicit stabilization policies. This is a sound engineering trajectory, provided acceptance gates and parity tests remain first-class.

---

## References

- AVBD Paper: "Augmented Variable Block Descent for Rigid Body Simulation"
- XPBD Paper: Macklin et al., "XPBD: Position-Based Simulation of Compliant Constrained Dynamics"
- Source: `physx/source/lowleveldynamics/src/DyAvbdSolver.cpp`
