#pragma once
#include "avbd_d6_core.h"
#include "avbd_types.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace AvbdRef {

// =============================================================================
// Pure AVBD Articulation — Phase 1: Articulation as AL Constraints
//
// Each articulation joint is modeled as a set of bilateral/inequality AL
// constraint rows in the existing AVBD block descent solver. This operates
// in maximal (Cartesian) coordinates — no Featherstone, no generalized
// coordinates. Joint constraints are treated identically to contact and D6
// constraints: they contribute Jacobian rows to the per-body 6×6 Hessian.
//
// Joint types:
//   REVOLUTE  — 5 bilateral (3 positional + 2 angular alignment) + 1 free
//   PRISMATIC — 5 bilateral (2 positional + 3 angular) + 1 free along axis
//   SPHERICAL — 3 bilateral (positional) + 3 free (angular)
//   FIXED     — 6 bilateral (3 positional + 3 angular)
//
// Key insight from §13.3: the converged AL multiplier λ* for each joint
// constraint row IS the inverse dynamics output (joint torque/force).
// =============================================================================

enum ArticJointType {
  eARTIC_REVOLUTE = 0,      // 1-DOF rotation
  eARTIC_PRISMATIC = 1,     // 1-DOF translation
  eARTIC_SPHERICAL = 2,     // 3-DOF rotation
  eARTIC_FIXED = 3          // 0-DOF
};

// Per-joint data within an articulation
struct ArticulationJoint {
  ArticJointType type;
  uint32_t bodyIndex;          // index into Solver::bodies
  int32_t parentJointIndex;    // -1 = root (attached to world or floating base)

  // Joint frame definition
  Vec3 parentAnchor;           // attachment point in parent body local frame
  Vec3 childAnchor;            // attachment point in child body local frame
  Vec3 axis;                   // joint axis in parent local frame (revolute/prismatic)

  // Constraint dual variables (AL multipliers)
  // Positional: up to 3 bilateral rows
  Vec3 lambdaPos;
  // Angular: up to 3 rows
  Vec3 lambdaAng;

  // Joint limits (for revolute angle / prismatic displacement)
  bool hasLimits;
  float limitLower;
  float limitUpper;
  float lambdaLimitLower;      // inequality AL multiplier (lower bound)
  float lambdaLimitUpper;      // inequality AL multiplier (upper bound)

  // PD drive
  bool hasDrive;
  float driveTarget;           // target position/angle
  float driveTargetVelocity;   // target velocity
  float driveStiffness;        // Kp
  float driveDamping;          // Kd
  float driveMaxForce;         // torque/force clamp
  float lambdaDrive;           // AL dual multiplier for prismatic drive

  // Joint friction
  float frictionCoefficient;   // viscous friction coefficient

  // Reference poses (captured at articulation setup)
  Quat refRotParent;           // parent rotation at setup time
  Quat refRotChild;            // child rotation at setup time
  Quat relativeRotation;       // refRotParent^-1 * refRotChild

  // Joint local frames (computed from axis)
  Quat localFrameParent;       // joint frame in parent body space
  Quat localFrameChild;        // joint frame in child body space

  // Penalty parameter
  float rho;

  ArticulationJoint()
      : type(eARTIC_REVOLUTE), bodyIndex(0), parentJointIndex(-1),
        hasLimits(false), limitLower(0), limitUpper(0),
        lambdaLimitLower(0), lambdaLimitUpper(0),
        hasDrive(false), driveTarget(0), driveTargetVelocity(0),
        driveStiffness(0), driveDamping(0), driveMaxForce(0),
        lambdaDrive(0),
        frictionCoefficient(0),
        rho(1e6f) {}
};

// Mimic joint: linear coupling q_A + gearRatio * q_B + offset = 0
struct MimicJointDef {
  int jointA;       // leader joint index
  int jointB;       // follower joint index
  float gearRatio;  // q_A + gearRatio * q_B + offset = 0
  float offset;
  float lambda;     // AL dual multiplier
  float rho;        // penalty parameter

  MimicJointDef() : jointA(-1), jointB(-1), gearRatio(1.0f), offset(0),
                    lambda(0), rho(1e6f) {}
};

// End-effector IK target: constrain a world-space point on a link
struct EndEffectorTarget {
  int jointIdx;         // which link to constrain (joint index)
  Vec3 localPoint;      // point in link local frame
  Vec3 targetPos;       // desired world position
  Vec3 lambda;          // AL dual multiplier (3 rows)
  float rho;            // penalty parameter

  EndEffectorTarget() : jointIdx(-1), rho(1e6f) {}
};

// =============================================================================
// Articulation — a tree of bodies connected by joints
//
// The articulation is a convenience wrapper that:
//   1. Stores the joint topology (parent-child links)
//   2. Contributes AL constraint rows to the per-body Hessian during primal
//   3. Updates dual multipliers λ the same way as D6/contact constraints
//   4. Optionally extracts converged λ* as inverse dynamics output
// =============================================================================

struct Articulation {
  bool fixedBase;              // true = root attached to world
  Vec3 fixedBasePos;           // world position of fixed base anchor
  std::vector<ArticulationJoint> joints;

  Articulation() : fixedBase(true) {}

  // -----------------------------------------------------------------------
  // addLink — add a link to the articulation
  //
  // parentJoint: index into this->joints (-1 = root, attached to world/base)
  // bodyIdx:     index into Solver::bodies
  // type:        joint type
  // axis:        joint axis in parent local frame (revolute: rotation axis,
  //              prismatic: slide axis)
  // parentAnchor: joint point in parent body local frame
  // childAnchor:  joint point in child body local frame
  // -----------------------------------------------------------------------
  int addLink(uint32_t bodyIdx, int32_t parentJoint, ArticJointType type,
              Vec3 axis, Vec3 parentAnchor, Vec3 childAnchor,
              const std::vector<Body> &bodies) {
    ArticulationJoint j;
    j.type = type;
    j.bodyIndex = bodyIdx;
    j.parentJointIndex = parentJoint;
    j.axis = axis.normalized();
    j.parentAnchor = parentAnchor;
    j.childAnchor = childAnchor;

    // Capture reference rotations
    j.refRotChild = bodies[bodyIdx].rotation;
    if (parentJoint >= 0) {
      uint32_t parentBody = joints[parentJoint].bodyIndex;
      j.refRotParent = bodies[parentBody].rotation;
    } else {
      j.refRotParent = Quat(); // world / identity for root
    }
    j.relativeRotation = j.refRotParent.conjugate() * j.refRotChild;

    // Build joint local frames from axis
    Vec3 ax = j.axis;
    Vec3 perp1;
    if (std::fabs(ax.x) < 0.9f)
      perp1 = ax.cross(Vec3(1, 0, 0)).normalized();
    else
      perp1 = ax.cross(Vec3(0, 1, 0)).normalized();
    Vec3 perp2 = ax.cross(perp1).normalized();

    // Joint frame: X = joint axis, Y = perp1, Z = perp2
    j.localFrameParent = quatFromColumns(ax, perp1, perp2);
    j.localFrameChild = j.relativeRotation.conjugate() * j.localFrameParent;

    int idx = (int)joints.size();
    joints.push_back(j);
    return idx;
  }

  // -----------------------------------------------------------------------
  // Configure joint limits
  // -----------------------------------------------------------------------
  void setJointLimits(int jointIdx, float lower, float upper) {
    if (jointIdx >= 0 && jointIdx < (int)joints.size()) {
      joints[jointIdx].hasLimits = true;
      joints[jointIdx].limitLower = lower;
      joints[jointIdx].limitUpper = upper;
    }
  }

  // -----------------------------------------------------------------------
  // Configure PD position drive
  // -----------------------------------------------------------------------
  void setJointDrive(int jointIdx, float stiffness, float damping,
                     float maxForce, float targetPos = 0.0f,
                     float targetVel = 0.0f) {
    if (jointIdx >= 0 && jointIdx < (int)joints.size()) {
      joints[jointIdx].hasDrive = true;
      joints[jointIdx].driveStiffness = stiffness;
      joints[jointIdx].driveDamping = damping;
      joints[jointIdx].driveMaxForce = maxForce;
      joints[jointIdx].driveTarget = targetPos;
      joints[jointIdx].driveTargetVelocity = targetVel;
    }
  }

  // -----------------------------------------------------------------------
  // Configure joint friction
  // -----------------------------------------------------------------------
  void setJointFriction(int jointIdx, float coefficient) {
    if (jointIdx >= 0 && jointIdx < (int)joints.size()) {
      joints[jointIdx].frictionCoefficient = coefficient;
    }
  }

  // -----------------------------------------------------------------------
  // Get parent body index for a joint
  // -----------------------------------------------------------------------
  uint32_t getParentBodyIndex(int jointIdx) const {
    if (jointIdx < 0 || jointIdx >= (int)joints.size()) return UINT32_MAX;
    int parent = joints[jointIdx].parentJointIndex;
    if (parent < 0) return UINT32_MAX; // root: parent is world
    return joints[parent].bodyIndex;
  }

  // -----------------------------------------------------------------------
  // Compute hinge angle (revolute joint)
  // Uses the same signed-angle method as existing D6Joint
  // -----------------------------------------------------------------------
  float computeJointAngle(int jointIdx, const std::vector<Body> &bodies) const {
    const ArticulationJoint &j = joints[jointIdx];
    uint32_t parentBody = getParentBodyIndex(jointIdx);

    Quat rotParent = (parentBody == UINT32_MAX) ? Quat() : bodies[parentBody].rotation;
    Quat rotChild = bodies[j.bodyIndex].rotation;

    // World-space joint frame
    Quat worldFrameParent = rotParent * j.localFrameParent;
    Quat worldFrameChild = rotChild * j.localFrameChild;

    // Relative rotation: parent_frame^-1 * child_frame
    Quat rel = worldFrameParent.conjugate() * worldFrameChild;
    if (rel.w < 0) rel = rel * (-1.0f);

    // Extract twist angle around X axis (joint axis)
    return 2.0f * atan2f(rel.x, rel.w);
  }

  // -----------------------------------------------------------------------
  // Compute prismatic displacement
  // -----------------------------------------------------------------------
  float computeJointDisplacement(int jointIdx, const std::vector<Body> &bodies) const {
    const ArticulationJoint &j = joints[jointIdx];
    uint32_t parentBody = getParentBodyIndex(jointIdx);

    Vec3 posParent = (parentBody == UINT32_MAX) ? fixedBasePos : bodies[parentBody].position;
    Quat rotParent = (parentBody == UINT32_MAX) ? Quat() : bodies[parentBody].rotation;

    Vec3 worldParentAnchor = posParent + rotParent.rotate(j.parentAnchor);
    Vec3 worldChildAnchor = bodies[j.bodyIndex].position +
                            bodies[j.bodyIndex].rotation.rotate(j.childAnchor);

    Vec3 worldAxis = rotParent.rotate(j.axis);
    return (worldChildAnchor - worldParentAnchor).dot(worldAxis);
  }

  // -----------------------------------------------------------------------
  // Mimic joints: linear coupling between two joints
  // -----------------------------------------------------------------------
  std::vector<MimicJointDef> mimicJoints;

  void addMimicJoint(int leaderJoint, int followerJoint,
                     float gearRatio, float offset = 0.0f, float rho_ = 1e6f) {
    MimicJointDef m;
    m.jointA = leaderJoint;
    m.jointB = followerJoint;
    m.gearRatio = gearRatio;
    m.offset = offset;
    m.rho = rho_;
    mimicJoints.push_back(m);
  }

  // -----------------------------------------------------------------------
  // End-effector IK targets
  // -----------------------------------------------------------------------
  std::vector<EndEffectorTarget> ikTargets;

  void addIKTarget(int jointIdx, Vec3 localPoint, Vec3 targetPos,
                   float rho_ = 1e6f) {
    EndEffectorTarget t;
    t.jointIdx = jointIdx;
    t.localPoint = localPoint;
    t.targetPos = targetPos;
    t.rho = rho_;
    ikTargets.push_back(t);
  }

  void setIKTarget(int targetIdx, Vec3 targetPos) {
    if (targetIdx >= 0 && targetIdx < (int)ikTargets.size())
      ikTargets[targetIdx].targetPos = targetPos;
  }

  // -----------------------------------------------------------------------
  // ID extraction: read converged λ* as joint torques/forces
  //
  // Per §13.6: τ_j = J_j^T λ*_j. The bilateral constraint multipliers
  // directly encode the constraint force at convergence.
  //
  // For revolute: λ_pos (3D) = positional joint force, λ_ang.y/z = alignment torque
  //   The net joint torque around the free axis = residual from the angular
  //   alignment + limit + drive contributions.
  // For prismatic: λ_pos gives 2D lateral force, λ_ang gives 3D alignment torque
  //   The net joint force along the free axis comes from limit + drive.
  // -----------------------------------------------------------------------
  Vec3 getJointForce(int jointIdx) const {
    if (jointIdx < 0 || jointIdx >= (int)joints.size()) return Vec3();
    return joints[jointIdx].lambdaPos;
  }

  Vec3 getJointTorque(int jointIdx) const {
    if (jointIdx < 0 || jointIdx >= (int)joints.size()) return Vec3();
    return joints[jointIdx].lambdaAng;
  }

  // Get the scalar constraint force along the free axis (limit + drive contribution)
  float getJointAxisForce(int jointIdx) const {
    if (jointIdx < 0 || jointIdx >= (int)joints.size()) return 0.0f;
    const ArticulationJoint &j = joints[jointIdx];
    // For limits: the net force is lambdaLimitUpper + lambdaLimitLower
    return j.lambdaLimitUpper + j.lambdaLimitLower;
  }

  // -----------------------------------------------------------------------
  // Convergence metric: max positional constraint violation across all joints
  // -----------------------------------------------------------------------
  float computeMaxPositionViolation(const std::vector<Body> &bodies) const {
    float maxViol = 0.0f;
    for (int ji = 0; ji < (int)joints.size(); ji++) {
      const ArticulationJoint &j = joints[ji];
      uint32_t parentBody = getParentBodyIndex(ji);
      bool parentStatic = (parentBody == UINT32_MAX);
      Vec3 worldParentAnchor;
      if (parentStatic) {
        worldParentAnchor = fixedBasePos + j.parentAnchor;
      } else {
        worldParentAnchor = bodies[parentBody].position +
                            bodies[parentBody].rotation.rotate(j.parentAnchor);
      }
      Vec3 worldChildAnchor = bodies[j.bodyIndex].position +
                              bodies[j.bodyIndex].rotation.rotate(j.childAnchor);
      float violation = (worldParentAnchor - worldChildAnchor).length();
      maxViol = std::max(maxViol, violation);
    }
    return maxViol;
  }
};

// =============================================================================
// addArticulationContribution — primal update per body
//
// Accumulates articulation constraint Hessian/gradient contributions into
// the per-body 6×6 LHS/RHS system, exactly like addD6Contribution does
// for D6 joints.
// =============================================================================
inline void addArticulationContribution(const Articulation &artic,
                                        int jointIdx, uint32_t bi,
                                        const std::vector<Body> &bodies,
                                        float dt, Mat66 &lhs, Vec6 &rhs) {
  const ArticulationJoint &j = artic.joints[jointIdx];
  uint32_t parentBody = artic.getParentBodyIndex(jointIdx);
  uint32_t childBody = j.bodyIndex;

  bool isChild = (bi == childBody);
  bool isParent = (bi == parentBody);
  if (!isChild && !isParent) return;

  bool parentStatic = (parentBody == UINT32_MAX);
  float dt2 = dt * dt;
  float sign = isParent ? 1.0f : -1.0f;

  const Body &body = bodies[bi];
  float effectiveRho = std::max(j.rho, body.mass / dt2);

  // World-space anchors
  Vec3 r = isParent ? body.rotation.rotate(j.parentAnchor)
                    : body.rotation.rotate(j.childAnchor);
  Vec3 worldParentAnchor, worldChildAnchor;
  if (parentStatic) {
    worldParentAnchor = artic.fixedBasePos + j.parentAnchor;
  } else {
    worldParentAnchor = bodies[parentBody].position +
                        bodies[parentBody].rotation.rotate(j.parentAnchor);
  }
  worldChildAnchor = bodies[childBody].position +
                     bodies[childBody].rotation.rotate(j.childAnchor);
  Vec3 C_lin = worldParentAnchor - worldChildAnchor;

  // Rotations
  Quat rotParent = parentStatic ? Quat() : bodies[parentBody].rotation;
  Quat rotChild = bodies[childBody].rotation;

  // Joint frames in world space
  Quat worldFrameParent = parentStatic ? j.localFrameParent
                                       : rotParent * j.localFrameParent;
  worldFrameParent = worldFrameParent.normalized();
  Quat worldFrameChild = rotChild * j.localFrameChild;
  worldFrameChild = worldFrameChild.normalized();

  // Joint axis in world space
  Vec3 worldAxis = worldFrameParent.rotate(Vec3(1, 0, 0));

  // --- Positional constraints ---
  switch (j.type) {
  case eARTIC_REVOLUTE:
  case eARTIC_FIXED:
  case eARTIC_SPHERICAL: {
    // All 3 positional DOFs locked (spherical joint = ball-socket)
    // Use world axes for numerical stability when all 3 are locked
    Vec3 axes[3] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
    for (int k = 0; k < 3; k++) {
      float Ck = C_lin.dot(axes[k]);
      Vec6 J(axes[k] * sign, r.cross(axes[k]) * sign);
      float lam = (&j.lambdaPos.x)[k];
      float f = effectiveRho * Ck + lam;
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
    break;
  }
  case eARTIC_PRISMATIC: {
    // 2 positional DOFs locked (perpendicular to slide axis), 1 free
    Vec3 perp1 = worldFrameParent.rotate(Vec3(0, 1, 0));
    Vec3 perp2 = worldFrameParent.rotate(Vec3(0, 0, 1));
    Vec3 perpAxes[2] = {perp1, perp2};
    for (int k = 0; k < 2; k++) {
      float Ck = C_lin.dot(perpAxes[k]);
      Vec6 J(perpAxes[k] * sign, r.cross(perpAxes[k]) * sign);
      float lam = (&j.lambdaPos.x)[k];
      float f = effectiveRho * Ck + lam;
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
    break;
  }
  }

  // --- Angular constraints ---
  switch (j.type) {
  case eARTIC_FIXED: {
    // All 3 angular DOFs locked
    // Use axis-angle decomposition for angular error
    for (int k = 0; k < 3; k++) {
      float angErr = computeAngularError(rotParent, rotChild,
                                         j.localFrameParent, j.localFrameChild, k);
      Vec3 localAxis(k == 0 ? 1.0f : 0.0f, k == 1 ? 1.0f : 0.0f,
                     k == 2 ? 1.0f : 0.0f);
      Vec3 wAxis = worldFrameParent.rotate(localAxis);
      Vec6 J(Vec3(), wAxis * sign * (-1.0f));
      float lam = (&j.lambdaAng.x)[k];
      float f = effectiveRho * angErr + lam;
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
    break;
  }
  case eARTIC_REVOLUTE: {
    // 2 angular DOFs locked (swing), 1 free (twist around joint axis)
    // Use cross-product axis alignment (same as D6 revolute pattern)
    Vec3 worldTwistParent = worldFrameParent.rotate(Vec3(1, 0, 0));
    Vec3 worldTwistChild = worldFrameChild.rotate(Vec3(1, 0, 0));
    Vec3 axisViolation = worldTwistParent.cross(worldTwistChild);

    Vec3 perp1, perp2;
    if (std::fabs(worldTwistParent.x) < 0.9f)
      perp1 = worldTwistParent.cross(Vec3(1, 0, 0));
    else
      perp1 = worldTwistParent.cross(Vec3(0, 1, 0));
    float p1Len = perp1.length();
    if (p1Len > 1e-6f) perp1 = perp1 * (1.0f / p1Len);
    perp2 = worldTwistParent.cross(perp1);
    float p2Len = perp2.length();
    if (p2Len > 1e-6f) perp2 = perp2 * (1.0f / p2Len);

    float err1 = axisViolation.dot(perp1);
    float err2 = axisViolation.dot(perp2);

    // Row 1
    {
      Vec6 J(Vec3(), perp1 * sign * (-1.0f));
      float f = effectiveRho * err1 + j.lambdaAng.y;
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
    // Row 2
    {
      Vec6 J(Vec3(), perp2 * sign * (-1.0f));
      float f = effectiveRho * err2 + j.lambdaAng.z;
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
    break;
  }
  case eARTIC_PRISMATIC: {
    // All 3 angular DOFs locked
    for (int k = 0; k < 3; k++) {
      float angErr = computeAngularError(rotParent, rotChild,
                                         j.localFrameParent, j.localFrameChild, k);
      Vec3 localAxis(k == 0 ? 1.0f : 0.0f, k == 1 ? 1.0f : 0.0f,
                     k == 2 ? 1.0f : 0.0f);
      Vec3 wAxis = worldFrameParent.rotate(localAxis);
      Vec6 J(Vec3(), wAxis * sign * (-1.0f));
      float lam = (&j.lambdaAng.x)[k];
      float f = effectiveRho * angErr + lam;
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
    break;
  }
  case eARTIC_SPHERICAL:
    // All angular DOFs free — no angular constraint rows
    break;
  }

  // --- Joint limits (inequality AL constraint) ---
  if (j.hasLimits) {
    if (j.type == eARTIC_REVOLUTE) {
      float angle = artic.computeJointAngle(jointIdx, bodies);
      // Lower limit violation
      if (angle < j.limitLower || j.lambdaLimitLower > 0.0f) {
        float viol = angle - j.limitLower; // negative when violated
        Vec6 J(Vec3(), worldAxis * sign * (-1.0f));
        float f = effectiveRho * viol + j.lambdaLimitLower;
        f = std::min(0.0f, f); // lower limit: force pushes angle up (negative lambda)
        if (std::fabs(f) > 0.0f) {
          rhs += J * f;
          lhs += outer(J, J * effectiveRho);
        }
      }
      // Upper limit violation
      if (angle > j.limitUpper || j.lambdaLimitUpper < 0.0f) {
        float viol = angle - j.limitUpper; // positive when violated
        Vec6 J(Vec3(), worldAxis * sign * (-1.0f));
        float f = effectiveRho * viol + j.lambdaLimitUpper;
        f = std::max(0.0f, f); // upper limit: force pushes angle down (positive lambda)
        if (std::fabs(f) > 0.0f) {
          rhs += J * f;
          lhs += outer(J, J * effectiveRho);
        }
      }
    } else if (j.type == eARTIC_PRISMATIC) {
      float disp = artic.computeJointDisplacement(jointIdx, bodies);
      // Prismatic limit Jacobian sign is negated vs bilateral because
      // displacement = (child - parent).dot(axis), opposite to C_lin = parent - child.
      float limitSign = -sign;
      if (disp < j.limitLower || j.lambdaLimitLower > 0.0f) {
        float viol = disp - j.limitLower;
        Vec6 J(worldAxis * limitSign, r.cross(worldAxis) * limitSign);
        float f = effectiveRho * viol + j.lambdaLimitLower;
        f = std::min(0.0f, f);
        if (std::fabs(f) > 0.0f) {
          rhs += J * f;
          lhs += outer(J, J * effectiveRho);
        }
      }
      if (disp > j.limitUpper || j.lambdaLimitUpper < 0.0f) {
        float viol = disp - j.limitUpper;
        Vec6 J(worldAxis * limitSign, r.cross(worldAxis) * limitSign);
        float f = effectiveRho * viol + j.lambdaLimitUpper;
        f = std::max(0.0f, f);
        if (std::fabs(f) > 0.0f) {
          rhs += J * f;
          lhs += outer(J, J * effectiveRho);
        }
      }
    }
  }

  // --- PD drive (soft AL constraint) ---
  if (j.hasDrive) {
    if (j.type == eARTIC_REVOLUTE) {
      float angle = artic.computeJointAngle(jointIdx, bodies);

      // Angular velocity from position-level delta
      Quat deltaQ = body.rotation * body.initialRotation.conjugate();
      if (deltaQ.w < 0) deltaQ = deltaQ * (-1.0f);
      Vec3 currentAngVel = Vec3(deltaQ.x, deltaQ.y, deltaQ.z) * (2.0f / dt);
      float currentAxisVel = currentAngVel.dot(worldAxis);

      // PD: stiffness * (target - angle) + damping * (targetVel - vel)
      float posError = j.driveTarget - angle;
      float velError = j.driveTargetVelocity - currentAxisVel;
      float driveTorque = j.driveStiffness * posError + j.driveDamping * velError;
      if (j.driveMaxForce > 0)
        driveTorque = std::max(-j.driveMaxForce,
                               std::min(j.driveMaxForce, driveTorque));

      // Convert torque to position-level correction via soft constraint
      float rho_drive = std::max(j.driveStiffness, j.driveDamping / dt) / dt2;
      if (rho_drive > 0.0f) {
        float correction = driveTorque * dt2 / body.mass;
        // Scale to something reasonable
        correction = std::max(-0.5f, std::min(0.5f, correction));

        Vec6 J(Vec3(), worldAxis * sign * (-1.0f));
        float f = rho_drive * (-correction);
        rhs += J * f;
        lhs += outer(J, J * rho_drive);
      }
    } else if (j.type == eARTIC_PRISMATIC) {
      float disp = artic.computeJointDisplacement(jointIdx, bodies);

      // Direct AL soft constraint: C_drive = disp - target
      // ∂disp/∂x_child = +axis, ∂disp/∂x_parent = -axis
      // So J for child = +axis, J for parent = -axis
      // In the bilateral convention: sign = -1 for child, +1 for parent
      // Thus J = axis * (-sign)
      float C_drive = disp - j.driveTarget;
      float rho_drive = j.driveStiffness / dt2;
      if (rho_drive > 0.0f) {
        Vec6 J(worldAxis * (-sign), r.cross(worldAxis) * (-sign));
        float f = rho_drive * C_drive + j.lambdaDrive;
        rhs += J * f;
        lhs += outer(J, J * rho_drive);
      }
    }
  }

  // --- Joint friction (viscous damping) ---
  if (j.frictionCoefficient > 0.0f) {
    if (j.type == eARTIC_REVOLUTE || j.type == eARTIC_SPHERICAL) {
      // Damping along the free angular DOF(s)
      Vec3 dwThis = body.deltaWInitial();
      float fricRho = j.frictionCoefficient / dt2;

      if (j.type == eARTIC_REVOLUTE) {
        // Single axis friction
        float vel = dwThis.dot(worldAxis);
        Vec6 J(Vec3(), worldAxis);
        float f = fricRho * vel;
        rhs += J * f;
        lhs += outer(J, J * fricRho);
      } else {
        // Spherical: all 3 axes
        for (int k = 0; k < 3; k++) {
          Vec3 ax(k == 0 ? 1.0f : 0.0f, k == 1 ? 1.0f : 0.0f,
                  k == 2 ? 1.0f : 0.0f);
          float vel = dwThis.dot(ax);
          Vec6 J(Vec3(), ax);
          float f = fricRho * vel;
          rhs += J * f;
          lhs += outer(J, J * fricRho);
        }
      }
    } else if (j.type == eARTIC_PRISMATIC) {
      // Linear damping along slide axis
      Vec3 dxThis = body.position - body.initialPosition;
      float vel = dxThis.dot(worldAxis);
      float fricRho = j.frictionCoefficient / dt2;
      Vec6 J(worldAxis, r.cross(worldAxis));
      float f = fricRho * vel;
      rhs += J * f;
      lhs += outer(J, J * fricRho);
    }
  }
}

// =============================================================================
// addMimicJointContribution — primal update for mimic joint constraint
//
// Mimic: q_A + gearRatio * q_B + offset = 0
// This is a bilateral constraint mapping joint DOFs of two links.
// =============================================================================
inline void addMimicJointContribution(const Articulation &artic,
                                      int mimicIdx, uint32_t bi,
                                      const std::vector<Body> &bodies,
                                      float dt, Mat66 &lhs, Vec6 &rhs) {
  const MimicJointDef &m = artic.mimicJoints[mimicIdx];
  if (m.jointA < 0 || m.jointB < 0) return;
  const ArticulationJoint &jA = artic.joints[m.jointA];
  const ArticulationJoint &jB = artic.joints[m.jointB];

  // Only act on the two involved bodies
  if (bi != jA.bodyIndex && bi != jB.bodyIndex) return;

  float dt2 = dt * dt;
  const Body &body = bodies[bi];
  float effectiveRho = std::max(m.rho, body.mass / dt2);

  // Compute current joint values
  float qA = 0, qB = 0;
  if (jA.type == eARTIC_REVOLUTE)
    qA = artic.computeJointAngle(m.jointA, bodies);
  else if (jA.type == eARTIC_PRISMATIC)
    qA = artic.computeJointDisplacement(m.jointA, bodies);

  if (jB.type == eARTIC_REVOLUTE)
    qB = artic.computeJointAngle(m.jointB, bodies);
  else if (jB.type == eARTIC_PRISMATIC)
    qB = artic.computeJointDisplacement(m.jointB, bodies);

  float C = qA + m.gearRatio * qB + m.offset;

  // World-space joint axes
  uint32_t parentA = artic.getParentBodyIndex(m.jointA);
  uint32_t parentB = artic.getParentBodyIndex(m.jointB);
  Quat rotParentA = (parentA == UINT32_MAX) ? Quat() : bodies[parentA].rotation;
  Quat rotParentB = (parentB == UINT32_MAX) ? Quat() : bodies[parentB].rotation;

  Vec3 worldAxisA = rotParentA.rotate(jA.axis);
  Vec3 worldAxisB = rotParentB.rotate(jB.axis);

  // Build Jacobian for the body we're updating
  float f = effectiveRho * C + m.lambda;

  if (bi == jA.bodyIndex) {
    Vec3 rA = body.rotation.rotate(jA.childAnchor);
    if (jA.type == eARTIC_REVOLUTE) {
      Vec6 J(Vec3(), worldAxisA * (-1.0f));
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    } else if (jA.type == eARTIC_PRISMATIC) {
      Vec6 J(worldAxisA * (-1.0f), rA.cross(worldAxisA) * (-1.0f));
      rhs += J * f;
      lhs += outer(J, J * effectiveRho);
    }
  }
  if (bi == jB.bodyIndex) {
    Vec3 rB = body.rotation.rotate(jB.childAnchor);
    float scaledF = f * m.gearRatio;
    if (jB.type == eARTIC_REVOLUTE) {
      Vec6 J(Vec3(), worldAxisB * (-1.0f));
      rhs += J * scaledF;
      lhs += outer(J, J * (effectiveRho * m.gearRatio * m.gearRatio));
    } else if (jB.type == eARTIC_PRISMATIC) {
      Vec6 J(worldAxisB * (-1.0f), rB.cross(worldAxisB) * (-1.0f));
      rhs += J * scaledF;
      lhs += outer(J, J * (effectiveRho * m.gearRatio * m.gearRatio));
    }
  }
}

// =============================================================================
// addIKTargetContribution — primal update for end-effector IK constraint
//
// Constrains a world-space point on a link to match a target position.
// This is a 3-row bilateral positional AL constraint.
// =============================================================================
inline void addIKTargetContribution(const Articulation &artic,
                                    int targetIdx, uint32_t bi,
                                    const std::vector<Body> &bodies,
                                    float dt, Mat66 &lhs, Vec6 &rhs) {
  const EndEffectorTarget &t = artic.ikTargets[targetIdx];
  if (t.jointIdx < 0) return;
  const ArticulationJoint &j = artic.joints[t.jointIdx];
  if (bi != j.bodyIndex) return;

  float dt2 = dt * dt;
  const Body &body = bodies[bi];
  float effectiveRho = std::max(t.rho, body.mass / dt2);

  Vec3 r = body.rotation.rotate(t.localPoint);
  Vec3 worldPoint = body.position + r;
  Vec3 C = t.targetPos - worldPoint;

  Vec3 axes[3] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
  for (int k = 0; k < 3; k++) {
    float Ck = C.dot(axes[k]);
    // Jacobian: moving body in +axis moves worldPoint in +axis, decreasing C
    // So sign is -1 for the body (similar to bilateral with "target - point")
    Vec6 J(axes[k] * (-1.0f), r.cross(axes[k]) * (-1.0f));
    float lam = (&t.lambda.x)[k];
    float f = effectiveRho * Ck + lam;
    rhs += J * f;
    lhs += outer(J, J * effectiveRho);
  }
}

// =============================================================================
// updateArticulationDual — dual update (lambda update, AL)
// =============================================================================
inline void updateArticulationDual(Articulation &artic, int jointIdx,
                                   const std::vector<Body> &bodies,
                                   float dt, float lambdaDecay) {
  ArticulationJoint &j = artic.joints[jointIdx];
  uint32_t parentBody = artic.getParentBodyIndex(jointIdx);
  uint32_t childBody = j.bodyIndex;
  bool parentStatic = (parentBody == UINT32_MAX);
  float dt2 = dt * dt;

  // Compute rhoDual (ADMM-safe)
  float mChild = bodies[childBody].mass;
  float mParent = parentStatic ? 0.0f : bodies[parentBody].mass;
  float mEff = (mParent <= 0.0f) ? mChild
             : (mChild <= 0.0f)  ? mParent
                                 : std::min(mParent, mChild);
  float Mh2 = mEff / dt2;
  float admm_step = j.rho * j.rho / (j.rho + Mh2);
  float rhoDual = std::min(Mh2, admm_step);

  // World-space anchors
  Vec3 worldParentAnchor, worldChildAnchor;
  if (parentStatic) {
    worldParentAnchor = artic.fixedBasePos + j.parentAnchor;
  } else {
    worldParentAnchor = bodies[parentBody].position +
                        bodies[parentBody].rotation.rotate(j.parentAnchor);
  }
  worldChildAnchor = bodies[childBody].position +
                     bodies[childBody].rotation.rotate(j.childAnchor);
  Vec3 C_lin = worldParentAnchor - worldChildAnchor;

  Quat rotParent = parentStatic ? Quat() : bodies[parentBody].rotation;
  Quat rotChild = bodies[childBody].rotation;

  Quat worldFrameParent = parentStatic ? j.localFrameParent
                                       : rotParent * j.localFrameParent;
  worldFrameParent = worldFrameParent.normalized();
  Quat worldFrameChild = rotChild * j.localFrameChild;
  worldFrameChild = worldFrameChild.normalized();

  Vec3 worldAxis = worldFrameParent.rotate(Vec3(1, 0, 0));

  // --- Positional dual ---
  switch (j.type) {
  case eARTIC_REVOLUTE:
  case eARTIC_FIXED:
  case eARTIC_SPHERICAL: {
    Vec3 axes[3] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
    for (int k = 0; k < 3; k++) {
      float Ck = C_lin.dot(axes[k]);
      (&j.lambdaPos.x)[k] = (&j.lambdaPos.x)[k] * lambdaDecay + Ck * rhoDual;
    }
    break;
  }
  case eARTIC_PRISMATIC: {
    Vec3 perp1 = worldFrameParent.rotate(Vec3(0, 1, 0));
    Vec3 perp2 = worldFrameParent.rotate(Vec3(0, 0, 1));
    (&j.lambdaPos.x)[0] = j.lambdaPos.x * lambdaDecay + C_lin.dot(perp1) * rhoDual;
    (&j.lambdaPos.x)[1] = j.lambdaPos.y * lambdaDecay + C_lin.dot(perp2) * rhoDual;
    break;
  }
  }

  // --- Angular dual ---
  switch (j.type) {
  case eARTIC_FIXED:
  case eARTIC_PRISMATIC: {
    for (int k = 0; k < 3; k++) {
      float angErr = computeAngularError(rotParent, rotChild,
                                         j.localFrameParent, j.localFrameChild, k);
      (&j.lambdaAng.x)[k] = (&j.lambdaAng.x)[k] * lambdaDecay + angErr * rhoDual;
    }
    break;
  }
  case eARTIC_REVOLUTE: {
    // Cross-product axis alignment
    Vec3 worldTwistParent = worldFrameParent.rotate(Vec3(1, 0, 0));
    Vec3 worldTwistChild = worldFrameChild.rotate(Vec3(1, 0, 0));
    Vec3 axisViol = worldTwistParent.cross(worldTwistChild);

    Vec3 perp1, perp2;
    if (std::fabs(worldTwistParent.x) < 0.9f)
      perp1 = worldTwistParent.cross(Vec3(1, 0, 0));
    else
      perp1 = worldTwistParent.cross(Vec3(0, 1, 0));
    float p1Len = perp1.length();
    if (p1Len > 1e-6f) perp1 = perp1 * (1.0f / p1Len);
    perp2 = worldTwistParent.cross(perp1);
    float p2Len = perp2.length();
    if (p2Len > 1e-6f) perp2 = perp2 * (1.0f / p2Len);

    j.lambdaAng.y = j.lambdaAng.y * lambdaDecay + axisViol.dot(perp1) * rhoDual;
    j.lambdaAng.z = j.lambdaAng.z * lambdaDecay + axisViol.dot(perp2) * rhoDual;
    break;
  }
  case eARTIC_SPHERICAL:
    // No angular constraints to update
    break;
  }

  // --- Joint limit dual ---
  if (j.hasLimits) {
    if (j.type == eARTIC_REVOLUTE) {
      float angle = artic.computeJointAngle(jointIdx, bodies);
      // Lower limit
      {
        float viol = angle - j.limitLower;
        float newLam = j.lambdaLimitLower * lambdaDecay + viol * rhoDual;
        j.lambdaLimitLower = std::min(0.0f, newLam); // must be ≤ 0
      }
      // Upper limit
      {
        float viol = angle - j.limitUpper;
        float newLam = j.lambdaLimitUpper * lambdaDecay + viol * rhoDual;
        j.lambdaLimitUpper = std::max(0.0f, newLam); // must be ≥ 0
      }
    } else if (j.type == eARTIC_PRISMATIC) {
      float disp = artic.computeJointDisplacement(jointIdx, bodies);
      {
        float viol = disp - j.limitLower;
        float newLam = j.lambdaLimitLower * lambdaDecay + viol * rhoDual;
        j.lambdaLimitLower = std::min(0.0f, newLam);
      }
      {
        float viol = disp - j.limitUpper;
        float newLam = j.lambdaLimitUpper * lambdaDecay + viol * rhoDual;
        j.lambdaLimitUpper = std::max(0.0f, newLam);
      }
    }
  }

  // --- Prismatic drive dual ---
  if (j.hasDrive && j.type == eARTIC_PRISMATIC) {
    float disp = artic.computeJointDisplacement(jointIdx, bodies);
    float C_drive = disp - j.driveTarget;
    float rho_drive_dual = std::min(Mh2, j.driveStiffness / dt2);
    j.lambdaDrive = j.lambdaDrive * lambdaDecay + C_drive * rho_drive_dual;
  }
}

// =============================================================================
// updateMimicDual — dual update for mimic joint
// =============================================================================
inline void updateMimicDual(Articulation &artic, int mimicIdx,
                            const std::vector<Body> &bodies,
                            float dt, float lambdaDecay) {
  MimicJointDef &m = artic.mimicJoints[mimicIdx];
  if (m.jointA < 0 || m.jointB < 0) return;
  const ArticulationJoint &jA = artic.joints[m.jointA];
  const ArticulationJoint &jB = artic.joints[m.jointB];
  float dt2 = dt * dt;

  float mA = bodies[jA.bodyIndex].mass;
  float mB = bodies[jB.bodyIndex].mass;
  float mEff = std::min(mA, mB);
  float Mh2 = mEff / dt2;
  float admm_step = m.rho * m.rho / (m.rho + Mh2);
  float rhoDual = std::min(Mh2, admm_step);

  float qA = 0, qB = 0;
  if (jA.type == eARTIC_REVOLUTE)
    qA = artic.computeJointAngle(m.jointA, bodies);
  else if (jA.type == eARTIC_PRISMATIC)
    qA = artic.computeJointDisplacement(m.jointA, bodies);
  if (jB.type == eARTIC_REVOLUTE)
    qB = artic.computeJointAngle(m.jointB, bodies);
  else if (jB.type == eARTIC_PRISMATIC)
    qB = artic.computeJointDisplacement(m.jointB, bodies);

  float C = qA + m.gearRatio * qB + m.offset;
  m.lambda = m.lambda * lambdaDecay + C * rhoDual;
}

// =============================================================================
// updateIKTargetDual — dual update for end-effector IK target
// =============================================================================
inline void updateIKTargetDual(Articulation &artic, int targetIdx,
                               const std::vector<Body> &bodies,
                               float dt, float lambdaDecay) {
  EndEffectorTarget &t = artic.ikTargets[targetIdx];
  if (t.jointIdx < 0) return;
  const ArticulationJoint &j = artic.joints[t.jointIdx];
  float dt2 = dt * dt;

  float mEff = bodies[j.bodyIndex].mass;
  float Mh2 = mEff / dt2;
  float admm_step = t.rho * t.rho / (t.rho + Mh2);
  float rhoDual = std::min(Mh2, admm_step);

  Vec3 r = bodies[j.bodyIndex].rotation.rotate(t.localPoint);
  Vec3 worldPoint = bodies[j.bodyIndex].position + r;
  Vec3 C = t.targetPos - worldPoint;

  t.lambda.x = t.lambda.x * lambdaDecay + C.x * rhoDual;
  t.lambda.y = t.lambda.y * lambdaDecay + C.y * rhoDual;
  t.lambda.z = t.lambda.z * lambdaDecay + C.z * rhoDual;
}

} // namespace AvbdRef
