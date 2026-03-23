#include "avbd_solver.h"
#include "avbd_articulation.h"
#include "avbd_d6_core.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace AvbdRef {

namespace {

inline bool shouldSortConstraints(const Solver &solver) {
  return solver.runtimeConfig().hasDeterminismFlag(DeterminismFlags::eSORT_CONSTRAINTS);
}

inline bool shouldSortBodies(const Solver &solver) {
  return solver.runtimeConfig().hasDeterminismFlag(DeterminismFlags::eSORT_BODIES);
}

inline bool shouldSortIslands(const Solver &solver) {
  return solver.runtimeConfig().hasDeterminismFlag(DeterminismFlags::eSORT_ISLANDS);
}

inline void syncBodyStateToAoS(Solver &solver, uint32_t bodyIndex) {
  if (bodyIndex < solver.bodies.size())
    solver.storage.bodies.scatterBodyToAoS(solver.bodies, bodyIndex);
}

inline void syncAllBodyStateToAoS(Solver &solver) {
  solver.storage.bodies.scatterToBodies(solver.bodies);
}

inline void syncContactsToAoS(Solver &solver) {
  solver.storage.contacts.scatterToContacts(solver.contacts);
}

} // namespace

// =============================================================================
// Factory methods  all create D6Joint entries in the unified d6Joints vector
// =============================================================================

uint32_t Solver::addSphericalJoint(uint32_t bodyA, uint32_t bodyB,
                                   Vec3 anchorA, Vec3 anchorB, float rho_) {
  D6Joint j;
  j.bodyA = bodyA;
  j.bodyB = bodyB;
  j.anchorA = anchorA;
  j.anchorB = anchorB;
  j.linearMotion = 0;     // all 3 linear LOCKED
  j.angularMotion = 0x2A; // all 3 angular FREE (2|(2<<2)|(2<<4))
  j.rho = rho_;

  // Compute localFrameB from initial relative rotation (matches PhysX)
  Quat rotA = (bodyA == UINT32_MAX) ? Quat() : bodies[bodyA].rotation;
  Quat rotB = (bodyB == UINT32_MAX) ? Quat() : bodies[bodyB].rotation;
  j.relativeRotation = rotA.conjugate() * rotB;
  j.localFrameB = j.relativeRotation.conjugate() * j.localFrameA;

  uint32_t idx = (uint32_t)d6Joints.size();
  d6Joints.push_back(j);
  return idx;
}

void Solver::setSphericalJointConeLimit(uint32_t jointIdx, Vec3 coneAxisA,
                                        float limitAngle) {
  if (jointIdx < d6Joints.size()) {
    d6Joints[jointIdx].coneAngleLimit = limitAngle;
    d6Joints[jointIdx].coneAxisA = coneAxisA;
    d6Joints[jointIdx].coneLambda = 0.0f;

    // Build localFrameA so X-axis = cone axis (matches PhysX joint frame)
    Vec3 axisNorm = coneAxisA.normalized();
    Vec3 perp;
    if (std::fabs(axisNorm.x) < 0.9f)
      perp = axisNorm.cross(Vec3(1, 0, 0)).normalized();
    else
      perp = axisNorm.cross(Vec3(0, 1, 0)).normalized();
    Vec3 perp2 = axisNorm.cross(perp);
    d6Joints[jointIdx].localFrameA = quatFromColumns(axisNorm, perp, perp2);
    d6Joints[jointIdx].localFrameB =
        d6Joints[jointIdx].relativeRotation.conjugate() *
        d6Joints[jointIdx].localFrameA;
  }
}

uint32_t Solver::addFixedJoint(uint32_t bodyA, uint32_t bodyB,
                               Vec3 anchorA, Vec3 anchorB, float rho_) {
  D6Joint j;
  j.bodyA = bodyA;
  j.bodyB = bodyB;
  j.anchorA = anchorA;
  j.anchorB = anchorB;
  j.linearMotion = 0;  // all 3 linear LOCKED
  j.angularMotion = 0; // all 3 angular LOCKED
  j.rho = rho_;
  // Compute initial relative rotation and localFrameB
  Quat rotA = (bodyA == UINT32_MAX) ? Quat() : bodies[bodyA].rotation;
  Quat rotB = (bodyB == UINT32_MAX) ? Quat() : bodies[bodyB].rotation;
  j.relativeRotation = rotA.conjugate() * rotB;
  j.localFrameB = j.relativeRotation.conjugate() * j.localFrameA;
  uint32_t idx = (uint32_t)d6Joints.size();
  d6Joints.push_back(j);
  return idx;
}

uint32_t Solver::addD6Joint(uint32_t bodyA, uint32_t bodyB,
                            Vec3 anchorA, Vec3 anchorB,
                            uint32_t linearMotion_, uint32_t angularMotion_,
                            float angularDamping_, float rho_) {
  D6Joint j;
  j.bodyA = bodyA;
  j.bodyB = bodyB;
  j.anchorA = anchorA;
  j.anchorB = anchorB;
  j.linearMotion = linearMotion_;
  j.angularMotion = angularMotion_;
  j.rho = rho_;
  j.angularDriveDamping =
      Vec3(angularDamping_, angularDamping_, angularDamping_);

  // Compute relativeRotation and localFrameB (matches PhysX)
  Quat rotA = (bodyA == UINT32_MAX) ? Quat() : bodies[bodyA].rotation;
  Quat rotB = (bodyB == UINT32_MAX) ? Quat() : bodies[bodyB].rotation;
  j.relativeRotation = rotA.conjugate() * rotB;
  j.localFrameB = j.relativeRotation.conjugate() * j.localFrameA;

  uint32_t idx = (uint32_t)d6Joints.size();
  d6Joints.push_back(j);
  return idx;
}

uint32_t Solver::addRevoluteJoint(uint32_t bodyA, uint32_t bodyB,
                                  Vec3 localAnchorA, Vec3 localAnchorB,
                                  Vec3 localAxisA, Vec3 localAxisB,
                                  float rho_) {
  D6Joint j;
  j.bodyA = bodyA;
  j.bodyB = bodyB;
  j.anchorA = localAnchorA;
  j.anchorB = localAnchorB;
  j.linearMotion = 0; // all 3 linear LOCKED

  // Angular: X(twist) = FREE, Y(swing1) = LOCKED, Z(swing2) = LOCKED
  // Hinge axis maps to joint-frame X
  j.angularMotion = 2; // 2|(0<<2)|(0<<4) = axis0=FREE, axis1=LOCKED, axis2=LOCKED

  j.rho = rho_;

  // Build reference axes perpendicular to hinge axis
  Vec3 axisA = localAxisA.normalized();
  Vec3 axisB = localAxisB.normalized();

  auto buildRefAxis = [](Vec3 axis) -> Vec3 {
    Vec3 perp;
    if (fabsf(axis.x) < 0.9f)
      perp = axis.cross(Vec3(1, 0, 0)).normalized();
    else
      perp = axis.cross(Vec3(0, 1, 0)).normalized();
    return perp;
  };

  Vec3 refA = buildRefAxis(axisA);
  Vec3 z_axisA = axisA.cross(refA);

  // localFrameA: X=hingeAxis, Y=refAxisA, Z=cross
  j.localFrameA = quatFromColumns(axisA, refA, z_axisA);

  // Store relative rotation for angular error computation
  Quat rotA = (bodyA == UINT32_MAX) ? Quat() : bodies[bodyA].rotation;
  Quat rotB = (bodyB == UINT32_MAX) ? Quat() : bodies[bodyB].rotation;
  j.relativeRotation = rotA.conjugate() * rotB;

  // Store revolute-specific fields for hinge angle measurement
  j.hingeAxisB = axisB;
  j.refAxisA = refA;

  // Compute refAxisB: project worldRefA onto B hinge plane, transform to B local
  Vec3 worldRefA = rotA.rotate(refA);
  Vec3 worldAxisB = rotB.rotate(axisB);
  Vec3 proj = worldRefA - worldAxisB * worldRefA.dot(worldAxisB);
  float projLen = proj.length();
  if (projLen > 1e-8f) {
    j.refAxisB = rotB.conjugate().rotate(proj * (1.0f / projLen));
  } else {
    j.refAxisB = buildRefAxis(axisB);
  }

  // Compute localFrameB from relativeRotation (matches PhysX convention)
  j.localFrameB = j.relativeRotation.conjugate() * j.localFrameA;

  uint32_t idx = (uint32_t)d6Joints.size();
  d6Joints.push_back(j);
  return idx;
}

void Solver::setRevoluteJointLimit(uint32_t jointIdx, float lowerLimit,
                                   float upperLimit) {
  if (jointIdx < d6Joints.size()) {
    // Change angular axis 0 from FREE to LIMITED
    d6Joints[jointIdx].angularMotion =
        (d6Joints[jointIdx].angularMotion & ~0x3) | 1; // axis 0 = LIMITED
    d6Joints[jointIdx].angularLimitLower[0] = lowerLimit;
    d6Joints[jointIdx].angularLimitUpper[0] = upperLimit;
    d6Joints[jointIdx].lambdaLimitAngular[0] = 0.0f;
  }
}

void Solver::setRevoluteJointDrive(uint32_t jointIdx, float targetVelocity,
                                   float maxForce) {
  if (jointIdx < d6Joints.size()) {
    // Use post-solve motor (matches PhysX) instead of AL velocity drive.
    // This avoids ADMM oscillation when coupled with gear constraints.
    d6Joints[jointIdx].motorEnabled = true;
    d6Joints[jointIdx].motorTargetVelocity = targetVelocity;
    d6Joints[jointIdx].motorMaxForce = maxForce;
  }
}

uint32_t Solver::addPrismaticJoint(uint32_t bodyA, uint32_t bodyB,
                                   Vec3 localAnchorA, Vec3 localAnchorB,
                                   Vec3 localAxisA, float rho_) {
  D6Joint j;
  j.bodyA = bodyA;
  j.bodyB = bodyB;
  j.anchorA = localAnchorA;
  j.anchorB = localAnchorB;
  j.angularMotion = 0; // all 3 angular LOCKED

  // Linear: X(slide) = FREE, Y = LOCKED, Z = LOCKED
  j.linearMotion = 2; // axis0=FREE, axis1=LOCKED, axis2=LOCKED

  j.rho = rho_;

  Vec3 axisA = localAxisA.normalized();
  Vec3 helper =
      (std::abs(axisA.x) > 0.9f) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
  Vec3 t1 = axisA.cross(helper).normalized();
  Vec3 t2 = axisA.cross(t1);
  j.localFrameA = quatFromColumns(axisA, t1, t2);

  // Store relative rotation and compute localFrameB
  Quat rotA = (bodyA == UINT32_MAX) ? Quat() : bodies[bodyA].rotation;
  Quat rotB = (bodyB == UINT32_MAX) ? Quat() : bodies[bodyB].rotation;
  j.relativeRotation = rotA.conjugate() * rotB;
  j.localFrameB = j.relativeRotation.conjugate() * j.localFrameA;

  uint32_t idx = (uint32_t)d6Joints.size();
  d6Joints.push_back(j);
  return idx;
}

void Solver::setPrismaticJointLimit(uint32_t jointIdx, float lowerLimit,
                                    float upperLimit) {
  if (jointIdx < d6Joints.size()) {
    // Change linear axis 0 from FREE to LIMITED
    d6Joints[jointIdx].linearMotion =
        (d6Joints[jointIdx].linearMotion & ~0x3) | 1; // axis 0 = LIMITED
    d6Joints[jointIdx].linearLimitLower[0] = lowerLimit;
    d6Joints[jointIdx].linearLimitUpper[0] = upperLimit;
    d6Joints[jointIdx].lambdaLimitLinear[0] = 0.0f;
  }
}

void Solver::setPrismaticJointDrive(uint32_t jointIdx, float targetVelocity,
                                    float damping) {
  if (jointIdx < d6Joints.size()) {
    d6Joints[jointIdx].driveFlags |= 0x01; // linear X drive
    d6Joints[jointIdx].driveLinearVelocity = Vec3(targetVelocity, 0, 0);
    d6Joints[jointIdx].linearDriveDamping.x = damping;
    d6Joints[jointIdx].lambdaDriveLinear = Vec3();
  }
}

void Solver::addGearJoint(uint32_t bodyA, uint32_t bodyB,
                          Vec3 axisA, Vec3 axisB,
                          float ratio, float rho_) {
  GearJoint j;
  j.bodyA = bodyA;
  j.bodyB = bodyB;
  j.axisA = axisA.normalized();
  j.axisB = axisB.normalized();
  j.gearRatio = ratio;
  j.rho = rho_;
  gearJoints.push_back(j);
}

// =============================================================================
// Body / Contact creation
// =============================================================================

uint32_t Solver::addBody(Vec3 pos, Quat rot, Vec3 halfExtent, float density,
                         float fric) {
  Body b;
  b.position = pos;
  b.rotation = rot;
  b.linearVelocity = {};
  b.angularVelocity = {};
  b.prevLinearVelocity = {};
  b.friction = fric;
  b.halfExtent = halfExtent;

  float vol = 8.0f * halfExtent.x * halfExtent.y * halfExtent.z;
  if (density > 0) {
    b.mass = vol * density;
    float sx = 2 * halfExtent.x, sy = 2 * halfExtent.y, sz = 2 * halfExtent.z;
    float Ixx = b.mass / 12.0f * (sy * sy + sz * sz);
    float Iyy = b.mass / 12.0f * (sx * sx + sz * sz);
    float Izz = b.mass / 12.0f * (sx * sx + sy * sy);
    b.inertiaTensor = Mat33::diag(Ixx, Iyy, Izz);
  } else {
    b.mass = 0;
    b.inertiaTensor = Mat33::diag(0, 0, 0);
  }
  b.computeDerived();

  uint32_t idx = (uint32_t)bodies.size();
  bodies.push_back(b);
  sleepSystem.resize(bodies.size());
  sleepSystem.wake(idx);
  return idx;
}

void Solver::addContact(uint32_t bodyA, uint32_t bodyB, Vec3 normal, Vec3 rA,
                        Vec3 rB, float depth, float fric) {
  if (bodyA < bodies.size())
    sleepSystem.wake(bodyA);
  if (bodyB < bodies.size())
    sleepSystem.wake(bodyB);
  ContactPrep::ContactMaterial material;
  material.dynamicFriction = fric;
  const float separation = -depth;
  addContact(ContactPrep::prepareRow(bodyA, bodyB, Vec3(), normal, separation,
                                     rA, rB, material, nullptr,
                                     bodyB == UINT32_MAX));
}

void Solver::addContact(const ContactPrep::ContactRow &row) {
  contacts.push_back(ContactPrep::toSolverContact(row));
}

// =============================================================================
// Contact constraint computation
// =============================================================================

void Solver::computeConstraint(Contact &c) {
  if (!storage.contacts.bodyA.empty()) {
    uint32_t ci = (uint32_t)(&c - contacts.data());
    auto &bodyState = storage.bodies;
    auto &contactState = storage.contacts;
    uint32_t bodyA = contactState.bodyA[ci];
    bool bStatic = (contactState.bodyB[ci] == UINT32_MAX);
    uint32_t bodyB = contactState.bodyB[ci];

    Vec3 rAw = bodyState.rotation[bodyA].rotate(contactState.anchorA[ci]);
    Vec3 rBw = bStatic ? Vec3() : bodyState.rotation[bodyB].rotate(contactState.anchorB[ci]);
    Vec3 n = contactState.normal[ci];
    Vec3 t1 = contactState.tangent0[ci];
    Vec3 t2 = contactState.tangent1[ci];

    c.JA = Vec6(n, rAw.cross(n));
    c.JB = bStatic ? Vec6() : Vec6(Vec3() - n, Vec3() - rBw.cross(n));
    c.JAt1 = Vec6(t1, rAw.cross(t1));
    c.JBt1 = bStatic ? Vec6() : Vec6(Vec3() - t1, Vec3() - rBw.cross(t1));
    c.JAt2 = Vec6(t2, rAw.cross(t2));
    c.JBt2 = bStatic ? Vec6() : Vec6(Vec3() - t2, Vec3() - rBw.cross(t2));

    Vec6 dpA(bodyState.position[bodyA] - bodyState.initialPosition[bodyA],
             bodyState.deltaWInitial(bodyA));
    Vec6 dpB;
    if (!bStatic)
      dpB = Vec6(bodyState.position[bodyB] - bodyState.initialPosition[bodyB],
                 bodyState.deltaWInitial(bodyB));

    contactState.C[ci][0] = contactState.C0[ci][0] * (1.0f - alpha) + dot(c.JA, dpA) + dot(c.JB, dpB);
    contactState.C[ci][1] = contactState.C0[ci][1] * (1.0f - alpha) + dot(c.JAt1, dpA) + dot(c.JBt1, dpB);
    contactState.C[ci][2] = contactState.C0[ci][2] * (1.0f - alpha) + dot(c.JAt2, dpA) + dot(c.JBt2, dpB);
    for (int i = 0; i < 3; ++i)
      c.C[i] = contactState.C[ci][i];
    float frictionBound = fabsf(contactState.lambda[ci][0]) * contactState.friction[ci];
    contactState.fmax[ci][1] = frictionBound;
    contactState.fmin[ci][1] = -frictionBound;
    contactState.fmax[ci][2] = frictionBound;
    contactState.fmin[ci][2] = -frictionBound;
    for (int i = 0; i < 3; ++i) {
      c.fmin[i] = contactState.fmin[ci][i];
      c.fmax[i] = contactState.fmax[ci][i];
    }
    return;
  }
  Body &bA = bodies[c.bodyA];
  bool bStatic = (c.bodyB == UINT32_MAX);
  Body *pB = bStatic ? nullptr : &bodies[c.bodyB];

  Vec3 rAw = bA.rotation.rotate(c.rA);
  Vec3 rBw = bStatic ? Vec3() : pB->rotation.rotate(c.rB);

  c.JA = Vec6(c.normal, rAw.cross(c.normal));
  c.JB = bStatic ? Vec6() : Vec6(Vec3() - c.normal, Vec3() - rBw.cross(c.normal));

  Vec3 t1, t2;
  if (fabsf(c.normal.y) > 0.9f)
    t1 = c.normal.cross(Vec3(1, 0, 0)).normalized();
  else
    t1 = c.normal.cross(Vec3(0, 1, 0)).normalized();
  t2 = c.normal.cross(t1);

  c.JAt1 = Vec6(t1, rAw.cross(t1));
  c.JBt1 = bStatic ? Vec6() : Vec6(Vec3() - t1, Vec3() - rBw.cross(t1));
  c.JAt2 = Vec6(t2, rAw.cross(t2));
  c.JBt2 = bStatic ? Vec6() : Vec6(Vec3() - t2, Vec3() - rBw.cross(t2));

  Vec6 dpA(bA.position - bA.initialPosition, bA.deltaWInitial());
  Vec6 dpB;
  if (!bStatic)
    dpB = Vec6(pB->position - pB->initialPosition, pB->deltaWInitial());

  c.C[0] = c.C0[0] * (1.0f - alpha) + dot(c.JA, dpA) + dot(c.JB, dpB);
  c.C[1] = c.C0[1] * (1.0f - alpha) + dot(c.JAt1, dpA) + dot(c.JBt1, dpB);
  c.C[2] = c.C0[2] * (1.0f - alpha) + dot(c.JAt2, dpA) + dot(c.JBt2, dpB);

  float frictionBound = fabsf(c.lambda[0]) * c.friction;
  c.fmax[1] = frictionBound;
  c.fmin[1] = -frictionBound;
  c.fmax[2] = frictionBound;
  c.fmin[2] = -frictionBound;
}

void Solver::computeC0(Contact &c) {
  if (!storage.contacts.bodyA.empty()) {
    uint32_t ci = (uint32_t)(&c - contacts.data());
    auto &bodyState = storage.bodies;
    auto &contactState = storage.contacts;
    uint32_t bodyA = contactState.bodyA[ci];
    bool bStatic = (contactState.bodyB[ci] == UINT32_MAX);
    uint32_t bodyB = contactState.bodyB[ci];

    Vec3 wA = bodyState.position[bodyA] + bodyState.rotation[bodyA].rotate(contactState.anchorA[ci]);
    Vec3 wB = bStatic ? contactState.anchorB[ci]
                      : (bodyState.position[bodyB] + bodyState.rotation[bodyB].rotate(contactState.anchorB[ci]));
    float rawC0 = (wA - wB).dot(contactState.normal[ci]) - contactState.depth[ci];
    const float c0Threshold = 0.05f;
    const float c0MaxDepth  = 0.20f;
    if (rawC0 < -c0Threshold) {
      float t = std::clamp((c0MaxDepth + rawC0) / (c0MaxDepth - c0Threshold), 0.0f, 1.0f);
      rawC0 *= t;
    }
    contactState.C0[ci][0] = rawC0;
    contactState.C0[ci][1] = 0.0f;
    contactState.C0[ci][2] = 0.0f;
    for (int i = 0; i < 3; ++i)
      c.C0[i] = contactState.C0[ci][i];
    return;
  }
  Body &bA = bodies[c.bodyA];
  bool bStatic = (c.bodyB == UINT32_MAX);
  Body *pB = bStatic ? nullptr : &bodies[c.bodyB];

  Vec3 wA = bA.position + bA.rotation.rotate(c.rA);
  Vec3 wB = bStatic ? c.rB : (pB->position + pB->rotation.rotate(c.rB));

  float rawC0 = (wA - wB).dot(c.normal) - c.depth;

  // Depth-adaptive C0 clamping: for deep penetrations (fast impacts),
  // reduce C0 so that alpha blending does not over-soften the correction.
  // Shallow contacts keep C0 unchanged; deep ones fade C0 toward zero.
  const float c0Threshold = 0.05f;  // 50 mm: only trigger on fast impacts
  const float c0MaxDepth  = 0.20f;  // 200 mm: full fade-out
  if (rawC0 < -c0Threshold) {
    float t = std::clamp(
        (c0MaxDepth + rawC0) / (c0MaxDepth - c0Threshold), 0.0f, 1.0f);
    rawC0 *= t;
  }

  c.C0[0] = rawC0;
  c.C0[1] = 0.0f;
  c.C0[2] = 0.0f;
}


void Solver::warmstart() {
  tasks().parallelFor({0u, (uint32_t)contacts.size()}, [&](TaskRange range) {
    for (uint32_t ci = range.begin; ci < range.end; ++ci) {
      auto &c = contacts[ci];
      for (int i = 0; i < 3; i++) {
        storage.contacts.lambda[ci][i] = storage.contacts.lambda[ci][i] * alpha * gamma;
        storage.contacts.penalty[ci][i] =
            std::max(PENALTY_MIN, std::min(PENALTY_MAX, storage.contacts.penalty[ci][i] * gamma));
        c.lambda[i] = storage.contacts.lambda[ci][i];
        c.penalty[i] = storage.contacts.penalty[ci][i];
      }
    }
  });
}

void Solver::stagePredictionAndInertia(float invDt, float dt2) {
  tasks().parallelFor({0u, (uint32_t)bodies.size()}, [&](TaskRange range) {
    for (uint32_t bi = range.begin; bi < range.end; ++bi) {
      auto &body = bodies[bi];
      if (storage.bodies.mass[bi] <= 0)
        continue;
      storage.bodies.updateInvInertiaWorld(bi);
      storage.bodies.inertialPosition[bi] =
          storage.bodies.position[bi] + storage.bodies.linearVelocity[bi] * dt + gravity * dt2;
      Quat angVel(0, storage.bodies.angularVelocity[bi].x, storage.bodies.angularVelocity[bi].y,
                  storage.bodies.angularVelocity[bi].z);
      storage.bodies.inertialRotation[bi] =
          (storage.bodies.rotation[bi] + angVel * storage.bodies.rotation[bi] * (0.5f * dt)).normalized();

      Vec3 accel = (storage.bodies.linearVelocity[bi] - storage.bodies.prevLinearVelocity[bi]) * invDt;
      float gravLen = gravity.length();
      float accelWeight = 0.0f;
      if (gravLen > 1e-6f) {
        Vec3 gravDir = gravity.normalized();
        accelWeight = std::max(0.0f, std::min(1.0f, accel.dot(gravDir) / gravLen));
      }

      storage.bodies.initialPosition[bi] = storage.bodies.position[bi];
      storage.bodies.initialRotation[bi] = storage.bodies.rotation[bi];
      storage.bodies.position[bi] = storage.bodies.position[bi] + storage.bodies.linearVelocity[bi] * dt +
                      gravity * (accelWeight * dt2);
      storage.bodies.rotation[bi] = storage.bodies.inertialRotation[bi];
      syncBodyStateToAoS(*this, bi);
    }
  });
}

void Solver::stageBuildAdjacency() {
  const uint32_t nBodies = (uint32_t)bodies.size();
  pipeline.adjacency.assign(nBodies, {});
  auto addEdge = [&](uint32_t a, uint32_t b) {
    if (a < nBodies && b < nBodies && a != UINT32_MAX && b != UINT32_MAX) {
      pipeline.adjacency[a].push_back(b);
      pipeline.adjacency[b].push_back(a);
    }
  };
  for (const auto &j : d6Joints)
    addEdge(j.bodyA, j.bodyB);
  for (const auto &j : gearJoints)
    addEdge(j.bodyA, j.bodyB);
  for (const auto &c : contacts)
    addEdge(c.bodyA, c.bodyB);
  for (const auto &artic : articulations) {
    for (int ji = 0; ji < (int)artic.joints.size(); ++ji) {
      uint32_t child = artic.joints[ji].bodyIndex;
      uint32_t parent = artic.getParentBodyIndex(ji);
      addEdge(parent, child);
    }
  }
  if (shouldSortBodies(*this)) {
    for (auto &neighbors : pipeline.adjacency) {
      std::sort(neighbors.begin(), neighbors.end());
      neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }
  }
}

void Solver::stagePropagateMass(float dt2) {
  const uint32_t nBodies = (uint32_t)bodies.size();
  pipeline.propagatedMass.resize(nBodies);
  for (uint32_t i = 0; i < nBodies; ++i)
    pipeline.propagatedMass[i] = bodies[i].mass;

  for (int d = 0; d < propagationDepth; ++d) {
    std::vector<float> next(nBodies, 0.0f);
    tasks().parallelFor({0u, nBodies}, [&](TaskRange range) {
      for (uint32_t i = range.begin; i < range.end; ++i) {
        float neighborSum = 0.0f;
        for (uint32_t nb : pipeline.adjacency[i])
          neighborSum += pipeline.propagatedMass[nb];
        next[i] = bodies[i].mass + propagationDecay * neighborSum;
      }
    });
    pipeline.propagatedMass.swap(next);
  }

  tasks().parallelFor({0u, (uint32_t)contacts.size()}, [&](TaskRange range) {
    for (uint32_t ci = range.begin; ci < range.end; ++ci) {
      auto &c = contacts[ci];
      float augA = pipeline.propagatedMass[storage.contacts.bodyA[ci]];
      float augB = (storage.contacts.bodyB[ci] != UINT32_MAX) ? pipeline.propagatedMass[storage.contacts.bodyB[ci]] : 0.0f;
      float massB = (storage.contacts.bodyB[ci] != UINT32_MAX) ? storage.bodies.mass[storage.contacts.bodyB[ci]] : 0.0f;
      float effectiveMass, scale;
      if (c.bodyB != UINT32_MAX && massB > 0.0f) {
        effectiveMass = std::max(augA, augB);
        scale = penaltyScaleDynDyn;
      } else {
        effectiveMass = augA;
        scale = penaltyScale;
      }
      float penFloor = std::max(PENALTY_MIN, scale * effectiveMass / dt2);
      for (int i = 0; i < 3; ++i) {
        storage.contacts.penalty[ci][i] = std::max(storage.contacts.penalty[ci][i], penFloor);
        c.penalty[i] = storage.contacts.penalty[ci][i];
      }
    }
  });
}

void Solver::stageComputeContactC0() {
  tasks().parallelFor({0u, (uint32_t)contacts.size()}, [&](TaskRange range) {
    for (uint32_t ci = range.begin; ci < range.end; ++ci)
      computeC0(contacts[ci]);
  });
}

void Solver::stageBuildIslands() {
  const uint32_t nBodies = (uint32_t)bodies.size();
  pipeline.islands.clear();
  std::vector<int> islandOfBody(nBodies, -1);

  for (uint32_t root = 0; root < nBodies; ++root) {
    if (bodies[root].mass <= 0 || islandOfBody[root] != -1)
      continue;
    IslandData island;
    std::vector<uint32_t> stack = {root};
    islandOfBody[root] = (int)pipeline.islands.size();
    while (!stack.empty()) {
      uint32_t bi = stack.back();
      stack.pop_back();
      island.bodies.push_back(bi);
      for (uint32_t nb : pipeline.adjacency[bi]) {
        if (nb < nBodies && bodies[nb].mass > 0 && islandOfBody[nb] == -1) {
          islandOfBody[nb] = islandOfBody[root];
          stack.push_back(nb);
        }
      }
    }
    pipeline.islands.push_back(std::move(island));
  }

  for (uint32_t bi = 0; bi < nBodies; ++bi) {
    if (bodies[bi].mass <= 0)
      continue;
    if (islandOfBody[bi] == -1) {
      IslandData island;
      island.bodies.push_back(bi);
      islandOfBody[bi] = (int)pipeline.islands.size();
      pipeline.islands.push_back(std::move(island));
    }
  }

  for (uint32_t ci = 0; ci < contacts.size(); ++ci) {
    const auto &c = contacts[ci];
    int island = (c.bodyA < nBodies) ? islandOfBody[c.bodyA] : -1;
    if (island >= 0)
      pipeline.islands[island].contacts.push_back(ci);
  }
  for (uint32_t ji = 0; ji < d6Joints.size(); ++ji) {
    const auto &j = d6Joints[ji];
    int island = (j.bodyA < nBodies && j.bodyA != UINT32_MAX) ? islandOfBody[j.bodyA] :
                 ((j.bodyB < nBodies) ? islandOfBody[j.bodyB] : -1);
    if (island >= 0)
      pipeline.islands[island].d6Joints.push_back(ji);
  }
  for (uint32_t gi = 0; gi < gearJoints.size(); ++gi) {
    const auto &g = gearJoints[gi];
    int island = (g.bodyA < nBodies) ? islandOfBody[g.bodyA] :
                 ((g.bodyB < nBodies) ? islandOfBody[g.bodyB] : -1);
    if (island >= 0)
      pipeline.islands[island].gearJoints.push_back(gi);
  }
  for (uint32_t ai = 0; ai < articulations.size(); ++ai) {
    const auto &artic = articulations[ai];
    int island = -1;
    for (const auto &j : artic.joints) {
      if (j.bodyIndex < nBodies && islandOfBody[j.bodyIndex] >= 0) {
        island = islandOfBody[j.bodyIndex];
        break;
      }
    }
    if (island >= 0) {
      pipeline.islands[island].articulations.push_back(ai);
      for (const auto &j : artic.joints)
        pipeline.islands[island].articulationBodies.push_back(j.bodyIndex);
    }
  }

  if (shouldSortBodies(*this)) {
    for (auto &island : pipeline.islands) {
      std::sort(island.bodies.begin(), island.bodies.end());
      std::sort(island.articulationBodies.begin(), island.articulationBodies.end());
      island.articulationBodies.erase(std::unique(island.articulationBodies.begin(), island.articulationBodies.end()), island.articulationBodies.end());
    }
  }
  if (shouldSortConstraints(*this)) {
    for (auto &island : pipeline.islands) {
      std::sort(island.contacts.begin(), island.contacts.end());
      std::sort(island.d6Joints.begin(), island.d6Joints.end());
      std::sort(island.gearJoints.begin(), island.gearJoints.end());
      std::sort(island.articulations.begin(), island.articulations.end());
    }
  }
  if (shouldSortIslands(*this)) {
    std::sort(pipeline.islands.begin(), pipeline.islands.end(), [](const Solver::IslandData &a, const Solver::IslandData &b) {
      const uint32_t aKey = a.bodies.empty() ? UINT32_MAX : a.bodies.front();
      const uint32_t bKey = b.bodies.empty() ? UINT32_MAX : b.bodies.front();
      return aKey < bKey;
    });
  }
}

void Solver::stageBuildContactBatches() {
  pipeline.contactBatches.assign(pipeline.islands.size(), {});
  for (size_t ii = 0; ii < pipeline.islands.size(); ++ii)
    pipeline.contactBatches[ii] = pipeline.islands[ii].contacts;
  storage.rebuildPackedContactBatches(pipeline.contactBatches);
}

void Solver::stageBuildConstraintGraphs() {
  pipeline.constraintGraphs.assign(pipeline.islands.size(), {});
  auto appendBody = [](std::vector<uint32_t> &out, uint32_t body) {
    if (body != UINT32_MAX)
      out.push_back(body);
  };

  for (size_t ii = 0; ii < pipeline.islands.size(); ++ii) {
    auto &graph = pipeline.constraintGraphs[ii];
    const auto &island = pipeline.islands[ii];

    for (uint32_t ci : island.contacts) {
      ConstraintGraph::Node node;
      node.type = ConstraintGraph::Node::ContactNode;
      node.index0 = ci;
      appendBody(node.bodies, contacts[ci].bodyA);
      appendBody(node.bodies, contacts[ci].bodyB);
      graph.nodes.push_back(std::move(node));
    }
    for (uint32_t ji : island.d6Joints) {
      ConstraintGraph::Node node;
      node.type = ConstraintGraph::Node::D6Node;
      node.index0 = ji;
      appendBody(node.bodies, d6Joints[ji].bodyA);
      appendBody(node.bodies, d6Joints[ji].bodyB);
      graph.nodes.push_back(std::move(node));
    }
    for (uint32_t gi : island.gearJoints) {
      ConstraintGraph::Node node;
      node.type = ConstraintGraph::Node::GearNode;
      node.index0 = gi;
      appendBody(node.bodies, gearJoints[gi].bodyA);
      appendBody(node.bodies, gearJoints[gi].bodyB);
      graph.nodes.push_back(std::move(node));
    }
    for (uint32_t ai : island.articulations) {
      const auto &artic = articulations[ai];
      for (uint32_t ji = 0; ji < artic.joints.size(); ++ji) {
        ConstraintGraph::Node node;
        node.type = ConstraintGraph::Node::ArticulationJointNode;
        node.index0 = ai;
        node.index1 = ji;
        appendBody(node.bodies, artic.joints[ji].bodyIndex);
        appendBody(node.bodies, artic.getParentBodyIndex((int)ji));
        graph.nodes.push_back(std::move(node));
      }
      for (uint32_t mi = 0; mi < artic.mimicJoints.size(); ++mi) {
        ConstraintGraph::Node node;
        node.type = ConstraintGraph::Node::MimicNode;
        node.index0 = ai;
        node.index1 = mi;
        appendBody(node.bodies, artic.joints[artic.mimicJoints[mi].jointA].bodyIndex);
        appendBody(node.bodies, artic.joints[artic.mimicJoints[mi].jointB].bodyIndex);
        graph.nodes.push_back(std::move(node));
      }
      for (uint32_t ti = 0; ti < artic.ikTargets.size(); ++ti) {
        ConstraintGraph::Node node;
        node.type = ConstraintGraph::Node::IKNode;
        node.index0 = ai;
        node.index1 = ti;
        if (artic.ikTargets[ti].jointIdx >= 0 &&
            (uint32_t)artic.ikTargets[ti].jointIdx < artic.joints.size())
          appendBody(node.bodies, artic.joints[artic.ikTargets[ti].jointIdx].bodyIndex);
        graph.nodes.push_back(std::move(node));
      }
    }

    std::vector<int> nodeColor(graph.nodes.size(), -1);
    for (size_t ni = 0; ni < graph.nodes.size(); ++ni) {
      std::vector<bool> used(graph.nodes.size(), false);
      for (size_t nj = 0; nj < ni; ++nj) {
        bool shareBody = false;
        for (uint32_t a : graph.nodes[ni].bodies)
          for (uint32_t b : graph.nodes[nj].bodies)
            if (a == b && a != UINT32_MAX)
              shareBody = true;
        if (shareBody && nodeColor[nj] >= 0)
          used[nodeColor[nj]] = true;
      }
      int color = 0;
      while (color < (int)used.size() && used[color])
        ++color;
      nodeColor[ni] = color;
      if ((size_t)color >= graph.colors.size())
        graph.colors.resize(color + 1);
      graph.colors[color].push_back((uint32_t)ni);
    }
  }
}

void Solver::stageBuildSweepOrders() {
  pipeline.sweepOrders.assign(pipeline.islands.size(), {});
  std::vector<bool> isArticBody(bodies.size(), false);
  if (useTreeSweep) {
    for (const auto &artic : articulations)
      for (const auto &joint : artic.joints)
        if (joint.bodyIndex < bodies.size())
          isArticBody[joint.bodyIndex] = true;
  }

  for (size_t ii = 0; ii < pipeline.islands.size(); ++ii) {
    auto &order = pipeline.sweepOrders[ii];
    const auto &island = pipeline.islands[ii];
    for (uint32_t bi : island.bodies)
      if (!isArticBody[bi])
        order.push_back(bi);
    for (uint32_t bi : island.articulationBodies)
      if (bi < bodies.size())
        order.push_back(bi);
    if (shouldSortBodies(*this))
      std::sort(order.begin(), order.end());
    order.erase(std::unique(order.begin(), order.end()), order.end());
  }
}

void Solver::stagePrimalSolve(float invDt, float dt2) {
  const float contactBoostFraction = 0.005f;
  int aaDim = (int)bodies.size() * 3;
  int aaCount = 0;
  std::vector<std::vector<float>> aaFHistory, aaXHistory;
  if (useAndersonAccel) {
    aaFHistory.assign(aaWindowSize, std::vector<float>(aaDim, 0.0f));
    aaXHistory.assign(aaWindowSize, std::vector<float>(aaDim, 0.0f));
  }
  auto packState = [&](std::vector<float> &state) {
    state.resize(aaDim);
    for (uint32_t i = 0; i < bodies.size(); i++) {
      state[i * 3 + 0] = storage.bodies.position[i].x;
      state[i * 3 + 1] = storage.bodies.position[i].y;
      state[i * 3 + 2] = storage.bodies.position[i].z;
    }
  };
  auto unpackState = [&](const std::vector<float> &state) {
    for (uint32_t i = 0; i < bodies.size(); i++) {
      if (storage.bodies.mass[i] <= 0) continue;
      storage.bodies.position[i] = Vec3(state[i * 3 + 0], state[i * 3 + 1], state[i * 3 + 2]);
      syncBodyStateToAoS(*this, i);
    }
  };

  float chebyOmega = 1.0f;
  std::vector<Vec3> chebyPrevPos, chebyPrevPrevPos;
  std::vector<Quat> chebyPrevRot, chebyPrevPrevRot;
  if (useChebyshev) {
    chebyPrevPos.resize(bodies.size());
    chebyPrevPrevPos.resize(bodies.size());
    chebyPrevRot.resize(bodies.size());
    chebyPrevPrevRot.resize(bodies.size());
    for (uint32_t i = 0; i < bodies.size(); ++i) {
      chebyPrevPos[i] = chebyPrevPrevPos[i] = storage.bodies.position[i];
      chebyPrevRot[i] = chebyPrevPrevRot[i] = storage.bodies.rotation[i];
    }
  }

  convergenceHistory.clear();

  for (int it = 0; it < iterations; ++it) {
    std::vector<float> preState;
    if (useAndersonAccel)
      packState(preState);
    if (useChebyshev) {
      for (uint32_t i = 0; i < bodies.size(); ++i) {
        chebyPrevPrevPos[i] = chebyPrevPos[i];
        chebyPrevPrevRot[i] = chebyPrevRot[i];
        chebyPrevPos[i] = storage.bodies.position[i];
        chebyPrevRot[i] = storage.bodies.rotation[i];
      }
    }

    for (size_t ii = 0; ii < pipeline.islands.size(); ++ii) {
      const auto &orderBase = pipeline.sweepOrders[ii];
      std::vector<uint32_t> order = orderBase;
      if (useTreeSweep && (it % 2 == 1))
        std::reverse(order.begin(), order.end());

      for (uint32_t bi : order) {
        if (storage.bodies.mass[bi] <= 0)
          continue;
        if (runtimeConfig().hasExecutionFlag(ExecutionFlags::eENABLE_SLEEPING) &&
            sleepSystem.isSleeping(bi))
          continue;
        Body &body = bodies[bi];

        bool bodyNeedsFull6x6 = false;
        if (use3x3Solve) {
          for (uint32_t ji : pipeline.islands[ii].d6Joints) {
            const auto &jnt = d6Joints[ji];
            if ((jnt.bodyA == bi || jnt.bodyB == bi) &&
                (jnt.linearMotion != 0 || jnt.angularMotion != 0x2A)) {
              bodyNeedsFull6x6 = true;
              break;
            }
          }
        }

        Mat66 lhs = storage.bodies.getMassMatrix(bi) / dt2;
        Vec6 disp(storage.bodies.position[bi] - storage.bodies.inertialPosition[bi],
                  storage.bodies.deltaWInertial(bi));
        Vec6 rhs = lhs * disp;
        float boostFloor = contactBoostFraction * storage.bodies.mass[bi] / dt2;

        const auto &packedBatches = storage.packedContactBatches[ii];
        auto accumulateContact = [&](uint32_t ci) {
          auto &c = contacts[ci];
          bool isA = (storage.contacts.bodyA[ci] == bi);
          bool isB = (storage.contacts.bodyB[ci] == bi);
          if (!isA && !isB)
            return;
          computeConstraint(c);
          for (int i = 0; i < 3; ++i) {
            Vec6 J = isA ? (i == 0 ? c.JA : (i == 1 ? c.JAt1 : c.JAt2))
                         : (i == 0 ? c.JB : (i == 1 ? c.JBt1 : c.JBt2));
            float pen = std::max(storage.contacts.penalty[ci][i], boostFloor);
            float f = std::max(storage.contacts.fmin[ci][i],
                               std::min(storage.contacts.fmax[ci][i],
                                        pen * storage.contacts.C[ci][i] + storage.contacts.lambda[ci][i]));
            rhs += J * f;
            lhs += outer(J, J * pen);
          }
        };
        for (const auto &batch : packedBatches) {
          if (batch.count == SoA::PACK_WIDTH) {
            for (uint32_t lane = 0; lane < SoA::PACK_WIDTH; ++lane)
              accumulateContact(batch.indices[lane]);
          } else {
            for (uint32_t lane = 0; lane < batch.count; ++lane)
              accumulateContact(batch.indices[lane]);
          }
        }

        for (uint32_t ji : pipeline.islands[ii].d6Joints)
          addD6Contribution(d6Joints[ji], bi, bodies, dt, lhs, rhs);

        for (uint32_t ai : pipeline.islands[ii].articulations) {
          const auto &artic = articulations[ai];
          for (int ji = 0; ji < (int)artic.joints.size(); ++ji)
            addArticulationContribution(artic, ji, bi, bodies, dt, lhs, rhs);
          for (int mi = 0; mi < (int)artic.mimicJoints.size(); ++mi)
            addMimicJointContribution(artic, mi, bi, bodies, dt, lhs, rhs);
          for (int ti = 0; ti < (int)artic.ikTargets.size(); ++ti)
            addIKTargetContribution(artic, ti, bi, bodies, dt, lhs, rhs);
        }

        for (uint32_t gi : pipeline.islands[ii].gearJoints) {
          auto &gnt = gearJoints[gi];
          bool isA = (gnt.bodyA == bi), isB = (gnt.bodyB == bi);
          if (!isA && !isB)
            continue;
          if (gnt.bodyA >= bodies.size() || gnt.bodyB >= bodies.size())
            continue;
          Body &bA = bodies[gnt.bodyA];
          Body &bB = bodies[gnt.bodyB];
          if (bA.mass <= 0.f || bB.mass <= 0.f)
            continue;
          Vec3 worldAxisA = bA.rotation.rotate(gnt.axisA);
          Vec3 worldAxisB = bB.rotation.rotate(gnt.axisB);
          Vec3 dwA = bA.deltaWInitial();
          Vec3 dwB = bB.deltaWInitial();
          float C = dwA.dot(worldAxisA) * gnt.gearRatio + dwB.dot(worldAxisB);
          float effectiveRho = std::max(gnt.rho, storage.bodies.mass[bi] / dt2);
          Vec3 J_ang = isA ? (worldAxisA * gnt.gearRatio) : worldAxisB;
          float f = effectiveRho * C + gnt.lambdaGear;
          for (int r = 0; r < 3; r++)
            for (int c2 = 0; c2 < 3; c2++)
              lhs.m[3 + r][3 + c2] += effectiveRho * (&J_ang.x)[r] * (&J_ang.x)[c2];
          for (int r = 0; r < 3; r++)
            rhs.v[3 + r] += f * (&J_ang.x)[r];
        }

        bool solve3x3ForBody = use3x3Solve && !bodyNeedsFull6x6;
        if (!solve3x3ForBody) {
          Vec6 delta = solveLDLT(lhs, rhs);
          storage.bodies.position[bi] -= delta.linear();
          Quat dq(0, delta[3], delta[4], delta[5]);
          storage.bodies.rotation[bi] = (storage.bodies.rotation[bi] - dq * storage.bodies.rotation[bi] * 0.5f).normalized();
        } else {
          Mat33 Alin, Aang;
          Vec3 rhsLin(rhs[0], rhs[1], rhs[2]);
          Vec3 rhsAng(rhs[3], rhs[4], rhs[5]);
          for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
              Alin.m[r][c] = lhs.m[r][c];
              Aang.m[r][c] = lhs.m[3 + r][3 + c];
            }
          Vec3 deltaPos = Alin.inverse() * rhsLin;
          Vec3 deltaTheta = Aang.inverse() * rhsAng;
          storage.bodies.position[bi] -= deltaPos;
          Quat dq(0, deltaTheta.x, deltaTheta.y, deltaTheta.z);
          storage.bodies.rotation[bi] = (storage.bodies.rotation[bi] - dq * storage.bodies.rotation[bi] * 0.5f).normalized();
        }
        syncBodyStateToAoS(*this, bi);
      }
    }

    auto dualStart = std::chrono::steady_clock::now();
    stageDualUpdate(dt2);
    lastStepStats.stageDualUpdateMs +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - dualStart).count();

    if (useAndersonAccel) {
      std::vector<float> postState;
      packState(postState);
      std::vector<float> fk(aaDim);
      for (int i = 0; i < aaDim; ++i)
        fk[i] = postState[i] - preState[i];
      int slot = aaCount % aaWindowSize;
      aaXHistory[slot] = preState;
      aaFHistory[slot] = fk;
      aaCount++;
      int mk = std::min(aaCount - 1, aaWindowSize);
      if (mk >= 1) {
        std::vector<std::vector<float>> deltaF(mk, std::vector<float>(aaDim));
        std::vector<std::vector<float>> deltaX(mk, std::vector<float>(aaDim));
        for (int j = 0; j < mk; ++j) {
          int prevSlot = (slot - 1 - j + aaWindowSize * 2) % aaWindowSize;
          for (int i = 0; i < aaDim; ++i) {
            deltaF[j][i] = fk[i] - aaFHistory[prevSlot][i];
            deltaX[j][i] = preState[i] - aaXHistory[prevSlot][i];
          }
        }
        std::vector<float> FTF(mk * mk, 0.0f), FTf(mk, 0.0f), theta(mk, 0.0f);
        for (int i = 0; i < mk; ++i) {
          for (int j = 0; j <= i; ++j) {
            float dot = 0;
            for (int d = 0; d < aaDim; ++d)
              dot += deltaF[i][d] * deltaF[j][d];
            FTF[i * mk + j] = FTF[j * mk + i] = dot;
          }
          for (int d = 0; d < aaDim; ++d)
            FTf[i] += deltaF[i][d] * fk[d];
        }
        float maxDiag = 0;
        for (int i = 0; i < mk; ++i)
          maxDiag = std::max(maxDiag, FTF[i * mk + i]);
        float reg = 1e-8f * std::max(maxDiag, 1.0f);
        for (int i = 0; i < mk; ++i)
          FTF[i * mk + i] += reg;
        for (int i = 0; i < mk; ++i) {
          float pivot = FTF[i * mk + i];
          if (std::fabs(pivot) < 1e-15f) continue;
          for (int j = i + 1; j < mk; ++j) {
            float factor = FTF[j * mk + i] / pivot;
            for (int k = i + 1; k < mk; ++k)
              FTF[j * mk + k] -= factor * FTF[i * mk + k];
            FTf[j] -= factor * FTf[i];
          }
        }
        for (int i = mk - 1; i >= 0; --i) {
          float sum = FTf[i];
          for (int j = i + 1; j < mk; ++j)
            sum -= FTF[i * mk + j] * theta[j];
          float pivot = FTF[i * mk + i];
          theta[i] = (std::fabs(pivot) > 1e-15f) ? (sum / pivot) : 0.0f;
        }
        std::vector<float> aaState(aaDim);
        for (int i = 0; i < aaDim; ++i) {
          float correction = 0;
          for (int j = 0; j < mk; ++j)
            correction += theta[j] * (deltaX[j][i] + deltaF[j][i]);
          aaState[i] = postState[i] - correction;
        }
        float violBefore = 0;
        for (auto &artic : articulations)
          violBefore = std::max(violBefore, artic.computeMaxPositionViolation(bodies));
        unpackState(aaState);
        float violAfter = 0;
        for (auto &artic : articulations)
          violAfter = std::max(violAfter, artic.computeMaxPositionViolation(bodies));
        if (violAfter > violBefore)
          unpackState(postState);
      }
    }

    if (useChebyshev && it >= 2) {
      float rhoSq = chebyshevSpectralRadius * chebyshevSpectralRadius;
      chebyOmega = (it == 2) ? (2.0f / (2.0f - rhoSq))
                             : (1.0f / (1.0f - rhoSq * chebyOmega / 4.0f));
      chebyOmega = std::max(1.0f, std::min(chebyOmega, 2.0f));
      for (uint32_t i = 0; i < bodies.size(); ++i) {
        if (storage.bodies.mass[i] <= 0) continue;
        storage.bodies.position[i] = chebyPrevPrevPos[i] +
            (storage.bodies.position[i] - chebyPrevPrevPos[i]) * chebyOmega;
        Quat qPrev = chebyPrevPrevRot[i];
        Quat qCur = storage.bodies.rotation[i];
        float dotQ = qPrev.w * qCur.w + qPrev.x * qCur.x +
                     qPrev.y * qCur.y + qPrev.z * qCur.z;
        if (dotQ < 0) qCur = qCur * (-1.0f);
        Quat qBlend(qPrev.w + chebyOmega * (qCur.w - qPrev.w),
                    qPrev.x + chebyOmega * (qCur.x - qPrev.x),
                    qPrev.y + chebyOmega * (qCur.y - qPrev.y),
                    qPrev.z + chebyOmega * (qCur.z - qPrev.z));
        storage.bodies.rotation[i] = qBlend.normalized();
        syncBodyStateToAoS(*this, i);
      }
    }

    if (!articulations.empty()) {
      float maxViol = 0;
      for (const auto &artic : articulations)
        maxViol = std::max(maxViol, artic.computeMaxPositionViolation(bodies));
      convergenceHistory.push_back(maxViol);
    }
  }
}

void Solver::stageDualUpdate(float dt2) {
  tasks().parallelFor({0u, (uint32_t)contacts.size()}, [&](TaskRange range) {
    for (uint32_t ci = range.begin; ci < range.end; ++ci) {
      auto &c = contacts[ci];
      computeConstraint(c);
      for (int i = 0; i < 3; i++) {
        float oldLambda = storage.contacts.lambda[ci][i];
        float rawLambda = storage.contacts.penalty[ci][i] * storage.contacts.C[ci][i] + oldLambda;
        storage.contacts.lambda[ci][i] = std::max(storage.contacts.fmin[ci][i], std::min(storage.contacts.fmax[ci][i], rawLambda));
        if (storage.contacts.lambda[ci][i] < storage.contacts.fmax[ci][i] &&
            storage.contacts.lambda[ci][i] > storage.contacts.fmin[ci][i])
          storage.contacts.penalty[ci][i] = std::min(storage.contacts.penalty[ci][i] +
                                                         beta * fabsf(storage.contacts.C[ci][i]),
                                                     PENALTY_MAX);
        c.lambda[i] = storage.contacts.lambda[ci][i];
        c.penalty[i] = storage.contacts.penalty[ci][i];
      }
    }
  });

  for (const auto &graph : pipeline.constraintGraphs) {
    for (const auto &colorBatch : graph.colors) {
      for (uint32_t nodeIdx : colorBatch) {
        const auto &node = graph.nodes[nodeIdx];
        switch (node.type) {
        case ConstraintGraph::Node::ContactNode:
          break;
        case ConstraintGraph::Node::D6Node:
          updateD6Dual(d6Joints[node.index0], bodies, dt, 0.99f);
          break;
        case ConstraintGraph::Node::GearNode: {
          auto &gnt = gearJoints[node.index0];
          if (gnt.bodyA >= bodies.size() || gnt.bodyB >= bodies.size())
            break;
          Body &bA = bodies[gnt.bodyA];
          Body &bB = bodies[gnt.bodyB];
          if (bA.mass <= 0.f || bB.mass <= 0.f)
            break;
          float mA = bA.mass, mB = bB.mass;
          float mEff2 = (mA > 0.f && mB > 0.f) ? std::min(mA, mB) : std::max(mA, mB);
          float Mh2 = mEff2 / dt2;
          float admm_step = gnt.rho * gnt.rho / (gnt.rho + Mh2);
          float rhoDual = std::min(Mh2, admm_step);
          Vec3 worldAxisA = bA.rotation.rotate(gnt.axisA);
          Vec3 worldAxisB = bB.rotation.rotate(gnt.axisB);
          Vec3 dwA = bA.deltaWInitial();
          Vec3 dwB = bB.deltaWInitial();
          float C = dwA.dot(worldAxisA) * gnt.gearRatio + dwB.dot(worldAxisB);
          gnt.lambdaGear = gnt.lambdaGear * 0.99f + rhoDual * C;
          break;
        }
        case ConstraintGraph::Node::ArticulationJointNode:
          updateArticulationDual(articulations[node.index0], (int)node.index1, bodies, dt, 0.99f);
          break;
        case ConstraintGraph::Node::MimicNode:
          updateMimicDual(articulations[node.index0], (int)node.index1, bodies, dt, 0.99f);
          break;
        case ConstraintGraph::Node::IKNode:
          updateIKTargetDual(articulations[node.index0], (int)node.index1, bodies, dt, 0.99f);
          break;
        }
      }
    }
  }
}

void Solver::applyPostSolveMotors(float invDt, float dt2) {
  for (auto &jnt : d6Joints) {
    if (!jnt.motorEnabled || jnt.motorMaxForce <= 0.0f)
      continue;
    bool isAStatic = (jnt.bodyA == UINT32_MAX || jnt.bodyA >= (uint32_t)bodies.size());
    bool isBStatic = (jnt.bodyB == UINT32_MAX || jnt.bodyB >= (uint32_t)bodies.size());
    if (isAStatic && isBStatic)
      continue;
    Quat rotA_m = isAStatic ? Quat() : bodies[jnt.bodyA].rotation;
    Vec3 worldAxis = (rotA_m * jnt.localFrameA).rotate(Vec3(1, 0, 0));
    float axLen = worldAxis.length();
    if (axLen > 1e-6f) worldAxis = worldAxis * (1.0f / axLen);
    if (!isBStatic) {
      Body &bodyB = bodies[jnt.bodyB];
      Quat deltaQB = bodyB.rotation * bodyB.initialRotation.conjugate();
      if (deltaQB.w < 0.0f) deltaQB = deltaQB * (-1.0f);
      Vec3 currentAngVel = Vec3(deltaQB.x, deltaQB.y, deltaQB.z) * (2.0f * invDt);
      float velocityError = jnt.motorTargetVelocity - currentAngVel.dot(worldAxis);
      Vec3 invITimesAxis = bodyB.invInertiaWorld * worldAxis;
      float effectiveInvInertia = worldAxis.dot(invITimesAxis);
      if (effectiveInvInertia >= 1e-10f) {
        float requiredTorque = velocityError * invDt / effectiveInvInertia;
        float clampedTorque = std::max(-jnt.motorMaxForce, std::min(jnt.motorMaxForce, requiredTorque));
        float deltaAngle = clampedTorque * effectiveInvInertia * dt2;
        float ha = deltaAngle * 0.5f;
        Quat dRot(cosf(ha), worldAxis.x * sinf(ha), worldAxis.y * sinf(ha), worldAxis.z * sinf(ha));
        bodyB.rotation = (dRot * bodyB.rotation).normalized();
      }
    }
    if (!isAStatic) {
      Body &bodyA = bodies[jnt.bodyA];
      Quat deltaQA = bodyA.rotation * bodyA.initialRotation.conjugate();
      if (deltaQA.w < 0.0f) deltaQA = deltaQA * (-1.0f);
      Vec3 currentAngVelA = Vec3(deltaQA.x, deltaQA.y, deltaQA.z) * (2.0f * invDt);
      float velocityErrorA = -jnt.motorTargetVelocity - currentAngVelA.dot(worldAxis);
      Vec3 invITimesAxisA = bodyA.invInertiaWorld * worldAxis;
      float effectiveInvInertiaA = worldAxis.dot(invITimesAxisA);
      if (effectiveInvInertiaA > 1e-10f) {
        float requiredTorqueA = velocityErrorA * invDt / effectiveInvInertiaA;
        float clampedTorqueA = std::max(-jnt.motorMaxForce, std::min(jnt.motorMaxForce, requiredTorqueA));
        float haA = clampedTorqueA * effectiveInvInertiaA * dt2 * 0.5f;
        Quat dRotA(cosf(haA), worldAxis.x * sinf(haA), worldAxis.y * sinf(haA), worldAxis.z * sinf(haA));
        bodyA.rotation = (dRotA * bodyA.rotation).normalized();
      }
    }
  }
}

void Solver::stageVelocityWriteback(float invDt) {
  tasks().parallelFor({0u, (uint32_t)bodies.size()}, [&](TaskRange range) {
    for (uint32_t bi = range.begin; bi < range.end; ++bi) {
      auto &body = bodies[bi];
      if (storage.bodies.mass[bi] <= 0)
        continue;
      body.prevLinearVelocity = storage.bodies.linearVelocity[bi];
      storage.bodies.prevLinearVelocity[bi] = storage.bodies.linearVelocity[bi];
      storage.bodies.linearVelocity[bi] =
          (storage.bodies.position[bi] - storage.bodies.initialPosition[bi]) * invDt;
      Quat dq = storage.bodies.rotation[bi] * storage.bodies.initialRotation[bi].conjugate();
      if (dq.w < 0)
        dq = -dq;
      storage.bodies.angularVelocity[bi] = Vec3(dq.x, dq.y, dq.z) * (2.0f * invDt);
      if (storage.bodies.linearDamping[bi] > 0.0f) {
        float decay = std::max(0.0f, 1.0f - storage.bodies.linearDamping[bi] * dt);
        storage.bodies.linearVelocity[bi] = storage.bodies.linearVelocity[bi] * decay;
      }
      if (storage.bodies.angularDamping[bi] > 0.0f) {
        float decay = std::max(0.0f, 1.0f - storage.bodies.angularDamping[bi] * dt);
        storage.bodies.angularVelocity[bi] = storage.bodies.angularVelocity[bi] * decay;
      }
      float linSpeed = storage.bodies.linearVelocity[bi].length();
      if (linSpeed > storage.bodies.maxLinearVelocity[bi])
        storage.bodies.linearVelocity[bi] = storage.bodies.linearVelocity[bi] *
                                            (storage.bodies.maxLinearVelocity[bi] / linSpeed);
      float angSpeed = storage.bodies.angularVelocity[bi].length();
      if (angSpeed > storage.bodies.maxAngularVelocity[bi])
        storage.bodies.angularVelocity[bi] = storage.bodies.angularVelocity[bi] *
                                             (storage.bodies.maxAngularVelocity[bi] / angSpeed);
      syncBodyStateToAoS(*this, bi);
    }
  });
}

void Solver::step(float dt_) {
  ProfileScope frameScope(runtimeConfig().hasExecutionFlag(ExecutionFlags::eENABLE_PROFILING) ? &profiler : nullptr, "Solver.step");
  dt = dt_;
  float invDt = 1.0f / dt;
  float dt2 = dt * dt;
  runtime.stats().reset();
  runtime.stats().numBodies = (uint32_t)bodies.size();
  runtime.stats().numContacts = (uint32_t)contacts.size();
  runtime.stats().numJoints = (uint32_t)(d6Joints.size() + gearJoints.size());
  runtime.stats().totalIterations = (uint32_t)iterations;
  sleepSystem.beginStep(bodies);
  storage.buildFromScene(bodies, contacts, d6Joints, gearJoints, articulations);
  lastStepStats.aosToSoABuildMs = msSince(t0);
  warmstart();
  stagePredictionAndInertia(invDt, dt2);
  stageBuildAdjacency();
  stagePropagateMass(dt2);
  stageComputeContactC0();
  t0 = Clock::now();
  stageBuildIslands();
  runtime.stats().numIslands = (uint32_t)pipeline.islands.size();
  stageBuildContactBatches();
  stageBuildConstraintGraphs();
  stageBuildSweepOrders();
  t0 = Clock::now();
  stagePrimalSolve(invDt, dt2);
  lastStepStats.stagePrimalSolveMs = msSince(t0);
  applyPostSolveMotors(invDt, dt2);
  storage.bodies.buildFromBodies(bodies);
  t0 = Clock::now();
  stageVelocityWriteback(invDt);
  lastStepStats.writebackMs = msSince(t0);
  t0 = Clock::now();
  storage.scatterToScene(bodies, contacts);
  if (runtimeConfig().hasExecutionFlag(ExecutionFlags::eENABLE_SLEEPING))
    sleepSystem.endStep(bodies, dt, runtimeConfig());
  runtime.stats().numSleepingBodies = sleepSystem.sleepingCount();
  runtime.stats().numActiveBodies = runtime.stats().numBodies - runtime.stats().numSleepingBodies;
}

} // namespace AvbdRef
