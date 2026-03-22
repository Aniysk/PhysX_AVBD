#pragma once
#include "avbd_types.h"
#include <algorithm>
#include <cmath>

namespace AvbdRef {
namespace ContactPrep {

static constexpr float CONTACT_PENALTY_MIN = 1000.0f;
static constexpr float CONTACT_PENALTY_MAX = 1e9f;

// Minimal solver-facing contact contract derived from the PhysX AVBD path.
// The standalone layer keeps only the fields the AVBD solver actually consumes:
//   - body ids
//   - world normal + tangent basis
//   - local/world contact anchors
//   - separation / penetration
//   - friction / restitution material properties
//   - cached lambdas / penalties / force limits
//
// PhysX parity map:
//   - DyAvbdContactPrep.h::AvbdContactPrep::convertContact/buildManifold
//   - DyAvbdContactPrep.h::AvbdFrictionConstraint::computeTangentBasis
//   - DyAvbdSolver.cpp penalty floor / C0 initialization / lambda warmstart

struct ContactMaterial {
  float dynamicFriction = 0.5f;
  float restitution = 0.0f;

  static float combineFriction(float a, float b) {
    return std::sqrt(std::max(0.0f, a) * std::max(0.0f, b));
  }

  static float combineRestitution(float a, float b) {
    return std::max(a, b);
  }
};

struct ContactCache {
  float lambda[3] = {0.0f, 0.0f, 0.0f};
  float penalty[3] = {CONTACT_PENALTY_MIN, CONTACT_PENALTY_MIN, CONTACT_PENALTY_MIN};
};

struct ContactPoint {
  uint32_t bodyA = UINT32_MAX;
  uint32_t bodyB = UINT32_MAX;
  Vec3 worldPoint = {};
  Vec3 worldNormal = {0.0f, 1.0f, 0.0f}; // points from bodyB toward bodyA
  float separation = 0.0f;               // < 0 penetrating, > 0 gap
  Vec3 localPointA = {};
  Vec3 localPointB = {};
  bool bodyBAnchorIsWorld = false;       // true for static world/ground anchors
};

struct ContactRow {
  uint32_t bodyA = UINT32_MAX;
  uint32_t bodyB = UINT32_MAX;
  Vec3 worldNormal = {0.0f, 1.0f, 0.0f};
  Vec3 tangent0 = {1.0f, 0.0f, 0.0f};
  Vec3 tangent1 = {0.0f, 0.0f, 1.0f};
  Vec3 localAnchorA = {};
  Vec3 localAnchorB = {};
  bool bodyBAnchorIsWorld = false;
  float separation = 0.0f;
  float penetration = 0.0f;
  float friction = 0.5f;
  float restitution = 0.0f;
  float lambda[3] = {0.0f, 0.0f, 0.0f};
  float penalty[3] = {CONTACT_PENALTY_MIN, CONTACT_PENALTY_MIN, CONTACT_PENALTY_MIN};
  float fmin[3] = {-1e30f, 0.0f, 0.0f};
  float fmax[3] = {0.0f, 0.0f, 0.0f};
};

inline void computeTangents(const Vec3 &normal, Vec3 &t0, Vec3 &t1) {
  // PhysX-side AVBD friction basis uses an "almost-up" branch for robustness.
  // Semantic reference: DyAvbdContactPrep.h::AvbdFrictionConstraint::computeTangentBasis
  if (std::fabs(normal.y) > 0.9f)
    t0 = normal.cross(Vec3(1, 0, 0)).normalized();
  else
    t0 = normal.cross(Vec3(0, 1, 0)).normalized();
  t1 = normal.cross(t0);
}

inline ContactRow prepareRow(const ContactPoint &point,
                             const ContactMaterial &material,
                             const ContactCache *cache = nullptr) {
  ContactRow row;
  row.bodyA = point.bodyA;
  row.bodyB = point.bodyB;
  row.worldNormal = point.worldNormal.normalized();
  computeTangents(row.worldNormal, row.tangent0, row.tangent1);
  row.localAnchorA = point.localPointA;
  row.localAnchorB = point.localPointB;
  row.bodyBAnchorIsWorld = point.bodyBAnchorIsWorld;
  row.separation = point.separation;
  row.penetration = std::max(0.0f, -point.separation);
  row.friction = material.dynamicFriction;
  row.restitution = material.restitution;
  if (cache) {
    for (int i = 0; i < 3; ++i) {
      row.lambda[i] = cache->lambda[i];
      row.penalty[i] = std::clamp(cache->penalty[i], CONTACT_PENALTY_MIN, CONTACT_PENALTY_MAX);
    }
  }
  return row;
}

inline ContactRow prepareRow(uint32_t bodyA, uint32_t bodyB,
                             const Vec3 &worldPoint,
                             const Vec3 &worldNormal,
                             float separation,
                             const Vec3 &localPointA,
                             const Vec3 &localPointB,
                             const ContactMaterial &material,
                             const ContactCache *cache = nullptr,
                             bool bodyBAnchorIsWorld = false) {
  ContactPoint point;
  point.bodyA = bodyA;
  point.bodyB = bodyB;
  point.worldPoint = worldPoint;
  point.worldNormal = worldNormal;
  point.separation = separation;
  point.localPointA = localPointA;
  point.localPointB = localPointB;
  point.bodyBAnchorIsWorld = bodyBAnchorIsWorld;
  return prepareRow(point, material, cache);
}

inline Contact toSolverContact(const ContactRow &row) {
  Contact c;
  c.bodyA = row.bodyA;
  c.bodyB = row.bodyB;
  c.normal = row.worldNormal;
  c.rA = row.localAnchorA;
  c.rB = row.localAnchorB;
  c.depth = row.penetration;
  c.friction = row.friction;
  for (int i = 0; i < 3; ++i) {
    c.lambda[i] = row.lambda[i];
    c.penalty[i] = row.penalty[i];
    c.fmin[i] = row.fmin[i];
    c.fmax[i] = row.fmax[i];
    c.C[i] = 0.0f;
    c.C0[i] = 0.0f;
  }
  return c;
}

} // namespace ContactPrep
} // namespace AvbdRef
