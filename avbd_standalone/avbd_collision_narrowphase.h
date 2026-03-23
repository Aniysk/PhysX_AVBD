#pragma once
#include "avbd_collision_broadphase.h"
#include "avbd_collision_cache.h"
#include <cmath>

namespace AvbdRef {

inline void quatToAxes(const Quat &q, Vec3 axes[3]) {
  float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
  axes[0] = {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qz * qw), 2 * (qx * qz - qy * qw)};
  axes[1] = {2 * (qx * qy - qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + qx * qw)};
  axes[2] = {2 * (qx * qz + qy * qw), 2 * (qy * qz - qx * qw), 1 - 2 * (qx * qx + qy * qy)};
}

inline void getBoxFace(const Vec3 &localDir, const Vec3 &he, Vec3 face[4]) {
  float ax = fabsf(localDir.x), ay = fabsf(localDir.y), az = fabsf(localDir.z);
  if (ax >= ay && ax >= az) {
    float s = localDir.x > 0 ? he.x : -he.x;
    face[0] = {s, -he.y, -he.z}; face[1] = {s, he.y, -he.z}; face[2] = {s, he.y, he.z}; face[3] = {s, -he.y, he.z};
  } else if (ay >= az) {
    float s = localDir.y > 0 ? he.y : -he.y;
    face[0] = {-he.x, s, -he.z}; face[1] = {he.x, s, -he.z}; face[2] = {he.x, s, he.z}; face[3] = {-he.x, s, he.z};
  } else {
    float s = localDir.z > 0 ? he.z : -he.z;
    face[0] = {-he.x, -he.y, s}; face[1] = {he.x, -he.y, s}; face[2] = {he.x, he.y, s}; face[3] = {-he.x, he.y, s};
  }
}

inline int clipPolygon(const Vec3 *poly, int count, const Vec3 &planeN, float planeDist, Vec3 *out, int maxOut = 15) {
  if (count == 0)
    return 0;
  int outCount = 0;
  for (int i = 0; i < count && outCount < maxOut; ++i) {
    int j = (i + 1) % count;
    float di = poly[i].dot(planeN) - planeDist;
    float dj = poly[j].dot(planeN) - planeDist;
    if (di <= 0) {
      out[outCount++] = poly[i];
      if (dj > 0 && outCount < maxOut) {
        float t = di / (di - dj);
        out[outCount++] = poly[i] + (poly[j] - poly[i]) * t;
      }
    } else if (dj <= 0 && outCount < maxOut) {
      float t = di / (di - dj);
      out[outCount++] = poly[i] + (poly[j] - poly[i]) * t;
    }
  }
  return outCount;
}

inline bool generateBoxGroundManifold(const CollisionShape &box, const CollisionShape &ground,
                                      float margin, CollisionOutput &output) {
  (void)ground;
  SolverContactManifold manifold;
  manifold.bodyA = box.bodyIndex;
  manifold.bodyB = UINT32_MAX;
  manifold.normal = Vec3(0, 1, 0);
  manifold.friction = box.dynamicFriction;
  ContactPrep::ContactMaterial material;
  material.dynamicFriction = box.dynamicFriction;

  Vec3 he = box.halfExtent;
  Vec3 localCorners[8];
  int ci = 0;
  for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
      for (int sz = -1; sz <= 1; sz += 2)
        localCorners[ci++] = {sx * he.x, sy * he.y, sz * he.z};

  for (const Vec3 &localCorner : localCorners) {
    Vec3 worldCorner = box.position + box.rotation.rotate(localCorner);
    float dist = worldCorner.y;
    if (dist >= margin)
      continue;
    Vec3 groundPoint(worldCorner.x, 0.0f, worldCorner.z);
    manifold.rows.push_back(ContactPrep::prepareRow(box.bodyIndex, UINT32_MAX, groundPoint,
                                                    manifold.normal, dist, localCorner, groundPoint,
                                                    material, nullptr, true));
  }

  if (manifold.rows.empty())
    return false;
  output.manifolds.push_back(manifold);
  return true;
}

inline bool generateBoxBoxManifold(const CollisionShape &shapeA, const CollisionShape &shapeB,
                                   float margin, CollisionOutput &output) {
  Vec3 axA[3], axB[3];
  quatToAxes(shapeA.rotation, axA);
  quatToAxes(shapeB.rotation, axB);
  Vec3 d = shapeB.position - shapeA.position;
  float heAf[3] = {shapeA.halfExtent.x, shapeA.halfExtent.y, shapeA.halfExtent.z};
  float heBf[3] = {shapeB.halfExtent.x, shapeB.halfExtent.y, shapeB.halfExtent.z};
  float minPen = 1e30f;
  Vec3 bestAxis;

  auto testAxis = [&](Vec3 axis) -> bool {
    float len2 = axis.length2();
    if (len2 < 1e-10f)
      return true;
    Vec3 n = axis * (1.0f / sqrtf(len2));
    float rA = fabsf(axA[0].dot(n)) * heAf[0] + fabsf(axA[1].dot(n)) * heAf[1] + fabsf(axA[2].dot(n)) * heAf[2];
    float rB = fabsf(axB[0].dot(n)) * heBf[0] + fabsf(axB[1].dot(n)) * heBf[1] + fabsf(axB[2].dot(n)) * heBf[2];
    float dist = n.dot(d);
    float pen = rA + rB - fabsf(dist) + margin;
    if (pen < 0.0f)
      return false;
    if (pen < minPen) {
      minPen = pen;
      bestAxis = (dist > 0.0f) ? n * -1.0f : n;
    }
    return true;
  };

  for (int i = 0; i < 3; ++i) if (!testAxis(axA[i])) return false;
  for (int i = 0; i < 3; ++i) if (!testAxis(axB[i])) return false;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) if (!testAxis(axA[i].cross(axB[j]))) return false;

  Vec3 normal = bestAxis;
  float bestDotA = 0.0f, bestDotB = 0.0f;
  int bestFaceIdxA = 0, bestFaceIdxB = 0;
  for (int i = 0; i < 3; ++i) {
    float dd = fabsf(normal.dot(axA[i]));
    if (dd > bestDotA) { bestDotA = dd; bestFaceIdxA = i; }
    dd = fabsf(normal.dot(axB[i]));
    if (dd > bestDotB) { bestDotB = dd; bestFaceIdxB = i; }
  }

  const CollisionShape *refBody; const CollisionShape *incBody;
  Vec3 refAx[3], incAx[3], refHe, incHe, refFaceN;
  if (bestDotA >= bestDotB) {
    refBody = &shapeA; incBody = &shapeB;
    for (int i = 0; i < 3; ++i) { refAx[i] = axA[i]; incAx[i] = axB[i]; }
    refHe = shapeA.halfExtent; incHe = shapeB.halfExtent; refFaceN = axA[bestFaceIdxA];
    if (refFaceN.dot(d) < 0.0f) refFaceN = -refFaceN;
  } else {
    refBody = &shapeB; incBody = &shapeA;
    for (int i = 0; i < 3; ++i) { refAx[i] = axB[i]; incAx[i] = axA[i]; }
    refHe = shapeB.halfExtent; incHe = shapeA.halfExtent; refFaceN = axB[bestFaceIdxB];
    if (refFaceN.dot(d) > 0.0f) refFaceN = -refFaceN;
  }

  Vec3 refLocalDir(refFaceN.dot(refAx[0]), refFaceN.dot(refAx[1]), refFaceN.dot(refAx[2]));
  Vec3 refFace[4], refFaceW[4];
  getBoxFace(refLocalDir, refHe, refFace);
  for (int i = 0; i < 4; ++i)
    refFaceW[i] = refBody->position + refBody->rotation.rotate(refFace[i]);

  Vec3 incLocalDir(-refFaceN.dot(incAx[0]), -refFaceN.dot(incAx[1]), -refFaceN.dot(incAx[2]));
  Vec3 incFace[4], poly[16];
  getBoxFace(incLocalDir, incHe, incFace);
  for (int i = 0; i < 4; ++i)
    poly[i] = incBody->position + incBody->rotation.rotate(incFace[i]);
  int polyCount = 4;
  for (int e = 0; e < 4; ++e) {
    Vec3 edgeStart = refFaceW[e];
    Vec3 edgeEnd = refFaceW[(e + 1) % 4];
    Vec3 sideN = refFaceN.cross((edgeEnd - edgeStart).normalized());
    float sideDist = sideN.dot(edgeStart);
    Vec3 tmp[16];
    polyCount = clipPolygon(poly, polyCount, sideN, sideDist, tmp);
    for (int i = 0; i < polyCount; ++i) poly[i] = tmp[i];
    if (polyCount == 0) return false;
  }

  SolverContactManifold manifold;
  manifold.friction = ContactPrep::ContactMaterial::combineFriction(shapeA.dynamicFriction, shapeB.dynamicFriction);
  ContactPrep::ContactMaterial material; material.dynamicFriction = manifold.friction;
  float refPlaneDist = refFaceN.dot(refFaceW[0]);

  if (bestDotA >= bestDotB) {
    manifold.bodyA = shapeB.bodyIndex; manifold.bodyB = shapeA.bodyIndex; manifold.normal = -normal;
  } else {
    manifold.bodyA = shapeA.bodyIndex; manifold.bodyB = shapeB.bodyIndex; manifold.normal = normal;
  }

  for (int i = 0; i < polyCount; ++i) {
    float separation = poly[i].dot(refFaceN) - refPlaneDist;
    float depth = -separation;
    if (depth < -margin)
      continue;
    Vec3 contactPt = poly[i] - refFaceN * separation;
    const CollisionShape &contactA = (manifold.bodyA == shapeA.bodyIndex) ? shapeA : shapeB;
    const CollisionShape &contactB = (manifold.bodyB == shapeA.bodyIndex) ? shapeA : shapeB;
    Vec3 rA = contactA.rotation.conjugate().rotate(contactPt - contactA.position);
    Vec3 rB = contactB.rotation.conjugate().rotate(contactPt - contactB.position);
    manifold.rows.push_back(ContactPrep::prepareRow(manifold.bodyA, manifold.bodyB, contactPt,
                                                    manifold.normal, -depth, rA, rB, material));
  }

  if (manifold.rows.empty())
    return false;
  output.manifolds.push_back(manifold);
  return true;
}

inline bool generateContactManifold(const CollisionShapeSet &shapeSet, const BroadPhasePair &pair,
                                    float margin, CollisionOutput &output) {
  const CollisionShape &a = shapeSet.shapes[pair.shapeIndexA];
  const CollisionShape &b = shapeSet.shapes[pair.shapeIndexB];
  if (a.type == CollisionShape::Type::Box && b.type == CollisionShape::Type::GroundPlane)
    return generateBoxGroundManifold(a, b, margin, output);
  if (a.type == CollisionShape::Type::GroundPlane && b.type == CollisionShape::Type::Box)
    return generateBoxGroundManifold(b, a, margin, output);
  if (a.type == CollisionShape::Type::Box && b.type == CollisionShape::Type::Box)
    return generateBoxBoxManifold(a, b, margin, output);
  return false;
}

} // namespace AvbdRef
