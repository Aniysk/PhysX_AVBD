#pragma once
#include "avbd_collision_shapes.h"
#include <algorithm>
#include <vector>

namespace AvbdRef {

struct BroadPhasePair {
  uint32_t shapeIndexA = 0;
  uint32_t shapeIndexB = 0;
};

inline float collisionShapeBoundingRadius(const CollisionShape &shape) {
  if (shape.type == CollisionShape::Type::Box)
    return std::max({shape.halfExtent.x, shape.halfExtent.y, shape.halfExtent.z}) * 1.74f;
  return 0.0f;
}

inline std::vector<BroadPhasePair>
generateBroadPhasePairs(const CollisionShapeSet &shapeSet, float margin = 0.02f) {
  std::vector<BroadPhasePair> pairs;
  for (uint32_t i = 0; i < shapeSet.shapes.size(); ++i) {
    const CollisionShape &a = shapeSet.shapes[i];
    if (a.type == CollisionShape::Type::GroundPlane)
      continue;

    if (shapeSet.groundShapeIndex >= 0 && a.isDynamic())
      pairs.push_back({i, static_cast<uint32_t>(shapeSet.groundShapeIndex)});

    for (uint32_t j = i + 1; j < shapeSet.shapes.size(); ++j) {
      const CollisionShape &b = shapeSet.shapes[j];
      if (b.type == CollisionShape::Type::GroundPlane)
        continue;
      if (!a.isDynamic() && !b.isDynamic())
        continue;
      Vec3 diff = b.position - a.position;
      float maxDist = collisionShapeBoundingRadius(a) + collisionShapeBoundingRadius(b) + margin;
      if (diff.length() > maxDist)
        continue;
      pairs.push_back({i, j});
    }
  }
  return pairs;
}

} // namespace AvbdRef
