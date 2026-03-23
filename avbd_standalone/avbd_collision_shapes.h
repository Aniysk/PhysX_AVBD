#pragma once
#include "avbd_contact_prep.h"
#include "avbd_solver.h"
#include <vector>

namespace AvbdRef {

struct CollisionShape {
  enum class Type : uint8_t { Box, GroundPlane };

  Type type = Type::Box;
  uint32_t bodyIndex = UINT32_MAX;
  Vec3 position = {};
  Quat rotation = {};
  Vec3 halfExtent = {};
  Vec3 planeNormal = {0.0f, 1.0f, 0.0f};
  float planeOffset = 0.0f;
  float dynamicFriction = 0.5f;
  float restitution = 0.0f;

  bool isDynamic() const { return bodyIndex != UINT32_MAX; }
};

struct CollisionShapeSet {
  std::vector<CollisionShape> shapes;
  int groundShapeIndex = -1;
};

inline CollisionShapeSet buildStandaloneCollisionShapes(const Solver &solver,
                                                        bool includeGround = true) {
  CollisionShapeSet set;
  if (includeGround) {
    CollisionShape ground;
    ground.type = CollisionShape::Type::GroundPlane;
    ground.bodyIndex = UINT32_MAX;
    ground.planeNormal = Vec3(0.0f, 1.0f, 0.0f);
    ground.planeOffset = 0.0f;
    set.groundShapeIndex = 0;
    set.shapes.push_back(ground);
  }

  for (uint32_t i = 0; i < solver.bodies.size(); ++i) {
    const Body &body = solver.bodies[i];
    CollisionShape shape;
    shape.type = CollisionShape::Type::Box;
    shape.bodyIndex = i;
    shape.position = body.position;
    shape.rotation = body.rotation;
    shape.halfExtent = body.halfExtent;
    shape.dynamicFriction = body.friction;
    set.shapes.push_back(shape);
  }
  return set;
}

} // namespace AvbdRef
