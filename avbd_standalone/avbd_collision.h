#pragma once
#include "avbd_collision_narrowphase.h"

namespace AvbdRef {

struct CollisionPipeline {
  float margin = 0.02f;
  bool includeGround = true;

  CollisionOutput run(const Solver &solver, const ContactCache *cache = nullptr) const {
    CollisionShapeSet shapes = buildStandaloneCollisionShapes(solver, includeGround);
    std::vector<BroadPhasePair> pairs = generateBroadPhasePairs(shapes, margin);
    CollisionOutput output;
    output.manifolds.reserve(pairs.size());
    for (const BroadPhasePair &pair : pairs)
      generateContactManifold(shapes, pair, margin, output);
    applyPersistentCache(output, cache);
    return output;
  }

  void generateIntoSolver(Solver &solver, const ContactCache *cache = nullptr) const {
    CollisionOutput output = run(solver, cache);
    output.appendToSolver(solver);
  }
};

inline int collideBoxGround(Solver &solver, uint32_t boxIdx, float margin = 0.02f) {
  CollisionShapeSet shapes;
  CollisionShape ground;
  ground.type = CollisionShape::Type::GroundPlane;
  shapes.groundShapeIndex = 0;
  shapes.shapes.push_back(ground);
  const Body &body = solver.bodies[boxIdx];
  CollisionShape box;
  box.type = CollisionShape::Type::Box;
  box.bodyIndex = boxIdx;
  box.position = body.position;
  box.rotation = body.rotation;
  box.halfExtent = body.halfExtent;
  box.dynamicFriction = body.friction;
  shapes.shapes.push_back(box);
  CollisionOutput output;
  generateContactManifold(shapes, {1, 0}, margin, output);
  output.appendToSolver(solver);
  return static_cast<int>(output.rowCount());
}

inline int collideBoxBox(Solver &solver, uint32_t idxA, uint32_t idxB, float margin = 0.02f) {
  CollisionShapeSet shapes;
  const Body &bodyA = solver.bodies[idxA];
  const Body &bodyB = solver.bodies[idxB];
  CollisionShape a, b;
  a.type = CollisionShape::Type::Box; a.bodyIndex = idxA; a.position = bodyA.position; a.rotation = bodyA.rotation; a.halfExtent = bodyA.halfExtent; a.dynamicFriction = bodyA.friction;
  b.type = CollisionShape::Type::Box; b.bodyIndex = idxB; b.position = bodyB.position; b.rotation = bodyB.rotation; b.halfExtent = bodyB.halfExtent; b.dynamicFriction = bodyB.friction;
  shapes.shapes.push_back(a); shapes.shapes.push_back(b);
  CollisionOutput output;
  generateContactManifold(shapes, {0, 1}, margin, output);
  output.appendToSolver(solver);
  return static_cast<int>(output.rowCount());
}

inline int collideAll(Solver &solver, float margin = 0.02f) {
  CollisionPipeline pipeline{margin, true};
  CollisionOutput output = pipeline.run(solver, nullptr);
  output.applyToSolver(solver);
  return static_cast<int>(output.rowCount());
}

} // namespace AvbdRef
