#pragma once
#include "avbd_contact_prep.h"
#include "avbd_solver.h"
#include <map>
#include <tuple>
#include <vector>

namespace AvbdRef {

struct SolverContactManifold {
  uint32_t bodyA = UINT32_MAX;
  uint32_t bodyB = UINT32_MAX;
  Vec3 normal = {0.0f, 1.0f, 0.0f};
  float friction = 0.5f;
  float restitution = 0.0f;
  std::vector<ContactPrep::ContactRow> rows;
};

struct CollisionOutput {
  std::vector<SolverContactManifold> manifolds;

  void clear() { manifolds.clear(); }

  size_t rowCount() const {
    size_t total = 0;
    for (const auto &m : manifolds)
      total += m.rows.size();
    return total;
  }

  void appendToSolver(Solver &solver) const {
    solver.contacts.reserve(solver.contacts.size() + rowCount());
    for (const auto &manifold : manifolds)
      for (const auto &row : manifold.rows)
        solver.addContact(row);
  }

  void applyToSolver(Solver &solver) const {
    solver.contacts.clear();
    appendToSolver(solver);
  }
};

struct ContactCache {
  struct Key {
    uint32_t bodyA, bodyB;
    int32_t nX, nY, nZ;
    int32_t rAx, rAy, rAz;
    int32_t rBx, rBy, rBz;

    bool operator<(const Key &o) const {
      return std::tie(bodyA, bodyB, nX, nY, nZ, rAx, rAy, rAz, rBx, rBy, rBz) <
             std::tie(o.bodyA, o.bodyB, o.nX, o.nY, o.nZ, o.rAx, o.rAy, o.rAz, o.rBx, o.rBy, o.rBz);
    }
  };
  struct Entry {
    ContactPrep::ContactCache solverCache;
  };

  std::map<Key, Entry> data;

  static int32_t quantize(float value, float scale) {
    return static_cast<int32_t>(value * scale + (value >= 0.0f ? 0.5f : -0.5f));
  }

  static Key makeKey(uint32_t bodyA, uint32_t bodyB, const Vec3 &normal,
                     const Vec3 &localA, const Vec3 &localB) {
    return {bodyA, bodyB,
            quantize(normal.x, 1000.0f), quantize(normal.y, 1000.0f), quantize(normal.z, 1000.0f),
            quantize(localA.x, 1000.0f), quantize(localA.y, 1000.0f), quantize(localA.z, 1000.0f),
            quantize(localB.x, 1000.0f), quantize(localB.y, 1000.0f), quantize(localB.z, 1000.0f)};
  }

  static Key makeKey(const Contact &c) {
    return makeKey(c.bodyA, c.bodyB, c.normal, c.rA, c.rB);
  }

  const ContactPrep::ContactCache *lookup(uint32_t bodyA, uint32_t bodyB,
                                          const Vec3 &normal,
                                          const Vec3 &localA,
                                          const Vec3 &localB) const {
    auto it = data.find(makeKey(bodyA, bodyB, normal, localA, localB));
    return it == data.end() ? nullptr : &it->second.solverCache;
  }

  void save(const Solver &solver) {
    data.clear();
    for (const auto &c : solver.contacts) {
      Entry e;
      for (int i = 0; i < 3; ++i) {
        e.solverCache.lambda[i] = c.lambda[i];
        e.solverCache.penalty[i] = c.penalty[i];
      }
      data[makeKey(c)] = e;
    }
  }

  void restore(Solver &solver) const {
    for (auto &c : solver.contacts) {
      auto it = data.find(makeKey(c));
      if (it == data.end())
        continue;
      for (int i = 0; i < 3; ++i) {
        c.lambda[i] = it->second.solverCache.lambda[i];
        c.penalty[i] = it->second.solverCache.penalty[i];
      }
    }
  }
};

inline void applyPersistentCache(CollisionOutput &output, const ContactCache *cache) {
  if (!cache)
    return;
  for (auto &manifold : output.manifolds) {
    for (auto &row : manifold.rows) {
      const ContactPrep::ContactCache *cached =
          cache->lookup(row.bodyA, row.bodyB, row.worldNormal, row.localAnchorA, row.localAnchorB);
      if (!cached)
        continue;
      for (int i = 0; i < 3; ++i) {
        row.lambda[i] = cached->lambda[i];
        row.penalty[i] = cached->penalty[i];
      }
    }
  }
}

} // namespace AvbdRef
