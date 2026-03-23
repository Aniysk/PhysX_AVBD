#pragma once
#include "avbd_collision_cache.h"
#include <map>
#include <vector>

namespace AvbdRef {

// =============================================================================
// Helper: Generate box-on-ground contacts
// =============================================================================
inline void addBoxGroundContacts(Solver &solver, uint32_t boxIdx,
                                 Vec3 halfExt) {
  Body &box = solver.bodies[boxIdx];

  float hx = halfExt.x, hy = halfExt.y, hz = halfExt.z;
  Vec3 corners[4] = {
      {-hx, -hy, -hz},
      {hx, -hy, -hz},
      {hx, -hy, hz},
      {-hx, -hy, hz},
  };

  Vec3 normal(0, 1, 0);

  for (int i = 0; i < 4; i++) {
    Vec3 worldCorner = box.position + box.rotation.rotate(corners[i]);
    float depth = -worldCorner.y; // positive when penetrating
    Vec3 groundPoint(worldCorner.x, 0, worldCorner.z);
    solver.addContact(boxIdx, UINT32_MAX, normal, corners[i], groundPoint,
                      depth, box.friction);
  }
}

// =============================================================================
// Helper: Generate box-on-box contacts
// =============================================================================
inline void addBoxOnBoxContacts(Solver &solver, uint32_t topIdx,
                                uint32_t bottomIdx, Vec3 halfExtTop,
                                Vec3 halfExtBot) {
  Body &top = solver.bodies[topIdx];
  Body &bot = solver.bodies[bottomIdx];

  float hx = halfExtTop.x, hy = halfExtTop.y, hz = halfExtTop.z;
  Vec3 corners[4] = {
      {-hx, -hy, -hz},
      {hx, -hy, -hz},
      {hx, -hy, hz},
      {-hx, -hy, hz},
  };

  Vec3 normal(0, 1, 0); // from bottom body to top body

  for (int i = 0; i < 4; i++) {
    Vec3 worldCornerA = top.position + top.rotation.rotate(corners[i]);
    float topOfBot = bot.position.y + halfExtBot.y;
    float depth = topOfBot - worldCornerA.y;

    Vec3 rA = corners[i];
    Vec3 worldContact(worldCornerA.x, topOfBot, worldCornerA.z);
    Vec3 rB = worldContact - bot.position;

    float fric = sqrtf(top.friction * bot.friction);
    solver.addContact(topIdx, bottomIdx, normal, rA, rB, depth, fric);
  }
}

// =============================================================================
// Dynamic contact generation (with proximity check)
// =============================================================================
inline void addBoxGroundContactsDynamic(Solver &solver, uint32_t boxIdx,
                                        Vec3 halfExt, float margin = 0.1f) {
  Body &box = solver.bodies[boxIdx];
  float hx = halfExt.x, hy = halfExt.y, hz = halfExt.z;
  Vec3 corners[4] = {
      {-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, -hy, hz}, {-hx, -hy, hz}};
  Vec3 normal(0, 1, 0);
  for (int i = 0; i < 4; i++) {
    Vec3 wc = box.position + box.rotation.rotate(corners[i]);
    if (wc.y > margin)
      continue;
    float depth = -wc.y;
    Vec3 gp(wc.x, 0, wc.z);
    solver.addContact(boxIdx, UINT32_MAX, normal, corners[i], gp, depth,
                      box.friction);
  }
}

} // namespace AvbdRef
