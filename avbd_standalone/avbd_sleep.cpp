#include "avbd_sleep.h"
#include "avbd_runtime.h"
#include "avbd_types.h"
#include <cmath>

namespace AvbdRef {

void SleepSystem::resize(size_t count) {
  if (mStates.size() < count)
    mStates.resize(count);
  else if (mStates.size() > count)
    mStates.resize(count);
}

void SleepSystem::beginStep(const std::vector<Body> &bodies) {
  resize(bodies.size());
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].mass <= 0.0f) {
      mStates[i].asleep = false;
      mStates[i].idleTime = 0.0f;
    }
  }
}

void SleepSystem::endStep(const std::vector<Body> &bodies, float dt, const RuntimeConfig &config) {
  resize(bodies.size());
  for (size_t i = 0; i < bodies.size(); ++i) {
    auto &state = mStates[i];
    if (bodies[i].mass <= 0.0f) {
      state.asleep = false;
      state.idleTime = 0.0f;
      continue;
    }
    const float linSpeed = bodies[i].linearVelocity.length();
    const float angSpeed = bodies[i].angularVelocity.length();
    if (linSpeed <= config.sleepLinearThreshold && angSpeed <= config.sleepAngularThreshold) {
      state.idleTime += dt;
      state.asleep = state.idleTime >= config.sleepTimeThreshold;
    } else {
      state.asleep = false;
      state.idleTime = 0.0f;
    }
  }
}

bool SleepSystem::isSleeping(uint32_t index) const {
  return index < mStates.size() ? mStates[index].asleep : false;
}

void SleepSystem::wake(uint32_t index) {
  if (index >= mStates.size())
    return;
  mStates[index].asleep = false;
  mStates[index].idleTime = 0.0f;
}

uint32_t SleepSystem::sleepingCount() const {
  uint32_t count = 0;
  for (const auto &state : mStates)
    count += state.asleep ? 1u : 0u;
  return count;
}

} // namespace AvbdRef
