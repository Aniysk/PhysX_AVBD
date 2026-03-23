#pragma once

#include <cstdint>
#include <vector>

namespace AvbdRef {

struct Body;
struct RuntimeConfig;

struct SleepState {
  bool asleep = false;
  float idleTime = 0.0f;
};

class SleepSystem {
public:
  void resize(size_t count);
  void beginStep(const std::vector<Body> &bodies);
  void endStep(const std::vector<Body> &bodies, float dt, const RuntimeConfig &config);
  bool isSleeping(uint32_t index) const;
  void wake(uint32_t index);
  uint32_t sleepingCount() const;

private:
  std::vector<SleepState> mStates;
};

} // namespace AvbdRef
