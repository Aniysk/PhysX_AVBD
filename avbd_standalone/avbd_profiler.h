#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace AvbdRef {

struct ProfileEvent {
  uint64_t callCount = 0;
  double totalMs = 0.0;
};

class Profiler {
public:
  void resetFrame() { mFrameEvents.clear(); }
  void addSample(const char *name, double ms) {
    auto &event = mFrameEvents[name];
    event.callCount++;
    event.totalMs += ms;
  }

  const std::unordered_map<std::string, ProfileEvent> &frameEvents() const {
    return mFrameEvents;
  }

private:
  std::unordered_map<std::string, ProfileEvent> mFrameEvents;
};

class ProfileScope {
public:
  ProfileScope(Profiler *profiler, const char *name)
      : mProfiler(profiler), mName(name), mStart(std::chrono::steady_clock::now()) {}
  ~ProfileScope() {
    if (!mProfiler)
      return;
    const auto end = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end - mStart).count();
    mProfiler->addSample(mName, ms);
  }

private:
  Profiler *mProfiler;
  const char *mName;
  std::chrono::steady_clock::time_point mStart;
};

} // namespace AvbdRef
