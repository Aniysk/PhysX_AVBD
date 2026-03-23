#pragma once

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace AvbdRef {

struct TaskRange {
  uint32_t begin = 0;
  uint32_t end = 0;
};

struct AllocatorPolicy {
  virtual ~AllocatorPolicy() = default;
  virtual void *allocate(size_t size, size_t alignment = alignof(std::max_align_t)) = 0;
  virtual void deallocate(void *ptr) = 0;
};

struct MallocAllocator final : AllocatorPolicy {
  void *allocate(size_t size, size_t) override { return std::malloc(size); }
  void deallocate(void *ptr) override { std::free(ptr); }
};

struct ExecutionBackend {
  virtual ~ExecutionBackend() = default;
  virtual void parallelFor(TaskRange range, const std::function<void(TaskRange)> &fn) = 0;
  virtual void wait() {}
};

struct InlineExecutionBackend final : ExecutionBackend {
  void parallelFor(TaskRange range, const std::function<void(TaskRange)> &fn) override { fn(range); }
};

struct DeterminismFlags {
  enum Enum : uint32_t {
    eNONE = 0,
    eSORT_CONSTRAINTS = 1u << 0,
    eSORT_ISLANDS = 1u << 1,
    eSORT_BODIES = 1u << 2,
    eSERIAL_EXECUTION = 1u << 3,
    eDETERMINISTIC_DEFAULT = eSORT_CONSTRAINTS | eSORT_ISLANDS | eSORT_BODIES | eSERIAL_EXECUTION
  };
};

struct ExecutionFlags {
  enum Enum : uint32_t {
    eNONE = 0,
    eENABLE_SLEEPING = 1u << 0,
    eENABLE_PROFILING = 1u << 1,
    eCOLLECT_STATISTICS = 1u << 2,
    eDEFAULT = eENABLE_SLEEPING | eCOLLECT_STATISTICS
  };
};

struct RuntimeConfig {
  uint32_t determinismFlags = DeterminismFlags::eNONE;
  uint32_t executionFlags = ExecutionFlags::eDEFAULT;
  float sleepLinearThreshold = 0.05f;
  float sleepAngularThreshold = 0.05f;
  float sleepTimeThreshold = 0.5f;

  bool hasDeterminismFlag(DeterminismFlags::Enum flag) const {
    return (determinismFlags & static_cast<uint32_t>(flag)) != 0;
  }
  bool hasExecutionFlag(ExecutionFlags::Enum flag) const {
    return (executionFlags & static_cast<uint32_t>(flag)) != 0;
  }
  void enableDeterministicMode() {
    determinismFlags = DeterminismFlags::eDETERMINISTIC_DEFAULT;
  }
};

struct RuntimeStatistics {
  uint32_t numBodies = 0;
  uint32_t numContacts = 0;
  uint32_t numJoints = 0;
  uint32_t numIslands = 0;
  uint32_t numSleepingBodies = 0;
  uint32_t numActiveBodies = 0;
  uint32_t totalIterations = 0;

  void reset();
};

struct RuntimeServices {
  AllocatorPolicy *allocator = nullptr;
  ExecutionBackend *executor = nullptr;
  RuntimeConfig config;
  RuntimeStatistics stats;
};

class Runtime {
public:
  Runtime();

  void setAllocator(AllocatorPolicy *allocator);
  void setExecutor(ExecutionBackend *executor);

  AllocatorPolicy &allocator();
  ExecutionBackend &executor();

  RuntimeConfig &config() { return mServices.config; }
  const RuntimeConfig &config() const { return mServices.config; }
  RuntimeStatistics &stats() { return mServices.stats; }
  const RuntimeStatistics &stats() const { return mServices.stats; }
  RuntimeServices &services() { return mServices; }
  const RuntimeServices &services() const { return mServices; }

private:
  MallocAllocator mDefaultAllocator;
  InlineExecutionBackend mInlineExecutor;
  RuntimeServices mServices;
};

} // namespace AvbdRef
