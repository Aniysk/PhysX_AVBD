#include "avbd_runtime.h"

namespace AvbdRef {

void RuntimeStatistics::reset() {
  numBodies = 0;
  numContacts = 0;
  numJoints = 0;
  numIslands = 0;
  numSleepingBodies = 0;
  numActiveBodies = 0;
  totalIterations = 0;
}

Runtime::Runtime() {
  mServices.allocator = &mDefaultAllocator;
  mServices.executor = &mInlineExecutor;
}

void Runtime::setAllocator(AllocatorPolicy *allocator) {
  mServices.allocator = allocator ? allocator : &mDefaultAllocator;
}

void Runtime::setExecutor(ExecutionBackend *executor) {
  mServices.executor = executor ? executor : &mInlineExecutor;
}

AllocatorPolicy &Runtime::allocator() { return *mServices.allocator; }
ExecutionBackend &Runtime::executor() { return *mServices.executor; }

} // namespace AvbdRef
