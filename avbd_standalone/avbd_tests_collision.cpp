#include "avbd_collision.h"
#include "avbd_benchmarks.h"
#include "avbd_test_utils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>


using namespace AvbdRef;

namespace {

struct BenchmarkTaskSystem final : TaskSystem {
  uint32_t workerCount;
  explicit BenchmarkTaskSystem(uint32_t count) : workerCount(count ? count : 1u) {}

  void parallelFor(TaskRange range, const std::function<void(TaskRange)> &fn) override {
    const uint32_t count = range.end > range.begin ? (range.end - range.begin) : 0u;
    if (workerCount <= 1 || count <= 1) {
      fn(range);
      return;
    }
    const uint32_t chunks = std::min(workerCount, count);
    std::vector<std::thread> threads;
    threads.reserve(chunks);
    for (uint32_t i = 0; i < chunks; ++i) {
      const uint32_t begin = range.begin + (count * i) / chunks;
      const uint32_t end = range.begin + (count * (i + 1)) / chunks;
      threads.emplace_back([=, &fn]() {
        if (begin < end)
          fn({begin, end});
      });
    }
    for (auto &thread : threads)
      thread.join();
  }
};

template <typename SetupFn, typename PreStepFn>
AvbdBenchmarkRecord runBenchmarkScenario(const char *name, const char *family,
                                         const char *execution,
                                         SetupFn setupFn, PreStepFn preStepFn,
                                         uint32_t warmupFrames = 60,
                                         uint32_t sampleFrames = 180,
                                         uint32_t taskWorkers = 1) {
  Solver solver;
  solver.gravity = {0, -9.81f, 0};
  solver.dt = 1.0f / 60.0f;
  solver.iterations = 20;
  BenchmarkTaskSystem taskSystem(taskWorkers);
  if (taskWorkers > 1)
    solver.setTaskSystem(&taskSystem);

  ContactCache cache;
  setupFn(solver);

  for (uint32_t frame = 0; frame < warmupFrames; ++frame) {
    solver.contacts.clear();
    preStepFn(solver, frame, cache);
    solver.step(solver.dt);
    cache.save(solver);
  }

  AvbdBenchmarkRecord record;
  record.name = name;
  record.family = family;
  record.execution = execution;
  double frameMs = 0.0;
  for (uint32_t frame = 0; frame < sampleFrames; ++frame) {
    solver.contacts.clear();
    preStepFn(solver, frame, cache);
    auto t0 = std::chrono::steady_clock::now();
    solver.step(solver.dt);
    frameMs += std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
    cache.save(solver);
    const auto &stats = solver.lastStepStats;
    record.stageBuildIslandsMs += stats.stageBuildIslandsMs;
    record.stagePrimalSolveMs += stats.stagePrimalSolveMs;
    record.stageDualUpdateMs += stats.stageDualUpdateMs;
    record.writebackMs += stats.writebackMs;
    record.aosToSoABuildMs += stats.aosToSoABuildMs;
    record.soaScatterMs += stats.soaScatterMs;
    record.constraintCount += stats.constraintCount;
    record.iterationCount += stats.primalIterations;
  }

  const double invSamples = 1.0 / double(sampleFrames);
  record.msPerFrame = frameMs * invSamples;
  record.stageBuildIslandsMs *= invSamples;
  record.stagePrimalSolveMs *= invSamples;
  record.stageDualUpdateMs *= invSamples;
  record.writebackMs *= invSamples;
  record.aosToSoABuildMs *= invSamples;
  record.soaScatterMs *= invSamples;
  record.constraintCount = uint32_t(double(record.constraintCount) * invSamples);
  record.iterationCount = uint32_t(double(record.iterationCount) * invSamples);
  return record;
}

void printBenchmarkTable(const std::vector<AvbdBenchmarkRecord> &records) {
  printf("\n=== AVBD Standalone Benchmarks ===\n");
  printf("%-24s %-12s %10s %12s %10s %10s %10s %10s %10s %10s\n",
         "workload", "exec", "ms/frame", "constraints", "iters",
         "islands", "primal", "dual", "writeback", "build+scat");
  for (const auto &r : records) {
    printf("%-24s %-12s %10.3f %12u %10u %10.3f %10.3f %10.3f %10.3f %10.3f\n",
           r.name.c_str(), r.execution.c_str(), r.msPerFrame, r.constraintCount,
           r.iterationCount, r.stageBuildIslandsMs, r.stagePrimalSolveMs,
           r.stageDualUpdateMs, r.writebackMs,
           r.aosToSoABuildMs + r.soaScatterMs);
  }
}

} // namespace

bool runStandaloneBenchmarks(int argc, const char *const *argv) {
  bool run = false;
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--bench")
      run = true;
  if (!run)
    return false;

  std::vector<AvbdBenchmarkRecord> records;
  registerCollisionBenchmarks(records);
  registerJointBenchmarks(records);
  registerArticulationBenchmarks(records);
  printBenchmarkTable(records);
  return true;
}

void registerCollisionBenchmarks(std::vector<AvbdBenchmarkRecord> &out) {
  auto addRuns = [&](const char *name, auto setup, auto preStep) {
    out.push_back(runBenchmarkScenario(name, "collision", "single-thread", setup,
                                       preStep, 60, 180, 1));
    out.push_back(runBenchmarkScenario(name, "collision", "task-system", setup,
                                       preStep, 60, 180, 4));
  };

  addRuns("box stacks",
          [](Solver &solver) {
            Vec3 halfExt(1, 1, 1);
            for (int i = 0; i < 10; ++i)
              for (int j = 0; j < 10 - i; ++j)
                solver.addBody({float(j * 2 - (10 - i)) * halfExt.x,
                                float(i * 2 + 1) * halfExt.y, 0},
                               Quat(), halfExt, 10.0f, 0.5f);
          },
          [](Solver &solver, uint32_t, ContactCache &cache) {
            collideAll(solver, 0.05f);
            cache.restore(solver);
          });

  addRuns("dense contact piles",
          [](Solver &solver) {
            Vec3 halfExt(0.6f, 0.6f, 0.6f);
            for (int y = 0; y < 8; ++y)
              for (int x = 0; x < 8; ++x)
                for (int z = 0; z < 8; ++z)
                  solver.addBody({(x - 4) * 1.05f, 1.0f + y * 1.05f, (z - 4) * 1.05f},
                                 Quat(), halfExt, 8.0f, 0.7f);
          },
          [](Solver &solver, uint32_t, ContactCache &cache) {
            collideAll(solver, 0.08f);
            cache.restore(solver);
          });
}

extern int gTestsPassed;
extern int gTestsFailed;

#define CHECK(cond, msg, ...)                                                  \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("  FAIL: " msg "\n", ##__VA_ARGS__);                              \
      gTestsFailed++;                                                          \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define PASS(msg)                                                              \
  do {                                                                         \
    printf("  PASS: %s\n", msg);                                               \
    gTestsPassed++;                                                            \
    return true;                                                               \
  } while (0)

bool test11_collisionSingleBox() {
  printf("\n--- Test 11: Collision-detected single box on ground ---\n");
  Solver solver;
  solver.gravity = {0, -9.8f, 0};
  solver.iterations = 10;
  solver.dt = 1.0f / 60.0f;
  Vec3 halfExt(1, 1, 1);
  uint32_t box = solver.addBody({0, 1, 0}, Quat(), halfExt, 10.0f, 0.5f);
  ContactCache cache;
  for (int frame = 0; frame < 120; frame++) {
    solver.contacts.clear();
    collideBoxGround(solver, box, 0.05f);
    cache.restore(solver);
    solver.step(solver.dt);
    cache.save(solver);
  }
  float finalY = solver.bodies[box].position.y;
  CHECK(fabsf(finalY - 1.0f) < 0.1f, "collision box drifted: y=%.4f", finalY);
  PASS("collision single box stable");
}

bool test12_collisionThreeStack() {
  printf("\n--- Test 12: Collision-detected 3-box stack ---\n");
  Solver solver;
  solver.gravity = {0, -9.8f, 0};
  solver.iterations = 10;
  solver.dt = 1.0f / 60.0f;
  Vec3 halfExt(1, 1, 1);
  float density = 10.0f;
  uint32_t b0 = solver.addBody({0, 1, 0}, Quat(), halfExt, density, 0.5f);
  uint32_t b1 = solver.addBody({0, 3, 0}, Quat(), halfExt, density, 0.5f);
  uint32_t b2 = solver.addBody({0, 5, 0}, Quat(), halfExt, density, 0.5f);
  ContactCache cache;
  for (int frame = 0; frame < 180; frame++) {
    solver.contacts.clear();
    collideAll(solver, 0.05f);
    cache.restore(solver);
    solver.step(solver.dt);
    cache.save(solver);
  }
  CHECK(fabsf(solver.bodies[b0].position.y - 1.0f) < 0.15f, "b0 drifted");
  CHECK(fabsf(solver.bodies[b1].position.y - 3.0f) < 0.15f, "b1 drifted");
  CHECK(fabsf(solver.bodies[b2].position.y - 5.0f) < 0.15f, "b2 drifted");
  PASS("collision-detected 3-box stack stable");
}

bool test13_collisionDrop() {
  printf("\n--- Test 13: Collision drop + settle ---\n");
  Solver solver;
  solver.gravity = {0, -9.8f, 0};
  solver.iterations = 10;
  solver.dt = 1.0f / 60.0f;
  Vec3 halfExt(1, 1, 1);
  uint32_t box = solver.addBody({0, 4, 0}, Quat(), halfExt, 10.0f, 0.5f);
  ContactCache cache;
  for (int frame = 0; frame < 360; frame++) {
    solver.contacts.clear();
    collideBoxGround(solver, box, 0.1f);
    cache.restore(solver);
    solver.step(solver.dt);
    cache.save(solver);
  }
  float finalY = solver.bodies[box].position.y;
  CHECK(fabsf(finalY - 1.0f) < 0.15f, "didn't settle: y=%.4f", finalY);
  PASS("collision drop settled");
}

bool test14_collisionPhysxTower() {
  printf("\n--- Test 14: Collision PhysX-scale 5-box tower (stress) ---\n");
  Solver solver;
  solver.gravity = {0, -9.8f, 0};
  solver.iterations = 12;
  solver.dt = 1.0f / 60.0f;
  Vec3 halfExt(2, 2, 2);
  float density = 10.0f;
  const int N = 5;
  uint32_t ids[N];
  for (int i = 0; i < N; i++)
    ids[i] =
        solver.addBody({0, 2.0f + 4.0f * i, 0}, Quat(), halfExt, density, 0.5f);
  ContactCache cache;
  bool exploded = false;
  float maxDrift = 0;
  for (int frame = 0; frame < 600; frame++) {
    solver.contacts.clear();
    collideAll(solver, 0.05f);
    cache.restore(solver);
    solver.step(solver.dt);
    cache.save(solver);
    for (int i = 0; i < N; i++) {
      float expected = 2.0f + 4.0f * i;
      float drift = fabsf(solver.bodies[ids[i]].position.y - expected);
      if (drift > maxDrift)
        maxDrift = drift;
      if (fabsf(solver.bodies[ids[i]].position.y) > 200.0f)
        exploded = true;
    }
  }
  CHECK(!exploded, "EXPLOSION!");
  CHECK(maxDrift < 0.5f, "tower collapsed: maxDrift=%.4f", maxDrift);
  PASS("collision PhysX-scale 5-box tower stable");
}

bool test15_pyramidStack() {
  printf("\n--- Test 15: Pyramid stack (PhysX SnippetHelloWorld layout) ---\n");
  Solver solver;
  solver.gravity = {0, -9.81f, 0};
  solver.iterations = 10;
  solver.dt = 1.0f / 60.0f;
  Vec3 halfExt(2, 2, 2);
  float density = 10.0f;
  const int size = 10;
  std::vector<uint32_t> ids;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size - i; j++) {
      float x = (float)(j * 2 - (size - i)) * halfExt.x;
      float y = (float)(i * 2 + 1) * halfExt.y;
      ids.push_back(solver.addBody({x, y, 0}, Quat(), halfExt, density, 0.5f));
    }
  }
  ContactCache cache;
  bool exploded = false;
  for (int frame = 0; frame < 600; frame++) {
    solver.contacts.clear();
    collideAll(solver, 0.05f);
    cache.restore(solver);
    solver.step(solver.dt);
    cache.save(solver);
    for (size_t bi = 0; bi < ids.size(); bi++) {
      if (fabsf(solver.bodies[ids[bi]].position.y) > 200.0f)
        exploded = true;
    }
  }
  CHECK(!exploded, "EXPLOSION!");
  PASS("pyramid stack stable (PhysX layout)");
}

bool test16_pyramidNoFriction() {
  printf("\n--- Test 16: Pyramid stack (NO friction) ---\n");
  Solver solver;
  solver.gravity = {0, -9.81f, 0};
  solver.iterations = 10;
  solver.dt = 1.0f / 60.0f;
  Vec3 halfExt(2, 2, 2);
  float density = 10.0f;
  const int size = 10;
  std::vector<uint32_t> ids;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size - i; j++) {
      float x = (float)(j * 2 - (size - i)) * halfExt.x;
      float y = (float)(i * 2 + 1) * halfExt.y;
      ids.push_back(solver.addBody({x, y, 0}, Quat(), halfExt, density, 0.0f));
    }
  }
  ContactCache cache;
  bool exploded = false;
  for (int frame = 0; frame < 300; frame++) {
    solver.contacts.clear();
    collideAll(solver, 0.05f);
    cache.restore(solver);
    solver.step(solver.dt);
    cache.save(solver);
    for (size_t bi = 0; bi < ids.size(); bi++) {
      if (fabsf(solver.bodies[ids[bi]].position.y) > 200.0f)
        exploded = true;
    }
  }
  CHECK(!exploded, "EXPLOSION even without friction!");
  PASS("pyramid no-friction: collapsed but no explosion");
}
