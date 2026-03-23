#pragma once

#include "avbd_solver.h"
#include <string>
#include <vector>

struct AvbdBenchmarkRecord {
  std::string name;
  std::string family;
  std::string execution;
  double msPerFrame = 0.0;
  double stageBuildIslandsMs = 0.0;
  double stagePrimalSolveMs = 0.0;
  double stageDualUpdateMs = 0.0;
  double writebackMs = 0.0;
  double aosToSoABuildMs = 0.0;
  double soaScatterMs = 0.0;
  uint32_t constraintCount = 0;
  uint32_t iterationCount = 0;
};

bool runStandaloneBenchmarks(int argc, const char* const* argv);
void registerCollisionBenchmarks(std::vector<AvbdBenchmarkRecord>& out);
void registerJointBenchmarks(std::vector<AvbdBenchmarkRecord>& out);
void registerArticulationBenchmarks(std::vector<AvbdBenchmarkRecord>& out);
