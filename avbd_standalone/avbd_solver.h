#pragma once
#include "avbd_articulation.h"
#include "avbd_contact_prep.h"
#include "avbd_types.h"
#include <functional>
#include <vector>

namespace AvbdRef {

static constexpr float PENALTY_MIN = 1000.0f;
static constexpr float PENALTY_MAX = 1e9f;

struct TaskRange {
  uint32_t begin = 0;
  uint32_t end = 0;
};

struct TaskSystem {
  virtual ~TaskSystem() = default;
  virtual void parallelFor(TaskRange range,
                           const std::function<void(TaskRange)> &fn) = 0;
  virtual void wait() {}
};

struct InlineTaskSystem final : TaskSystem {
  void parallelFor(TaskRange range,
                   const std::function<void(TaskRange)> &fn) override {
    fn(range);
  }
};

struct Solver {
  Vec3 gravity = {0, -9.8f, 0};
  int iterations = 10;
  float alpha = 0.95f;              // stabilization
  float beta = 1000.0f;             // penalty growth rate
  float gamma = 0.99f;              // warmstart decay
  float penaltyScale = 0.25f;       // body-ground penalty floor
  float penaltyScaleDynDyn = 0.05f; // dynamic-dynamic penalty
  int propagationDepth = 4;         // graph-propagation depth
  float propagationDecay = 0.5f;    // per-edge decay factor
  float dt = 1.0f / 60.0f;
  bool use3x3Solve = false; // false=6x6 LDLT (default), true=block-elim 3x3
  bool verbose = false;     // per-iteration logging

  // Phase 3: Convergence acceleration
  bool useTreeSweep = false;             // tree-structured sweep ordering for artic chains
  bool useAndersonAccel = false;         // Anderson Acceleration on body positions
  int aaWindowSize = 3;                  // AA window size (m)
  bool useChebyshev = false;             // Chebyshev semi-iterative position relaxation
  float chebyshevSpectralRadius = 0.92f; // estimated spectral radius

  // Per-step convergence history (populated if articulations present)
  std::vector<float> convergenceHistory;

  std::vector<Body> bodies;
  std::vector<Contact> contacts;
  std::vector<D6Joint> d6Joints;     // unified: all joint types
  std::vector<GearJoint> gearJoints; // kept separate (velocity constraint)
  std::vector<Articulation> articulations; // pure AVBD articulations (AL constraints)

  struct IslandData {
    std::vector<uint32_t> bodies;
    std::vector<uint32_t> contacts;
    std::vector<uint32_t> d6Joints;
    std::vector<uint32_t> gearJoints;
    std::vector<uint32_t> articulations;
    std::vector<uint32_t> articulationBodies;
  };

  struct ConstraintGraph {
    struct Node {
      enum Type : uint8_t { ContactNode, D6Node, GearNode, ArticulationJointNode, MimicNode, IKNode };
      Type type = ContactNode;
      uint32_t index0 = 0;
      uint32_t index1 = 0;
      std::vector<uint32_t> bodies;
    };

    std::vector<Node> nodes;
    std::vector<std::vector<uint32_t>> colors;
  };

  struct PipelineBuffers {
    std::vector<std::vector<uint32_t>> adjacency;
    std::vector<float> propagatedMass;
    std::vector<IslandData> islands;
    std::vector<std::vector<uint32_t>> contactBatches;
    std::vector<ConstraintGraph> constraintGraphs;
    std::vector<std::vector<uint32_t>> sweepOrders;
  };

  TaskSystem *taskSystem = nullptr;
  InlineTaskSystem inlineTaskSystem;
  PipelineBuffers pipeline;

  // Joint creation (all return index into d6Joints)
  uint32_t addSphericalJoint(uint32_t bodyA, uint32_t bodyB,
                             Vec3 localAnchorA, Vec3 localAnchorB,
                             float rho_ = 1e6f);
  uint32_t addFixedJoint(uint32_t bodyA, uint32_t bodyB,
                         Vec3 localAnchorA, Vec3 localAnchorB,
                         float rho_ = 1e6f);
  uint32_t addD6Joint(uint32_t bodyA, uint32_t bodyB,
                      Vec3 anchorA, Vec3 anchorB,
                      uint32_t linearMotion_ = 0,
                      uint32_t angularMotion_ = 0x2A,
                      float angularDamping_ = 0.0f, float rho_ = 1e6f);
  uint32_t addRevoluteJoint(uint32_t bodyA, uint32_t bodyB,
                            Vec3 localAnchorA, Vec3 localAnchorB,
                            Vec3 localAxisA,
                            Vec3 localAxisB = Vec3(0, 0, 1),
                            float rho = 1e6f);
  uint32_t addPrismaticJoint(uint32_t bodyA, uint32_t bodyB,
                             Vec3 localAnchorA, Vec3 localAnchorB,
                             Vec3 localAxisA, float rho = 1e6f);

  void setTaskSystem(TaskSystem *tasks) { taskSystem = tasks; }
  TaskSystem &tasks() { return taskSystem ? *taskSystem : inlineTaskSystem; }

  // Cone limit (spherical joints)
  void setSphericalJointConeLimit(uint32_t jointIdx, Vec3 coneAxisA,
                                  float limitAngle);

  // Revolute limit/drive (operates on d6Joints by index)
  void setRevoluteJointLimit(uint32_t jointIdx, float lowerLimit,
                             float upperLimit);
  void setRevoluteJointDrive(uint32_t jointIdx, float targetVelocity,
                             float maxForce);

  // Prismatic limit/drive (operates on d6Joints by index)
  void setPrismaticJointLimit(uint32_t jointIdx, float lowerLimit,
                              float upperLimit);
  void setPrismaticJointDrive(uint32_t jointIdx, float targetVelocity,
                              float damping);

  // Gear joint (separate)
  void addGearJoint(uint32_t bodyA, uint32_t bodyB,
                    Vec3 axisA, Vec3 axisB,
                    float ratio = -1.f, float rho = 1e5f);

  // Body creation
  uint32_t addBody(Vec3 pos, Quat rot, Vec3 halfExtent, float density,
                   float fric = 0.5f);

  // Contact creation
  void addContact(uint32_t bodyA, uint32_t bodyB, Vec3 normal, Vec3 rA,
                  Vec3 rB, float depth, float fric = 0.5f);
  void addContact(const ContactPrep::ContactRow &row);

  // Solver core
  void computeConstraint(Contact &c);
  void computeC0(Contact &c);
  void warmstart();
  void stagePredictionAndInertia(float invDt, float dt2);
  void stageBuildAdjacency();
  void stagePropagateMass(float dt2);
  void stageComputeContactC0();
  void stageBuildIslands();
  void stageBuildContactBatches();
  void stageBuildConstraintGraphs();
  void stageBuildSweepOrders();
  void stagePrimalSolve(float invDt, float dt2);
  void stageDualUpdate(float dt2);
  void stageVelocityWriteback(float invDt);
  void applyPostSolveMotors(float invDt, float dt2);
  void step(float dt_);
};

} // namespace AvbdRef
