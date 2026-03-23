#pragma once
#include "avbd_articulation.h"
#include "avbd_contact_prep.h"
#include "avbd_types.h"
#include <array>
#include <vector>

namespace AvbdRef {
namespace SoA {

static constexpr uint32_t PACK_WIDTH = 4;

struct BodyStorage {
  std::vector<Vec3> position;
  std::vector<Quat> rotation;
  std::vector<Vec3> linearVelocity;
  std::vector<Vec3> angularVelocity;
  std::vector<Vec3> prevLinearVelocity;
  std::vector<Vec3> initialPosition;
  std::vector<Quat> initialRotation;
  std::vector<Vec3> inertialPosition;
  std::vector<Quat> inertialRotation;
  std::vector<float> mass;
  std::vector<float> invMass;
  std::vector<Mat33> inertiaTensor;
  std::vector<Mat33> invInertiaWorld;
  std::vector<float> friction;
  std::vector<Vec3> halfExtent;
  std::vector<float> linearDamping;
  std::vector<float> angularDamping;
  std::vector<float> maxLinearVelocity;
  std::vector<float> maxAngularVelocity;

  void clear();
  void resize(size_t count);
  void buildFromBodies(const std::vector<Body> &bodies);
  void scatterToBodies(std::vector<Body> &bodies) const;
  void scatterBodyToAoS(std::vector<Body> &bodies, uint32_t bodyIndex) const;
  Mat66 getMassMatrix(uint32_t bodyIndex) const;
  Vec3 deltaWInitial(uint32_t bodyIndex) const;
  Vec3 deltaWInertial(uint32_t bodyIndex) const;
  void updateInvInertiaWorld(uint32_t bodyIndex);
};

struct ContactStorage {
  std::vector<uint32_t> bodyA;
  std::vector<uint32_t> bodyB;
  std::vector<Vec3> normal;
  std::vector<Vec3> tangent0;
  std::vector<Vec3> tangent1;
  std::vector<Vec3> anchorA;
  std::vector<Vec3> anchorB;
  std::vector<float> depth;
  std::vector<float> friction;
  std::vector<std::array<float, 3>> C;
  std::vector<std::array<float, 3>> C0;
  std::vector<std::array<float, 3>> fmin;
  std::vector<std::array<float, 3>> fmax;
  std::vector<std::array<float, 3>> lambda;
  std::vector<std::array<float, 3>> penalty;

  void clear();
  void resize(size_t count);
  void buildFromContacts(const std::vector<Contact> &contacts);
  void scatterToContacts(std::vector<Contact> &contacts) const;
};

struct JointRowMetadataStorage {
  std::vector<uint32_t> bodyA;
  std::vector<uint32_t> bodyB;
  std::vector<uint8_t> type;
  std::vector<uint8_t> rowCount;

  void clear();
  void buildFromScene(const std::vector<D6Joint> &d6Joints,
                      const std::vector<GearJoint> &gearJoints,
                      const std::vector<Articulation> &articulations);
};

struct PackedContactBatch4 {
  uint32_t count = 0;
  std::array<uint32_t, PACK_WIDTH> indices = {UINT32_MAX, UINT32_MAX, UINT32_MAX,
                                              UINT32_MAX};
};

struct SolverStorage {
  BodyStorage bodies;
  ContactStorage contacts;
  JointRowMetadataStorage jointRows;
  std::vector<std::vector<PackedContactBatch4>> packedContactBatches;

  void buildFromScene(const std::vector<Body> &aosBodies,
                      const std::vector<Contact> &aosContacts,
                      const std::vector<D6Joint> &d6Joints,
                      const std::vector<GearJoint> &gearJoints,
                      const std::vector<Articulation> &articulations);
  void scatterToScene(std::vector<Body> &aosBodies,
                      std::vector<Contact> &aosContacts) const;
  void rebuildPackedContactBatches(
      const std::vector<std::vector<uint32_t>> &contactBatches);
};

} // namespace SoA
} // namespace AvbdRef
