#include "avbd_soa_storage.h"
#include "avbd_articulation.h"
#include <algorithm>

namespace AvbdRef {
namespace SoA {

void BodyStorage::clear() { *this = BodyStorage(); }
void BodyStorage::resize(size_t count) {
  position.resize(count); rotation.resize(count); linearVelocity.resize(count);
  angularVelocity.resize(count); prevLinearVelocity.resize(count);
  initialPosition.resize(count); initialRotation.resize(count);
  inertialPosition.resize(count); inertialRotation.resize(count);
  mass.resize(count); invMass.resize(count); inertiaTensor.resize(count);
  invInertiaWorld.resize(count); friction.resize(count); halfExtent.resize(count);
  linearDamping.resize(count); angularDamping.resize(count);
  maxLinearVelocity.resize(count); maxAngularVelocity.resize(count);
}
void BodyStorage::buildFromBodies(const std::vector<Body> &bodies) {
  resize(bodies.size());
  for (size_t i = 0; i < bodies.size(); ++i) {
    const Body &b = bodies[i]; position[i]=b.position; rotation[i]=b.rotation;
    linearVelocity[i]=b.linearVelocity; angularVelocity[i]=b.angularVelocity;
    prevLinearVelocity[i]=b.prevLinearVelocity; initialPosition[i]=b.initialPosition;
    initialRotation[i]=b.initialRotation; inertialPosition[i]=b.inertialPosition;
    inertialRotation[i]=b.inertialRotation; mass[i]=b.mass; invMass[i]=b.invMass;
    inertiaTensor[i]=b.inertiaTensor; invInertiaWorld[i]=b.invInertiaWorld; friction[i]=b.friction;
    halfExtent[i]=b.halfExtent; linearDamping[i]=b.linearDamping; angularDamping[i]=b.angularDamping;
    maxLinearVelocity[i]=b.maxLinearVelocity; maxAngularVelocity[i]=b.maxAngularVelocity;
  }
}
void BodyStorage::scatterToBodies(std::vector<Body> &bodies) const {
  for (uint32_t i = 0; i < bodies.size() && i < position.size(); ++i) scatterBodyToAoS(bodies, i);
}
void BodyStorage::scatterBodyToAoS(std::vector<Body> &bodies, uint32_t i) const {
  Body &b = bodies[i]; b.position=position[i]; b.rotation=rotation[i]; b.linearVelocity=linearVelocity[i];
  b.angularVelocity=angularVelocity[i]; b.prevLinearVelocity=prevLinearVelocity[i];
  b.initialPosition=initialPosition[i]; b.initialRotation=initialRotation[i];
  b.inertialPosition=inertialPosition[i]; b.inertialRotation=inertialRotation[i];
  b.mass=mass[i]; b.invMass=invMass[i]; b.inertiaTensor=inertiaTensor[i]; b.invInertiaWorld=invInertiaWorld[i];
  b.friction=friction[i]; b.halfExtent=halfExtent[i]; b.linearDamping=linearDamping[i];
  b.angularDamping=angularDamping[i]; b.maxLinearVelocity=maxLinearVelocity[i];
  b.maxAngularVelocity=maxAngularVelocity[i];
}
Mat66 BodyStorage::getMassMatrix(uint32_t i) const { Mat66 M; for (int r=0;r<3;r++) M.m[r][r]=mass[i]; for(int r=0;r<3;r++) for(int c=0;c<3;c++) M.m[3+r][3+c]=inertiaTensor[i].m[r][c]; return M; }
Vec3 BodyStorage::deltaWInitial(uint32_t i) const { Quat dq = rotation[i] * initialRotation[i].conjugate(); if (dq.w < 0) dq = dq * (-1.f); return Vec3(dq.x,dq.y,dq.z) * 2.0f; }
Vec3 BodyStorage::deltaWInertial(uint32_t i) const { Quat dq = rotation[i] * inertialRotation[i].conjugate(); if (dq.w < 0) dq = dq * (-1.f); return Vec3(dq.x,dq.y,dq.z) * 2.0f; }
void BodyStorage::updateInvInertiaWorld(uint32_t i) {
  if (mass[i] <= 0) return; Mat33 invIlocal = inertiaTensor[i].inverse(); Mat33 R;
  float qw=rotation[i].w,qx=rotation[i].x,qy=rotation[i].y,qz=rotation[i].z;
  R.m[0][0]=1-2*(qy*qy+qz*qz); R.m[0][1]=2*(qx*qy-qz*qw); R.m[0][2]=2*(qx*qz+qy*qw);
  R.m[1][0]=2*(qx*qy+qz*qw); R.m[1][1]=1-2*(qx*qx+qz*qz); R.m[1][2]=2*(qy*qz-qx*qw);
  R.m[2][0]=2*(qx*qz-qy*qw); R.m[2][1]=2*(qy*qz+qx*qw); R.m[2][2]=1-2*(qx*qx+qy*qy);
  Mat33 RI; for(int r=0;r<3;r++) for(int c=0;c<3;c++){ RI.m[r][c]=0; for(int k=0;k<3;k++) RI.m[r][c]+=R.m[r][k]*invIlocal.m[k][c]; }
  for(int r=0;r<3;r++) for(int c=0;c<3;c++){ invInertiaWorld[i].m[r][c]=0; for(int k=0;k<3;k++) invInertiaWorld[i].m[r][c]+=RI.m[r][k]*R.m[c][k]; }
}

void ContactStorage::clear() { *this = ContactStorage(); }
void ContactStorage::resize(size_t count) { bodyA.resize(count); bodyB.resize(count); normal.resize(count); tangent0.resize(count); tangent1.resize(count); anchorA.resize(count); anchorB.resize(count); depth.resize(count); friction.resize(count); C.resize(count); C0.resize(count); fmin.resize(count); fmax.resize(count); lambda.resize(count); penalty.resize(count); }
void ContactStorage::buildFromContacts(const std::vector<Contact> &contacts) { resize(contacts.size()); for(size_t i=0;i<contacts.size();++i){ const Contact &c=contacts[i]; bodyA[i]=c.bodyA; bodyB[i]=c.bodyB; normal[i]=c.normal; ContactPrep::computeTangents(normal[i], tangent0[i], tangent1[i]); anchorA[i]=c.rA; anchorB[i]=c.rB; depth[i]=c.depth; friction[i]=c.friction; for(int a=0;a<3;a++){ C[i][a]=c.C[a]; C0[i][a]=c.C0[a]; fmin[i][a]=c.fmin[a]; fmax[i][a]=c.fmax[a]; lambda[i][a]=c.lambda[a]; penalty[i][a]=c.penalty[a]; } } }
void ContactStorage::scatterToContacts(std::vector<Contact> &contacts) const { for(size_t i=0;i<contacts.size() && i<bodyA.size();++i){ auto &c=contacts[i]; c.bodyA=bodyA[i]; c.bodyB=bodyB[i]; c.normal=normal[i]; c.rA=anchorA[i]; c.rB=anchorB[i]; c.depth=depth[i]; c.friction=friction[i]; for(int a=0;a<3;a++){ c.C[a]=C[i][a]; c.C0[a]=C0[i][a]; c.fmin[a]=fmin[i][a]; c.fmax[a]=fmax[i][a]; c.lambda[a]=lambda[i][a]; c.penalty[a]=penalty[i][a]; } } }

void JointRowMetadataStorage::clear() { *this = JointRowMetadataStorage(); }
void JointRowMetadataStorage::buildFromScene(const std::vector<D6Joint> &d6Joints, const std::vector<GearJoint> &gearJoints, const std::vector<Articulation> &articulations) {
  clear();
  for (const auto &j : d6Joints) { bodyA.push_back(j.bodyA); bodyB.push_back(j.bodyB); type.push_back(0); rowCount.push_back(6); }
  for (const auto &g : gearJoints) { bodyA.push_back(g.bodyA); bodyB.push_back(g.bodyB); type.push_back(1); rowCount.push_back(1); }
  for (const auto &a : articulations) for (size_t ji=0; ji<a.joints.size(); ++ji) { bodyA.push_back(a.getParentBodyIndex((int)ji)); bodyB.push_back(a.joints[ji].bodyIndex); type.push_back(2); rowCount.push_back(6); }
}

void SolverStorage::buildFromScene(const std::vector<Body> &aosBodies,const std::vector<Contact> &aosContacts,const std::vector<D6Joint> &d6Joints,const std::vector<GearJoint> &gearJoints,const std::vector<Articulation> &articulations){ bodies.buildFromBodies(aosBodies); contacts.buildFromContacts(aosContacts); jointRows.buildFromScene(d6Joints, gearJoints, articulations); }
void SolverStorage::scatterToScene(std::vector<Body> &aosBodies,std::vector<Contact> &aosContacts) const { bodies.scatterToBodies(aosBodies); contacts.scatterToContacts(aosContacts); }
void SolverStorage::rebuildPackedContactBatches(const std::vector<std::vector<uint32_t>> &contactBatches){ packedContactBatches.assign(contactBatches.size(), {}); for(size_t i=0;i<contactBatches.size();++i){ const auto &src = contactBatches[i]; auto &dst = packedContactBatches[i]; for(size_t base=0;base<src.size(); base += PACK_WIDTH){ PackedContactBatch4 batch; batch.count = (uint32_t)std::min<size_t>(PACK_WIDTH, src.size()-base); for(uint32_t lane=0; lane<batch.count; ++lane) batch.indices[lane]=src[base+lane]; dst.push_back(batch);} } }

} // namespace SoA
} // namespace AvbdRef
