// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
#include "PxPhysicsAPI.h"
#include <chrono>
#include <cstdio>
#include <vector>

using namespace physx;

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;

struct Record {
  const char* name;
  double msPerFrame;
  PxU32 actors;
  PxU32 articulations;
};

static Record runBoxStacks(PxPhysics& physics, PxScene& scene, PxMaterial& material) {
  for (PxU32 i = 0; i < 10; ++i)
    for (PxU32 j = 0; j < 10 - i; ++j) {
      PxTransform pose(PxVec3(PxReal(j * 2 - (10 - i)), PxReal(i * 2 + 1), 0.0f));
      PxRigidDynamic* body = PxCreateDynamic(physics, pose, PxBoxGeometry(1,1,1), material, 10.0f);
      scene.addActor(*body);
    }
  for (PxU32 i = 0; i < 60; ++i) { scene.simulate(1.0f/60.0f); scene.fetchResults(true); }
  double ms = 0.0;
  for (PxU32 i = 0; i < 180; ++i) {
    auto t0 = std::chrono::steady_clock::now();
    scene.simulate(1.0f/60.0f); scene.fetchResults(true);
    ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  }
  return {"box stacks", ms / 180.0, scene.getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC), 0};
}

int snippetMain(int, const char* const*) {
  PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
  PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), false, nullptr);
  PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(4);
  PxSceneDesc desc(physics->getTolerancesScale());
  desc.gravity = PxVec3(0, -9.81f, 0);
  desc.cpuDispatcher = dispatcher;
  desc.filterShader = PxDefaultSimulationFilterShader;
  desc.solverType = PxSolverType::eAVBD;
  PxScene* scene = physics->createScene(desc);
  PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.5f);
  scene->addActor(*PxCreatePlane(*physics, PxPlane(0,1,0,0), *material));

  Record record = runBoxStacks(*physics, *scene, *material);
  std::printf("PhysX AVBD benchmark: %s ms/frame=%.3f actors=%u articulations=%u\n",
              record.name, record.msPerFrame, record.actors, record.articulations);

  material->release();
  scene->release();
  dispatcher->release();
  physics->release();
  foundation->release();
  return 0;
}
