#pragma once
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"

#include "SimNode.h"
#include "SimDebugDrawer.h"
class SimInstance;

class SimWorld
{
public:
    SimWorld();
    ~SimWorld();

    SimInstance* getSimInstance();
    void setSimInstance(SimInstance* instance);

    btDiscreteDynamicsWorld* getBtWorld();
    SimNode* getTerrainNode();

private:
    void init();

    btBroadphaseInterface* _broadphase;
    btDefaultCollisionConfiguration* _collisionConfig;
    btCollisionDispatcher* _dispatcher;
    btConstraintSolverPoolMt* _solverPool;
    btSequentialImpulseConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _world;

    SimDebugDrawer* _dbgDrawer;
    SimNode* _terrainNode;
    SimInstance* _owner = NULL;

    btScalar _terrainSize = 64.0;
};
