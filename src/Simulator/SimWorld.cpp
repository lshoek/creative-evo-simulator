#include "SimWorld.h"
#include "SimInstance.h"
#include "SimDefines.h"

SimWorld::SimWorld() { init(); }

void handleCollisions(btDynamicsWorld* worldPtr, bool bCanvasSensors);

void worldUpdateCallback(btDynamicsWorld* world, btScalar timeStep)
{
    if (world->getWorldUserInfo()) {
        SimWorld* userWorld = (SimWorld*)world->getWorldUserInfo();

        if (userWorld->getSimInstance()) {
            SimInstance* instance = userWorld->getSimInstance();
            handleCollisions(world, instance->getCreature()->_sensorMode == SimCreature::Canvas);
        }
    }
}

void SimWorld::init()
{
    _broadphase = new btDbvtBroadphase();
    _collisionConfig = new btDefaultCollisionConfiguration();

    if (_bMultiThreading) {
        _dispatcher = new btCollisionDispatcherMt(_collisionConfig);
        _solverPool = new btConstraintSolverPoolMt(BT_MAX_THREAD_COUNT);
        _solver = new btSequentialImpulseConstraintSolverMt();
        _world = new btDiscreteDynamicsWorldMt(_dispatcher, _broadphase, _solverPool, _solver, _collisionConfig);
        btSetTaskScheduler(btCreateDefaultTaskScheduler());
    }
    else {
        _dispatcher = new btCollisionDispatcher(_collisionConfig);
        _solver = new btSequentialImpulseConstraintSolver();
        _world = new btDiscreteDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfig);
    }

    _dbgDrawer = new SimDebugDrawer();
    _dbgDrawer->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe |
        btIDebugDraw::DBG_DrawConstraints |
        btIDebugDraw::DBG_DrawConstraintLimits
    );

    _world->setGravity(btVector3(0, -9.81, 0));
    _world->setDebugDrawer(_dbgDrawer);
    _world->setWorldUserInfo(this);
    _world->setInternalTickCallback(&worldUpdateCallback, this, false);

    _terrainNode = new SimNode(TerrainTag, _world);
    _terrainNode->initPlane(btVector3(0, 0, 0), _terrainSize, 0);
    _terrainNode->addToWorld();
}

void handleCollisions(btDynamicsWorld* worldPtr, bool bCanvasSensors)
{
    int numManifolds = worldPtr->getDispatcher()->getNumManifolds();

    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = worldPtr->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* o1 = (btCollisionObject*)(contactManifold->getBody0());
        btCollisionObject* o2 = (btCollisionObject*)(contactManifold->getBody1());

        bool bBrushContact = false;
        for (int j = 0; j < contactManifold->getNumContacts(); j++)
        {
            // Collisions for touch sensors
            if (!bCanvasSensors &&
                o1->getUserIndex() & BodyTag && o2->getUserIndex() & ~BodyTag ||
                o1->getUserIndex() & ~BodyTag && o2->getUserIndex() & BodyTag)
            {
                SimCreature* creaturePtr = nullptr;

                // brushtag should always have a simcreature as user pointer
                if (o1->getUserIndex() & BodyTag) {
                    creaturePtr = ((SimNode*)o1->getUserPointer())->getCreaturePtr();
                    creaturePtr->setTouchSensor(o1);
                }
                else {
                    creaturePtr = ((SimNode*)o2->getUserPointer())->getCreaturePtr();
                    creaturePtr->setTouchSensor(o2);
                }
            }
            // Collisions for canvas sensors
            if (bBrushContact) continue; // test: register single contact point per update
            if ((o1->getUserIndex() & BrushTag && o2->getUserIndex() & CanvasTag) ||
                (o1->getUserIndex() & CanvasTag && o2->getUserIndex() & BrushTag))
            {
                SimCanvasNode* canvasPtr = nullptr;
                SimNode* brushNodePtr = nullptr;

                if (o1->getUserIndex() & CanvasTag) {
                    canvasPtr = (SimCanvasNode*)o1->getUserPointer();
                    brushNodePtr = (SimNode*)o2->getUserPointer();
                }
                else {
                    canvasPtr = (SimCanvasNode*)o2->getUserPointer();
                    brushNodePtr = (SimNode*)o1->getUserPointer();
                }

                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                btVector3 localPt = pt.getPositionWorldOnA() - canvasPtr->getPosition();

                canvasPtr->addBrushStroke(localPt, brushNodePtr->getBrushPressure(), brushNodePtr->isBrushActivated());
                bBrushContact = true;
            }
        }
        //contactManifold->clearManifold();
    }
}

SimInstance* SimWorld::getSimInstance()
{
    return _owner;
}

void SimWorld::setSimInstance(SimInstance* instance)
{
    _owner = instance;
}

btDiscreteDynamicsWorld* SimWorld::getBtWorld()
{
    return _world;
}

SimNode* SimWorld::getTerrainNode()
{
    return _terrainNode;
}

SimWorld::~SimWorld()
{
    delete _terrainNode;
    _owner = 0;

    delete _broadphase;
    delete _collisionConfig;
    delete _solver;
    delete _dispatcher;

    if (_bMultiThreading) {
        delete _solverPool;
    }
    delete _dbgDrawer;
    delete _world;
}
