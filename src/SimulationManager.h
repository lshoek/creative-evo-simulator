#pragma once

#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "bullet/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"

#include "SimNode.h"
#include "SimInstance.h"
#include "SimDebugDrawer.h"
#include "SimSharedData.h"
#include "Scheduler.h"
#include "ImageSaverThread.h"
#include "GenomeBase.h"
#include "ofMain.h"
#include "ofxGrabCam.h"

typedef std::function<void(uint32_t)> simRunCallback_t;

class SimulationManager
{
public:
    void init();
    void startSimulation();

    void updateTime();
    void updateSimInstances(double timeStep);

    void draw();
    void dealloc();

    // Returns ticket that listener can use to check when sim is finished and genome fitness is updated
    int queueSimulationInstance(const GenomeBase& genome, float duration, bool bMultiEval);
    ofEvent<SimResult> onSimulationInstanceFinished;

    void loadShaders();

    bool isInitialized();
    bool isSimulationInstanceActive();

    ofFbo* getCanvasFbo();
    ofxGrabCam* getCamera();
    float getSimulationTime();

    SimCreature* getFocusCreature();
    glm::vec3 getFocusOrigin();
    void shiftFocus();
    void nextSimulation();

    MorphologyInfo getWalkerMorphologyInfo();
    std::shared_ptr<DirectedGraph> getMorphologyGenome();
    void initTestEnvironment();

    void setMaxParallelSims(int max);
    void setMorphologyGenomeModeEnabled(bool b);
    bool IsMorphologyGenomeModeEnabled();

    bool bDebugDraw = false;
    bool bTestMode = false;
    bool bCameraSnapFocus = true;
    bool bFeasibilityChecks = false;

    glm::vec3 lightPosition;
    glm::vec3 lightDirection;
    uint32_t simulationSpeed = 1;

private:
    void initPhysics();
    void initTerrain();

    void performTrueSteps(btScalar timeStep);
    void handleCollisions(btDynamicsWorld* _worldPtr);

    void writeToPixels(const ofTexture& tex, ofPixels& pix);
    void saveToDisk(const ofPixels& pix);

    int runSimulationInstance(GenomeBase& genome, int ticket, float duration);
    std::vector<simRunCallback_t> _simulationInstanceCallbackQueue;
    std::vector<SimInstance*> _simulationInstances;
    std::mutex _cbQueueMutex;

    ofxGrabCam cam;
    SimNode* _terrainNode;
    std::shared_ptr<DirectedGraph> _testBodyGenome;

    // physics
    btBroadphaseInterface* _broadphase;
    btDefaultCollisionConfiguration* _collisionConfig;
    btCollisionDispatcher* _dispatcher;
    btConstraintSolverPoolMt* _solverPool;
    btSequentialImpulseConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _world;

    SimDebugDrawer* _dbgDrawer;

    // time
    btClock _clock;
    btScalar _startTime = 0;            // clock time that simulation started (ms)
    btScalar _runTime = 0;              // clock time simulation has run (ms)
    btScalar _simulationSpeed = 1.0;    // speed of simulation relative to clock time
    btScalar _simulationTime = 0;       // simulation time after speedup (s)

    const btScalar _fixedTimeStep = 1.0 / 60.0;
    const btScalar _fixedTimeStepMillis = (1.0 / 60.0)*1000.0;
    btScalar _targetFrameTimeMillis;

    btScalar _time = 0;
    btScalar _prevTime = 0;
    btScalar _frameTime = 0;
    btScalar _frameTimeAccumulator = 0;

    // graphics
    std::shared_ptr<ofShader> _terrainShader;
    std::shared_ptr<ofShader> _nodeShader;
    std::shared_ptr<ofShader> _unlitShader;
    std::shared_ptr<ofShader> _canvasUpdateShader;

    std::shared_ptr<ofTexture> _nodeTexture;
    std::shared_ptr<ofTexture> _terrainTexture;
    std::shared_ptr<ofMaterial> _terrainMaterial;
    std::shared_ptr<ofMaterial> _nodeMaterial;
    std::shared_ptr<ofLight> _light;

    btScalar terrainSize = 64.0;
    btScalar canvasSize = 4.0;
    btScalar canvasMargin = 4.0;

    bool bInitialized = false;
    bool bUseMorphologyGenomes = true;

    int simInstanceId = 0;
    int simInstanceGridSize = 2;
    uint32_t simInstanceLimit = 256;
    uint32_t focusIndex = 0;

    // for fixed walker creature
    uint32_t _numWalkerLegs = 8;

    // pbo
    ImageSaverThread _imageSaverThread;
    ofBufferObject pixelWriteBuffers[2];
    ofBufferObject* pboPtr;
    ofPixels writePixels;
    ofBuffer writeBuffer;
    uint32_t iPBO;

    glm::ivec2 _canvasRes;

    const glm::vec3 right = glm::vec3(1, 0, 0);
    const glm::vec3 up =    glm::vec3(0, 1, 0);
    const glm::vec3 fwd =   glm::vec3(0, 0, 1);
};
