#pragma once

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"

#include "SimNode.h"
#include "SimInstance.h"
#include "SimDebugDrawer.h"
#include "SimSharedData.h"
#include "Scheduler.h"
#include "ImageSaverThread.h"
#include "BufferSender.h"
#include "BufferReceiver.h"
#include "ImageSaver.h"
#include "GenomeBase.h"
#include "ofMain.h"
#include "ofxGrabCam.h"
#include "ofxShadowMap.h"

typedef std::function<void(uint32_t)> simRunCallback_t;

class SimulationManager
{
public:
    enum EvaluationType
    {
        Coverage,
        CircleCoverage,
        InverseCircleCoverage
    };
    enum LearningMode
    {
        NEAT,
        External
    };
    struct SimSettings
    {
        SimSettings(LearningMode learningMode, EvaluationType evalType, uint32_t w, uint32_t h, std::string genomeFile, std::string host, int inPort, int outPort) :
            learningMode(learningMode), evalType(evalType), canvasWidth(w), canvasHeight(h), genomeFile(genomeFile), host(host), inPort(inPort), outPort(outPort) {}
        LearningMode learningMode = LearningMode::NEAT;
        EvaluationType evalType = EvaluationType::Coverage;
        uint32_t canvasWidth = 256;
        uint32_t canvasHeight = 256;
        std::string genomeFile = "";
        std::string host = "localhost";
        int inPort = 1024;
        int outPort = 1025;
    };

    void init(SimSettings settings);
    void startSimulation(std::string id, EvaluationType evalType = EvaluationType::Coverage);
    void stopSimulation();

    void updateTime();
    void lateUpdate();
    void updateSimInstances(double timeStep);

    void shadowPass();
    void draw();
    void dealloc();

    std::string getUniqueSimId();
    const std::string& getStatus();

    // Returns ticket that listener can use to check when sim is finished and genome fitness is updated
    int queueSimulationInstance(const GenomeBase& genome, float duration, bool bMultiEval);
    ofEvent<SimResult> onSimulationInstanceFinished;

    void loadShaders();

    bool isInitialized();
    bool isSimulationActive();
    bool isSimulationInstanceActive();

    ofxGrabCam* getCamera();
    float getSimulationTime();

    SimCreature* getFocusCreature();
    SimCanvasNode* getFocusCanvas();
    glm::vec3 getFocusOrigin();
    void shiftFocus();
    void abortSimInstances();

    MorphologyInfo getWalkerBodyInfo();
    std::shared_ptr<DirectedGraph> getBodyGenome();

    glm::ivec2 getCanvasResolution();
    glm::ivec2 getCanvasNeuronInputResolution();

    void loadBodyGenomeFromDisk(std::string filename);
    void generateRandomBodyGenome();
    void initTestEnvironment();

    void setMaxParallelSims(int max);
    void setCanvasNeuronInputResolution(uint32_t width, uint32_t height);

    bool bAutoLoadGenome = true;
    bool bDebugDraw = false;
    bool bShadows = true;
    bool bTestMode = false;
    bool bMouseLight = false;
    bool bViewLightSpaceDepth = false;
    bool bViewCanvasEvaluationMask = false;
    bool bCameraSnapFocus = true;
    bool bFeasibilityChecks = false;
    bool bCanvasInputNeurons = false;
    bool bUseBodyGenomes = true;
    bool bAxisAlignedAttachments = false;
    bool bSaveArtifactsToDisk = false;

    glm::vec3 lightPosition;
    uint32_t simulationSpeed = 1;

    EvaluationType evaluationType;
    LearningMode learningMode;

private:
    void initPhysics();
    void initTerrain();

    void setLightUniforms(const std::shared_ptr<ofShader>& shader);

    void performTrueSteps(btScalar timeStep);
    void handleCollisions(btDynamicsWorld* _worldPtr);

    double evaluateArtifact(SimInstance* instance);

    int runSimulationInstance(GenomeBase& genome, int ticket, float duration);
    std::vector<simRunCallback_t> _simulationInstanceCallbackQueue;
    std::vector<SimInstance*> _simulationInstances;
    std::mutex _cbQueueMutex;

    std::string _status = "";
    std::string _uniqueSimId = "_NA";
    std::string _simDir = NTRS_SIMS_DIR;

    ofxGrabCam cam;
    SimNode* _terrainNode;

    std::shared_ptr<DirectedGraph> _selectedBodyGenome;
    std::shared_ptr<SimCreature> _testCreature;

    // shadows
    ofxShadowMap _shadowMap;

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
    btScalar _simulationSpeed = 0.0;    // speed of simulation relative to clock time
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
    std::shared_ptr<ofShader> _canvasColorShader;
    std::shared_ptr<ofShader> _canvasUpdateShader;

    std::shared_ptr<ofTexture> _nodeTexture;
    std::shared_ptr<ofTexture> _terrainTexture;
    std::shared_ptr<ofMaterial> _terrainMaterial;
    std::shared_ptr<ofMaterial> _nodeMaterial;

    std::shared_ptr<ofLight> _light;
    float _lightDistanceFromFocus = 32.0f;

    btScalar terrainSize = 64.0;
    btScalar canvasSize = 4.0;
    btScalar canvasMargin = 4.0;

    bool bInitialized = false;
    bool bSimulationActive = false;
    bool bStopSimulationQueued = false;

    int simInstanceId = 0;
    int simInstanceGridSize = 2;
    uint32_t simInstanceLimit = 256;
    uint32_t focusIndex = 0;
    uint32_t maxGenGenomeAttempts = 5000;

    // canvas
    glm::ivec2 _canvasResolution;
    glm::ivec2 _canvasNeuralInputResolution;

    cv::Mat _artifactMat;
    cv::Mat _maskMat, _invMaskMat;
    cv::Mat _rewardMat, _penaltyMat;
    cv::Mat* _rewardMaskPtr;
    cv::Mat* _penaltyMaskPtr;
    double _maxReward = 255.0;

    ofxCvGrayscaleImage _cvDebugImage;

    // for fixed walker creature
    uint32_t _numWalkerLegs = 8;

    // io
    BufferSender _bufferSender;
    BufferReceiver _bufferReceiver;
    ImageSaver _imageSaver;

    const glm::vec3 right = glm::vec3(1, 0, 0);
    const glm::vec3 up =    glm::vec3(0, 1, 0);
    const glm::vec3 fwd =   glm::vec3(0, 0, 1);
};
