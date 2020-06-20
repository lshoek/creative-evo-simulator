#pragma once

#include "SimWorld.h"
#include "SimNode.h"
#include "SimInstance.h"
#include "SimDebugDrawer.h"
#include "SimSharedData.h"
#include "Scheduler.h"
#include "ImageSaverThread.h"
#include "BufferSender.h"
#include "NetworkManager.h"
#include "ImageSaver.h"
#include "ofMain.h"
#include "ofxGrabCam.h"
#include "ofxShadowMap.h"
#include "ofxOpenCv.h"

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
    struct SimSettings
    {
        EvaluationType evalType = EvaluationType::Coverage;
        uint32_t canvasSize = 256;
        uint32_t canvasSizeConv = 128;
        uint32_t maxParallelSims = 1;
        std::string genomeFile = "";
        std::string host = "localhost";
        int inPort = 1024;
        int outPort = 1025;
    };

    void init(SimSettings settings);
    void startSimulation(std::string id, EvaluationType evalType = EvaluationType::Coverage);
    void stopSimulation();

    void update();
    void lateUpdate();

    void shadowPass();
    void draw();
    void dealloc();

    std::string getUniqueSimId();
    const std::string& getStatus();

    // Returns ticket that listener can use to check when sim is finished and genome fitness is updated
    int queueSimInstance(int localId, int generation, float duration);
    ofEvent<SimResult> onSimulationInstanceFinished;

    void loadShaders();

    bool isInitialized();
    bool isSimulationActive();
    bool isSimulationInstanceActive();

    ofxGrabCam* getCamera();

    SimCreature* getFocusCreature();
    SimCanvasNode* getFocusCanvas();
    glm::vec3 getFocusOrigin();
    std::string getFocusInfo();
    void shiftFocus();
    void abortSimInstances();

    MorphologyInfo getWalkerBodyInfo();
    std::shared_ptr<DirectedGraph> getBodyGenome();

    glm::ivec2 getCanvasResolution();
    glm::ivec2 getCanvasConvResolution();

    bool loadBodyGenomeFromDisk(std::string filename);
    void generateRandomBodyGenome();

    bool bAutoLoadGenome = true;
    bool bDebugDraw = false;
    bool bShadows = true;
    bool bMouseLight = false;
    bool bViewLightSpaceDepth = false;
    bool bViewCanvasEvaluationMask = false;
    bool bCameraSnapFocus = true;
    bool bFeasibilityChecks = false;
    bool bCanvasSensors = false;
    bool bUseBodyGenomes = true;
    bool bAxisAlignedAttachments = false;
    bool bSaveArtifactsToDisk = false;
    bool bMultiEval = false;

    glm::vec3 lightPosition;
    uint32_t simulationSpeed = 1;

    EvaluationType evaluationType;

private:
    void setLightUniforms(const std::shared_ptr<ofShader>& shader);
    void setStatus(std::string msg);

    int createSimInstance(int localId, int generation, float duration);
    void updateSimInstance(SimInstance* instance, double timeStep);

    void performTrueSteps(btScalar timeStep);
    double evaluateArtifact(SimInstance* instance);

    std::vector<simRunCallback_t> _simulationInstanceCallbackQueue;
    std::vector<SimInstance*> _simulationInstances;
    std::mutex _cbQueueMutex;

    std::string _status = "";
    std::string _uniqueSimId = "_NA";
    std::string _simDir = NTRS_SIMS_DIR;

    ofxGrabCam cam;

    std::shared_ptr<DirectedGraph> _selectedBodyGenome;
    std::shared_ptr<SimCreature> _previewCreature;

    // preview world
    SimWorld* _previewWorld;

    // shadows
    ofxShadowMap _shadowMap;

    // time
    btClock _clock;
    btScalar _startTime = 0;            // clock time that simulation started (ms)
    btScalar _runTime = 0;              // clock time simulation has run (ms)
    btScalar _simulationSpeed = 0.0;    // speed of simulation relative to clock time
    btScalar _targetFrameTimeMillis;

    btScalar _time = 0;
    btScalar _prevTime = 0;
    btScalar _frameTime = 0;
    btScalar _frameTimeAccumulator = 0;

    // graphics
    std::shared_ptr<ofShader> _terrainShader;
    std::shared_ptr<ofShader> _nodeShader;
    std::shared_ptr<ofShader> _canvasShader;
    std::shared_ptr<ofShader> _canvasColorShader;
    std::shared_ptr<ofShader> _canvasUpdateShader;

    std::shared_ptr<ofTexture> _nodeTexture;
    std::shared_ptr<ofTexture> _terrainTexture;
    std::shared_ptr<ofMaterial> _terrainMaterial;
    std::shared_ptr<ofMaterial> _nodeMaterial;
    std::shared_ptr<ofMaterial> _canvasMaterial;

    std::shared_ptr<ofLight> _light;
    float _lightDistanceFromFocus = 32.0f;

    btScalar canvasSize = 4.0;
    btScalar canvasMargin = 4.0;

    bool bInitialized = false;
    bool bSimulationActive = false;
    bool bStopSimulationQueued = false;
    bool bCanvasDownSampling = false;
    bool bGenomeLoaded = false;

    int _simInstanceIdCounter = 0;
    int _simInstanceGridSize = 2;
    uint32_t _simInstanceLimit = 256;
    uint32_t _focusIndex = 0;
    uint32_t _maxGenGenomeAttempts = 5000;

    // canvas
    glm::ivec2 _canvasResolution;
    glm::ivec2 _canvasConvResolution;

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
    NetworkManager _networkManager;

    ofEventListener _connectionEstablishedListener;
    ofEventListener _connectionClosedListener;
    ofEventListener _onActivationReceived;
    ofEventListener _infoReceivedListener;

    ImageSaver _imageSaver;

    const glm::vec3 right = glm::vec3(1, 0, 0);
    const glm::vec3 up =    glm::vec3(0, 1, 0);
    const glm::vec3 fwd =   glm::vec3(0, 0, 1);
};
