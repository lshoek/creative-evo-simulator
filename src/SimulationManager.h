#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "SimNode.h"
#include "SimCreature.h"
#include "SimDebugDrawer.h"
#include "SimSharedData.h"
#include "ImageSaverThread.h"
#include "GenomeBase.h"
#include "ofMain.h"
#include "ofxGrabCam.h"

typedef std::function<void(uint32_t)> simRunCallback_t;

class SimulationManager
{
public:
    void init();
    void update(double timestep);
    void draw();
    void dealloc();

    // Returns ticket that listener can use to check when sim is finished and genome fitness is updated
    int queueSimulationInstance(const GenomeBase& genome, float duration);
    ofEvent<SimResult> onSimulationInstanceFinished;

    void loadShaders();

    bool isInitialized();
    bool isSimulationInstanceActive();

    ofFbo* getCanvasFbo();

    ofxGrabCam* getCamera();
    SimCreature* getFocusCreature();
    glm::vec3 getFocusOrigin();
    void shiftFocus();

    bool bDraw = true;
    bool bDebugDraw = false;
    bool bTestMode = false;
    bool bCameraSnapFocus = true;

    glm::vec3 lightPosition;
    glm::vec3 lightDirection;

    MorphologyInfo getWalkerMorphologyInfo();
    void initTestEnvironment();

private:
    struct SimInstance {
        int instanceId;
        float startTime, duration;
        SimCreature* creature;
        SimCanvasNode* canvas;

        SimInstance(int id, SimCreature* crtr, SimCanvasNode* canv, float startTime, float duration) :
            instanceId(id), creature(crtr), canvas(canv), startTime(startTime), duration(duration) {}
        ~SimInstance() {
            delete creature;
            delete canvas;
        }
    };

    void initPhysics();
    void initTerrain();

    void writeToPixels(const ofTexture& tex, ofPixels& pix);
    void saveToDisk(const ofPixels& pix);

    int runSimulationInstance(GenomeBase& genome, int ticket, float duration);
    std::vector<simRunCallback_t> _simulationInstanceCallbackQueue;
    std::vector<SimInstance*> _simulationInstances;
    std::mutex _cbQueueMutex;

    ofxGrabCam cam;

    SimNode* _terrainNode;
    SimCreature* _debugSnakeCreature;

    btBroadphaseInterface* _broadphase;
    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _dispatcher;
    btSequentialImpulseConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _world;

    SimDebugDrawer* _dbgDrawer;

    std::shared_ptr<ofShader> _terrainShader;
    std::shared_ptr<ofShader> _nodeShader;
    std::shared_ptr<ofShader> _unlitShader;
    std::shared_ptr<ofShader> _canvasUpdateShader;

    std::shared_ptr<ofTexture> _nodeTexture;
    std::shared_ptr<ofTexture> _terrainTexture;
    std::shared_ptr<ofMaterial> _terrainMaterial;
    std::shared_ptr<ofMaterial> _nodeMaterial;
    std::shared_ptr<ofLight> _light;

    float terrainSize = 48.0f;
    float canvasSize = 4.0f;

    bool bTerrainInitialized = false;

    int focusIndex = 0;
    int simInstanceId = 0;
    int simInstanceLimit = 256;

    // for fixed walker creature
    int _numWalkerLegs = 8;

    // pbo
    ImageSaverThread _imageSaverThread;
    ofBufferObject pixelWriteBuffers[2];
    ofBufferObject* pboPtr;
    ofPixels writePixels;
    ofBuffer writeBuffer;
    int iPBO;

    glm::ivec2 _canvasRes;

    const glm::vec3 right = glm::vec3(1, 0, 0);
    const glm::vec3 up =    glm::vec3(0, 1, 0);
    const glm::vec3 fwd =   glm::vec3(0, 0, 1);
};
