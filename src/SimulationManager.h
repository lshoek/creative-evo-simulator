#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "SimNode.h"
#include "SimCreature.h"
#include "SimDebugDrawer.h"
#include "ImageSaverThread.h"
#include "GenomeBase.h"
#include "ofMain.h"

class SimulationManager
{
public:
    void init();
    void update(double timestep);
    void draw();
    void dealloc();

    void runSimulationInstance(GenomeBase genome);

    void applyForce(bool bEnable);
    void loadShaders();

    void saveCanvas();
    bool isInitialized();

    ofFbo* getCanvasFbo();

    SimCreature* getFocusCreature();
    glm::vec3 getFocusOrigin();
    void shiftFocus();

    bool bDraw = true;
    bool bDebugDraw = false;
    bool bTestMode = false;

    glm::vec3 lightPosition;
    glm::vec3 lightDirection;

    void initTestEnvironment();

private:
    void initPhysics();
    void initTerrain();

    btBroadphaseInterface* _broadphase;
    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _dispatcher;
    btSequentialImpulseConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _world;

    SimDebugDrawer* _dbgDrawer;

    ofShader _terrainShader;
    ofShader _nodeShader;
    ofShader _unlitShader;
    ofShader _canvasUpdateShader;

    ofTexture _nodeTexture;
    ofTexture _terrainTexture;
    ofMaterial _material;

    SimNode* _terrainNode;
    SimCanvasNode* _canvasNode;
    SimCreature* _creature;
    SimCreature* _debugSnakeCreature;

    std::vector<SimCreature*> _creatures;

    float terrainSize = 48.0f;
    float canvasSize = 8.0f;

    bool bRandomSize = false;

    bool bTerrainInitialized = false;
    bool bCreaturesInitialized = false;

    const int numCreatures = 2;
    int focusIndex = 0;

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
