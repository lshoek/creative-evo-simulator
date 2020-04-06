#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "SimNode.h"
#include "SimDebugDrawer.h"
#include "ofMain.h"

#define NodeTag 0
#define JointTag 1
#define BrushTag 2
#define TerrainTag 3

class SimulationManager
{
public:
    void init();
    void update(float delta);
    void draw();
    void reset();
    void dealloc();

    void applyForce(bool bEnable);
    void loadShaders();

    bool isMainCreatureAvailable();
    glm::vec3 getMainCreatureOrigin();

    unsigned int getNumNodes();
    const std::vector<SimNode*>& getNodes();

    bool bDraw = true;
    bool bDebugDraw = false;

    glm::vec3 lightPosition;
    glm::vec3 lightDirection;

private:
    void initPhysics();
    void initObjects();

    void initDebugSnake();
    void disableCollisionOnLinkedNodes();

    btBroadphaseInterface* _broadphase;
    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _dispatcher;
    btSequentialImpulseConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _world;

    btHingeConstraint* _hingeConstraint;

    SimDebugDrawer* _dbgDrawer;

    ofShader _terrainShader;
    ofShader _nodeShader;
    ofTexture _nodeTexture;
    ofMaterial _material;

    SimNode* _terrainNode;
    std::vector<SimNode*> _nodes;

    float maxBoxExtents = 4.0f;
    float boxExtents = 4.0f;
    float defaultMass = 0.5f;
    float terrainSize = 500.0f;
    float spawnAreaPct = 0.125f;

    float maxDistBetweenNodes = 100.0f;
    float distPct = 0.5f;

    unsigned int maxNodes = 32;

    bool bRandomSize = false;

    const glm::vec3 right = glm::vec3(1, 0, 0);
    const glm::vec3 up = glm::vec3(0, 1, 0);
    const glm::vec3 fwd = glm::vec3(0, 0, 1);
};
