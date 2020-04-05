#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"

void SimulationManager::init()
{
    bRandomSize = true;
    boxExtents = maxBoxExtents;
    maxDistBetweenNodes = ofLerp(0, sqrt((maxBoxExtents * 2) * (maxBoxExtents * 2) * 3), distPct);

    initPhysics();
    initObjects();

    // rendering
    _material.setDiffuseColor(ofFloatColor(1.0f, 1.0f));
    _material.setAmbientColor(ofFloatColor(0.375f, 1.0f));
    _material.setSpecularColor(ofFloatColor(1.0f, 1.0f));
    _material.setEmissiveColor(ofFloatColor(0.0f, 1.0f));
    _material.setShininess(50.0f);

    loadShaders();
    ofLoadImage(_nodeTexture, "textures/box.png");
    _nodeTexture.generateMipmap();
    _nodeTexture.setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);

    _terrainNode->setMaterial(&_material);
    _terrainNode->setShader(&_terrainShader);

    for (int i = 0; i < maxNodes; i++) {
        _nodes[i]->setMaterial(&_material);
        _nodes[i]->setTexture(&_nodeTexture);
        _nodes[i]->setShader(&_nodeShader);
    }
}

void SimulationManager::initPhysics()
{
    _broadphase = new btDbvtBroadphase();
    _collisionConfiguration = new btDefaultCollisionConfiguration();
    _dispatcher = new btCollisionDispatcher(_collisionConfiguration);
    _solver = new btSequentialImpulseConstraintSolver();

    _dbgDrawer = new SimDebugDrawer();
    _dbgDrawer->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe | 
        btIDebugDraw::DBG_DrawConstraints | 
        btIDebugDraw::DBG_DrawContactPoints
    );

    _world = new btDiscreteDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfiguration);
    _world->setGravity(btVector3(0, -9.8, 0));
    _world->setDebugDrawer(_dbgDrawer);
}

void SimulationManager::initObjects()
{
    _terrainNode = new SimNode();
    _terrainNode->init("terrainNode", glm::vec3(0), glm::vec3(terrainSize, 1, terrainSize), 0, false, TerrainTag);
    _terrainNode->setShader(&_terrainShader);
    _world->addRigidBody(_terrainNode->getRigidBody());

    _nodes.resize(maxNodes);
    initDebugSnake();
}

void SimulationManager::initDebugSnake()
{
    glm::vec3 start = glm::vec3(
        ofRandom(-terrainSize * spawnAreaPct, terrainSize * spawnAreaPct), boxExtents,
        ofRandom(-terrainSize * spawnAreaPct, terrainSize * spawnAreaPct)
    );

    glm::vec3 size = glm::vec3(boxExtents);
    glm::vec3 dir = right;
    glm::vec3 cur, prev = start;
    glm::quat lookatRot;

    cur = start;
    for (int i = 0; i < maxNodes; i++)
    {
        char id[32];
        sprintf_s(id, "testNode_%d", i);

        lookatRot = tb::quatDiff(right, glm::rotation(right, dir) * glm::rotate(right, 0.125f, up));
        dir = glm::normalize(lookatRot * right);

        prev = cur;
        cur = prev + dir * maxDistBetweenNodes;

        if (bRandomSize) {
            size = glm::vec3(ofRandom(1, maxBoxExtents), maxBoxExtents, ofRandom(1, maxBoxExtents));
        }
        _nodes[i] = new SimNode();
        _nodes[i]->init(id, cur, size, defaultMass, true, NodeTag);
        _nodes[i]->setRotation(lookatRot);

        _world->addRigidBody(_nodes[i]->getRigidBody());

        if (i > 0) {
            btVector3 pivotInA(0, 0, 0);
            btVector3 pivotInB =
                _nodes[i - 1]->getRigidBody()->getWorldTransform().inverse()(_nodes[i]->getRigidBody()->getWorldTransform()(pivotInA));

            btTypedConstraint* p2p = new btPoint2PointConstraint(
                *_nodes[i]->getRigidBody(), *_nodes[i - 1]->getRigidBody(), pivotInA, pivotInB);
            _world->addConstraint(p2p);
        }
    }
    disableCollisionOnLinkedNodes();
}

void SimulationManager::update(float delta)
{
    _world->stepSimulation(delta);
}

void SimulationManager::draw()
{
    if (bDraw) {
        _terrainNode->draw();
        for (int i = 0; i < maxNodes; i++) {
            _nodes[i]->draw();
        }
    }
    if (bDebugDraw) {
        _world->debugDrawWorld();
    }
}

void SimulationManager::disableCollisionOnLinkedNodes()
{
    for (int i = 1; i < maxNodes-1; i++) {
        _nodes[i]->getRigidBody()->setIgnoreCollisionCheck(_nodes[i - 1]->getRigidBody(), true);
        _nodes[i]->getRigidBody()->setIgnoreCollisionCheck(_nodes[i + 1]->getRigidBody(), true);
    }
}

void SimulationManager::applyForce(bool bEnable)
{
    if (isMainCreatureAvailable()) {
        if (bEnable) {
            _nodes[0]->getRigidBody()->setActivationState(DISABLE_DEACTIVATION);
            _nodes[0]->getRigidBody()->applyCentralForce(btVector3(0, 500, 0));
        }
        else {
            _nodes[0]->getRigidBody()->clearForces();
        }
    }
}

void SimulationManager::loadShaders()
{
    _terrainShader.load("shaders/checkers");
    _nodeShader.load("shaders/phong");
}

bool SimulationManager::isMainCreatureAvailable()
{
    return !_nodes.empty() && _nodes[0]->hasBody();
}

glm::vec3 SimulationManager::getMainCreatureOrigin()
{
    if (isMainCreatureAvailable()) {
        return _nodes[0]->getPosition();
    }
    else return glm::vec3(0);
}

const std::vector<SimNode*>& SimulationManager::getNodes()
{
    return _nodes;
}

unsigned int SimulationManager::getNumNodes()
{
    return _nodes.size();
}

void SimulationManager::reset()
{
    initObjects();
}

void SimulationManager::dealloc()
{
    delete _terrainNode;

    delete _world;
    delete _solver;
    delete _collisionConfiguration;
    delete _dispatcher;
    delete _broadphase;
}
