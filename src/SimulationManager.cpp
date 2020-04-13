#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"

#define NTRS_ARTWORK_PATH "output/artworks/"

void SimulationManager::init()
{
    _canvasRes = glm::ivec2(1024, 1024);

    initPhysics();
    initTerrain();

    // rendering
    _material.setDiffuseColor(ofFloatColor(1.0f, 1.0f));
    _material.setAmbientColor(ofFloatColor(0.375f, 1.0f));
    _material.setSpecularColor(ofFloatColor(1.0f, 1.0f));
    _material.setEmissiveColor(ofFloatColor(0.0f, 1.0f));
    _material.setShininess(50.0f);

    loadShaders();
    ofLoadImage(_nodeTexture, "textures/box_256.png");
    ofLoadImage(_terrainTexture, "textures/checkers_64.png");

    _nodeTexture.generateMipmap();
    _nodeTexture.setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    _terrainTexture.generateMipmap();
    _terrainTexture.setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    _terrainTexture.setTextureWrap(GL_REPEAT, GL_REPEAT);

    lightPosition = glm::vec3(0.0f, 1.0f, 0.25f);
    lightDirection = glm::vec3(0.0f, 1.0f, 0.75f);

    // pbo
    _imageSaverThread.setup(NTRS_ARTWORK_PATH);
    writePixels.allocate(_canvasRes.x, _canvasRes.y, GL_RGBA);
    for (int i = 0; i < 2; i++) {
        pixelWriteBuffers[i].allocate(_canvasRes.x*_canvasRes.y*4, GL_DYNAMIC_READ);
    }
    int iPBO = 0;
    pboPtr = &pixelWriteBuffers[iPBO];
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
    _world->setGravity(btVector3(0, -9.81, 0));
    _world->setDebugDrawer(_dbgDrawer);

    // for later
    //_world->setInternalTickCallback(&tickCallback, this, true);
}

void SimulationManager::initTerrain()
{
    _terrainNode = new SimNode(TerrainTag);
    _terrainNode->initPlane(glm::vec3(0), terrainSize, 0);
    _terrainNode->setShader(&_terrainShader);
    _terrainNode->setMaterial(&_material);
    _terrainNode->setTexture(&_terrainTexture);

    _world->addRigidBody(_terrainNode->getRigidBody());
    bTerrainInitialized = true;
}

void SimulationManager::initTestEnvironment()
{
    _canvasNode = new SimCanvasNode(CanvasTag, canvasSize, _canvasRes.x, _canvasRes.y);
    _canvasNode->setPosition(glm::vec3(0, 0.1f, 0));
    _canvasNode->setCanvasUpdateShader(&_canvasUpdateShader);
    _canvasNode->setShader(&_unlitShader);
    _canvasNode->setMaterial(&_material);
    _canvasNode->setTexture(&_nodeTexture);
    _world->addRigidBody(_canvasNode->getRigidBody());

    btVector3 offset(0, 1.0f, 0);
    _creature = new SimCreature(btVector3(0, 0, 0), 6, _world, offset, true);
    _creature->setAppearance(&_nodeShader, &_material, &_nodeTexture);
    _creature->addToWorld();

    _debugSnakeCreature = new SimCreature(btVector3(0, 0, 0), 0, _world, offset, false);
    _debugSnakeCreature->initSnake(btVector3(0, 0, 24), 32, 1.0f, 0.75f, true);
    _debugSnakeCreature->setAppearance(&_nodeShader, &_material, &_nodeTexture);

    _creatures.push_back(_creature);
    _creatures.push_back(_debugSnakeCreature);
    
    bCreaturesInitialized = true;
    bTestMode = true;
}

void SimulationManager::runSimulationInstance(GenomeBase genome)
{
    _canvasNode = new SimCanvasNode(CanvasTag, canvasSize, _canvasRes.x, _canvasRes.y);
    _canvasNode->setPosition(glm::vec3(0, 0.1f, 0));
    _canvasNode->setCanvasUpdateShader(&_canvasUpdateShader);
    _canvasNode->setShader(&_unlitShader);
    _canvasNode->setMaterial(&_material);
    _canvasNode->setTexture(&_nodeTexture);
    _world->addRigidBody(_canvasNode->getRigidBody());

    btVector3 offset(0, 1.0f, 0);
    _creature = new SimCreature(btVector3(0, 0, 0), 6, _world, offset, true);
    _creature->setAppearance(&_nodeShader, &_material, &_nodeTexture);
    _creature->addToWorld();

    _creatures.push_back(_creature);
    bCreaturesInitialized = true;

    //std::vector<double> input;
    //std::vector<double> output = genome.activate(input);
}

void SimulationManager::update(double timeStep)
{
    _world->stepSimulation(timeStep);

    if (bCreaturesInitialized) {
        for (SimCreature*& c : _creatures) {
            c->update(timeStep);
        }
        if (bTerrainInitialized) {
            _canvasNode->update();
        }
    }
}

void SimulationManager::draw()
{
    _terrainShader.begin();
    _terrainShader.setUniform3f("light_pos", lightPosition);
    _terrainShader.setUniform3f("light_dir", lightDirection);
    _terrainShader.end();

    _nodeShader.begin();
    _nodeShader.setUniform3f("light_pos", lightPosition);
    _nodeShader.setUniform3f("light_dir", lightDirection);
    _nodeShader.end();

    if (bDraw) {
        if (bTerrainInitialized) {
            _terrainNode->draw();
        }
        if (bCreaturesInitialized) {
            _canvasNode->draw();
            _creature->draw();
            _debugSnakeCreature->draw();
        }
    }
    if (bDebugDraw) {
        _world->debugDrawWorld();
    }
}

// apply upward force to root node (use for debug only)
void SimulationManager::applyForce(bool bEnable)
{
    if (bEnable) {
        _creature->getRigidBodies()[0]->setActivationState(DISABLE_DEACTIVATION);
        _creature->getRigidBodies()[0]->applyCentralForce(btVector3(0, 500, 0));
        _debugSnakeCreature->getSimNodes()[0]->getRigidBody()->setActivationState(DISABLE_DEACTIVATION);
        _debugSnakeCreature->getSimNodes()[0]->getRigidBody()->applyCentralForce(btVector3(0, 500, 0));
    }
    else {
        _creature->getRigidBodies()[0]->clearForces();
        _debugSnakeCreature->getSimNodes()[0]->getRigidBody()->clearForces();
    }
}

SimCreature* SimulationManager::getFocusCreature()
{
    return _creatures[focusIndex];
}

glm::vec3 SimulationManager::getFocusOrigin()
{
    return SimUtils::bulletToGlm(_creatures[focusIndex]->getPosition());
}

ofFbo* SimulationManager::getCanvasFbo()
{
    return _canvasNode->getCanvasFbo();
}

void SimulationManager::shiftFocus()
{
    focusIndex = (focusIndex + 1) % _creatures.size();
}

void SimulationManager::saveCanvas()
{
    _canvasNode->getCanvasFbo()->getTexture().copyTo(pixelWriteBuffers[iPBO]);

    pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);
    unsigned char* p = pboPtr->map<unsigned char>(GL_READ_ONLY);
    writePixels.setFromExternalPixels(p, _canvasRes.x, _canvasRes.y, OF_PIXELS_RGBA);

    ofSaveImage(writePixels, writeBuffer, OF_IMAGE_FORMAT_JPEG, OF_IMAGE_QUALITY_BEST);
    _imageSaverThread.send(&writeBuffer);

    pboPtr->unmap();
    pboPtr->unbind(GL_PIXEL_UNPACK_BUFFER);

    iPBO = SWAP(iPBO);
    pboPtr = &pixelWriteBuffers[iPBO];
}

void SimulationManager::loadShaders()
{
    _terrainShader.load("shaders/checkers");
    _nodeShader.load("shaders/phong");
    _unlitShader.load("shaders/unlit");
    _canvasUpdateShader.load("shaders/canvas");
}

bool SimulationManager::isInitialized()
{
    return bTerrainInitialized && bCreaturesInitialized;
}

void SimulationManager::dealloc()
{
    if (_terrainNode)           delete _terrainNode;
    if (_canvasNode)            delete _canvasNode;
    if (_creature)              delete _creature;
    if (_debugSnakeCreature)    delete _debugSnakeCreature;

    delete _world;
    delete _solver;
    delete _collisionConfiguration;
    delete _dispatcher;
    delete _broadphase;
}
