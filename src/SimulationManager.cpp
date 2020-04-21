#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"

#define NTRS_ARTWORK_PATH "output/artworks/"

void SimulationManager::init()
{
    _canvasRes = glm::ivec2(256, 256);
    _simulationInstances.reserve(simInstanceLimit);
    _simulationInstanceCallbackQueue.reserve(simInstanceLimit);

    // rendering -- load shaders and texture data from disk
    loadShaders();

    cam.setNearClip(0.01f);
    cam.setFarClip(1000.0f);
    cam.setPosition(8.0f, 8.0f, 4.0f);
    cam.lookAt(glm::vec3(0));

    _nodeTexture = std::make_shared<ofTexture>();
    _terrainTexture = std::make_shared<ofTexture>();
    ofLoadImage(*_nodeTexture, "textures/box_256.png");
    ofLoadImage(*_terrainTexture, "textures/checkers_64.png");

    // rendering -- configure appearance
    _nodeTexture->generateMipmap();
    _nodeTexture->setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    _terrainTexture->generateMipmap();
    _terrainTexture->setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    _terrainTexture->setTextureWrap(GL_REPEAT, GL_REPEAT);

    ofMaterialSettings mtl_settings;
    mtl_settings.ambient = ofFloatColor(0.25f, 1.0f);
    mtl_settings.diffuse = ofFloatColor(1.0f, 1.0f);
    mtl_settings.specular = ofFloatColor(1.0f, 1.0f);
    mtl_settings.emissive = ofFloatColor(0.0f, 1.0f);
    mtl_settings.shininess = 32;
    //mtl_settings.postFragment = "postFrag";

    _nodeMaterial = std::make_shared<ofMaterial>();
    _nodeMaterial->setup(mtl_settings);

    _terrainMaterial = std::make_shared<ofMaterial>();
    _terrainMaterial->setup(mtl_settings);

    lightPosition = glm::vec3(0.0f, 10.0f, 0.25f);
    lightDirection = glm::vec3(0.0f, 1.0f, 0.75f);

    _light = std::make_shared<ofLight>();
    _light->setDirectional();
    _light->setAmbientColor(ofColor::white);
    _light->setDiffuseColor(ofColor::white);
    _light->setSpecularColor(ofColor::white);
    _light->setPosition(lightPosition);
    _light->lookAt(_light->getPosition() + lightDirection);

    // physics -- init
    initPhysics();
    initTerrain();

    // pixel buffer object for writing image data to disk fast
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
    _terrainNode = new SimNode(TerrainTag, _world);
    _terrainNode->initPlane(glm::vec3(0), terrainSize, 0);
    _terrainNode->setShader(_terrainShader);
    _terrainNode->setMaterial(_terrainMaterial);
    _terrainNode->setLight(_light);
    _terrainNode->setTexture(_terrainTexture);

    _world->addRigidBody(_terrainNode->getRigidBody());
    bTerrainInitialized = true;
}

void SimulationManager::initTestEnvironment()
{
    SimCanvasNode* canv;
    canv = new SimCanvasNode(CanvasTag, canvasSize, _canvasRes.x, _canvasRes.y, _world);
    canv->setPosition(glm::vec3(0, 0.1f, 0));
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setShader(_terrainShader);
    canv->setMaterial(_nodeMaterial);
    canv->setLight(_light);
    canv->setTexture(_nodeTexture);
    _world->addRigidBody(canv->getRigidBody());

    btVector3 offset(0, 1.0f, 0);

    SimCreature* crtr;
    crtr = new SimCreature(btVector3(0, 0, 0), 6, _world, offset, true);
    crtr->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    crtr->addToWorld();

    SimCreature* snake;
    snake = new SimCreature(btVector3(0, 0, 0), 0, _world, offset, false);
    snake->initSnake(btVector3(0, 0, 24), 32, 1.0f, 0.75f, true);
    snake->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    _debugSnakeCreature = snake;

    _simulationInstances.push_back(new SimInstance(0, crtr, canv, ofGetElapsedTimef(), 10.0f));

    bTestMode = true;
}

int SimulationManager::queueSimulationInstance(const GenomeBase& genome, float duration)
{
    // dont allow this when using test objects
    if (bTestMode) return -1;

    int ticket = simInstanceId;

    {
        std::lock_guard<std::mutex> guard(_cbQueueMutex);
        _simulationInstanceCallbackQueue.push_back(
            std::bind(&SimulationManager::runSimulationInstance, this, genome, ticket, duration)
        );
    }

    // todo: prevent duplicates; requires a more sophisticated mechanism
    simInstanceId = (simInstanceId+1) % simInstanceLimit;

    return ticket;
}

int SimulationManager::runSimulationInstance(GenomeBase& genome, int ticket, float duration)
{
    // todo: locate based on ticket id
    //int gridSize = 2;
    //int grid_x = ticket % gridSize;
    //int grid_z = ticket / gridSize;

    //btVector3 position(grid_x * canvasSize*2.0f, 0, grid_z * canvasSize*2.0f);
    btVector3 position(0, 0, 0);

    // some fixed offset that works (may later be based on creature size)
    btVector3 offset(0, 1.0f, 0);

    // make sure to create, add to/remove from world and destroy properly
    SimCanvasNode* canv;
    canv = new SimCanvasNode(CanvasTag, canvasSize, _canvasRes.x, _canvasRes.y, _world);
    canv->setPosition(glm::vec3(position.x(), position.y() + 0.1f, position.z()));
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setShader(_nodeShader);
    canv->setMaterial(_nodeMaterial);
    canv->setLight(_light);
    canv->setTexture(_nodeTexture);
    canv->addToWorld();

    SimCreature* crtr;
    crtr = new SimCreature(position, _numWalkerLegs, _world, offset, true);
    crtr->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    crtr->setLight(_light);
    crtr->setControlPolicyGenome(genome);
    crtr->addToWorld();

    _simulationInstances.push_back(new SimInstance(ticket, crtr, canv, ofGetElapsedTimef(), duration));

    return ticket;
}

void SimulationManager::update(double timeStep)
{
    {
        // create new simulation instances on main thread
        std::lock_guard<std::mutex> guard(_cbQueueMutex);
        for (simRunCallback_t cb : _simulationInstanceCallbackQueue) {
            cb(0);
        }
        _simulationInstanceCallbackQueue.clear();
    }

    // close simulation instances that are finished
    // TODO: Make this simulated runtime, not app runtime
    for (int i=0; i<_simulationInstances.size(); i++) {
        if (ofGetElapsedTimef() - _simulationInstances[i]->startTime > _simulationInstances[i]->duration) {

            ofLog() << "ended : " << _simulationInstances[i]->instanceId;

            ofPixels pix;
            writeToPixels(_simulationInstances[i]->canvas->getCanvasFbo()->getTexture(), pix);

            double total = 0.0;
            for (ofPixels::ConstPixel p : pix.getConstPixelsIter()) {
                // black paint so calculate inverse color
                total += 1.0 - (p[0] + p[1] + p[2])/(3.0*255.0);
            }
            total /= double(_canvasRes.x * _canvasRes.y);

            SimResult result;
            result.instanceId = _simulationInstances[i]->instanceId;
            result.fitness = total;
            onSimulationInstanceFinished.notify(result);

            delete _simulationInstances[i];
            _simulationInstances.erase(_simulationInstances.begin() + i);

            // optional
            //saveToDisk(pix);
        }
    }

    _world->stepSimulation(timeStep);
    for (auto& s : _simulationInstances) {
        s->canvas->update();
        s->creature->update(timeStep);
    }
}

void SimulationManager::draw()
{
    if (bCameraSnapFocus) {
        cam.lookAt(getFocusOrigin());
    }
    cam.begin();

    _light->setPosition(lightPosition);
    _light->lookAt(_light->getPosition() + lightDirection);

    _terrainShader->begin();
    _terrainShader->setUniform3f("light.position", _light->getPosition());
    _terrainShader->setUniform3f("light.direction", _light->getLookAtDir());
    _terrainShader->setUniform4f("light.ambient", _light->getAmbientColor());
    _terrainShader->setUniform4f("light.diffuse", _light->getDiffuseColor());
    _terrainShader->setUniform4f("light.specular", _light->getSpecularColor());
    _terrainShader->setUniform3f("eyePos", cam.getPosition());
    _terrainShader->end();

    _nodeShader->begin();
    _nodeShader->setUniform3f("light.position", _light->getPosition());
    _nodeShader->setUniform3f("light.direction", _light->getLookAtDir());
    _nodeShader->setUniform4f("light.ambient", _light->getAmbientColor());
    _nodeShader->setUniform4f("light.diffuse", _light->getDiffuseColor());
    _nodeShader->setUniform4f("light.specular", _light->getSpecularColor());
    _nodeShader->setUniform3f("eyePos", cam.getPosition());
    _nodeShader->end();

    if (bDraw) {
        if (bTerrainInitialized) {
            _terrainNode->draw();
        }           
        for (auto &s : _simulationInstances) {
            s->canvas->draw();
            s->creature->draw();
        }
        if (bTestMode) {
            _debugSnakeCreature->draw();
        }
    }
    if (bDebugDraw) {
        _world->debugDrawWorld();
        ofDrawIcoSphere(lightPosition, 2.0f);
    }
    cam.end();
}

ofxGrabCam* SimulationManager::getCamera()
{
    return &cam;
}

SimCreature* SimulationManager::getFocusCreature()
{
    if (!_simulationInstances.empty()) {
        return _simulationInstances[focusIndex]->creature;
    }
    else return nullptr;
}

glm::vec3 SimulationManager::getFocusOrigin()
{
    if (!_simulationInstances.empty()) {
        return SimUtils::bulletToGlm(_simulationInstances[focusIndex]->creature->getPosition());
    }
    else return glm::vec3(0);
}

ofFbo* SimulationManager::getCanvasFbo()
{
    if (!_simulationInstances.empty()) {
        _simulationInstances[focusIndex]->canvas->getCanvasFbo();
    }
    else return nullptr;
}

void SimulationManager::shiftFocus()
{
    focusIndex = (focusIndex + 1) % _simulationInstances.size();
}

// As morphology is fixed this will only work for the walker creature for now
MorphologyInfo SimulationManager::getWalkerMorphologyInfo()
{
    int numBodyParts = 2 * _numWalkerLegs + 1;
    int numJoints = numBodyParts - 1;

    return MorphologyInfo(numBodyParts, numJoints);
}

void SimulationManager::writeToPixels(const ofTexture& tex, ofPixels& pix)
{
    tex.copyTo(pixelWriteBuffers[iPBO]);

    pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);
    unsigned char* p = pboPtr->map<unsigned char>(GL_READ_ONLY);
    pix.setFromExternalPixels(p, _canvasRes.x, _canvasRes.y, OF_PIXELS_RGBA);

    pboPtr->unmap();
    pboPtr->unbind(GL_PIXEL_UNPACK_BUFFER);

    iPBO = SWAP(iPBO);
    pboPtr = &pixelWriteBuffers[iPBO];
}

void SimulationManager::saveToDisk(const ofPixels& pix)
{
    ofSaveImage(pix, writeBuffer, OF_IMAGE_FORMAT_JPEG, OF_IMAGE_QUALITY_BEST);
    _imageSaverThread.send(&writeBuffer);
}

void SimulationManager::loadShaders()
{
    _terrainShader = std::make_shared<ofShader>();
    _nodeShader = std::make_shared<ofShader>();
    _unlitShader = std::make_shared<ofShader>();
    _canvasUpdateShader = std::make_shared<ofShader>();

    _terrainShader->load("shaders/checkers");
    _nodeShader->load("shaders/phong");
    _unlitShader->load("shaders/unlit");
    _canvasUpdateShader->load("shaders/canvas");
}

bool SimulationManager::isInitialized()
{
    return bTerrainInitialized;
}

bool SimulationManager::isSimulationInstanceActive()
{
    return !_simulationInstances.empty();
}

void SimulationManager::dealloc()
{
    if (_terrainNode)           delete _terrainNode;
    if (_debugSnakeCreature)    delete _debugSnakeCreature;

    for (auto &s : _simulationInstances) {
        delete s;
    }
    delete _world;
    delete _solver;
    delete _collisionConfiguration;
    delete _dispatcher;
    delete _broadphase;
}
