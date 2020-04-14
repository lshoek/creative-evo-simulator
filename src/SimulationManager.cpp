#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"

#define NTRS_ARTWORK_PATH "output/artworks/"

void SimulationManager::init()
{
    _canvasRes = glm::ivec2(512, 512);
    _simulationInstances.reserve(simInstanceLimit);
    _simulationInstanceCallbackQueue.reserve(simInstanceLimit);

    // rendering -- load shaders and texture data from disk
    loadShaders();

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

    _material = std::make_shared<ofMaterial>();
    _material->setDiffuseColor(ofFloatColor(1.0f, 1.0f));
    _material->setAmbientColor(ofFloatColor(0.375f, 1.0f));
    _material->setSpecularColor(ofFloatColor(1.0f, 1.0f));
    _material->setEmissiveColor(ofFloatColor(0.0f, 1.0f));
    _material->setShininess(50.0f);

    lightPosition = glm::vec3(0.0f, 1.0f, 0.25f);
    lightDirection = glm::vec3(0.0f, 1.0f, 0.75f);

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
    _terrainNode = new SimNode(TerrainTag);
    _terrainNode->initPlane(glm::vec3(0), terrainSize, 0);
    _terrainNode->setShader(_terrainShader);
    _terrainNode->setMaterial(_material);
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
    canv->setShader(_unlitShader);
    canv->setMaterial(_material);
    canv->setTexture(_nodeTexture);
    _world->addRigidBody(canv->getRigidBody());

    btVector3 offset(0, 1.0f, 0);

    SimCreature* crtr;
    crtr = new SimCreature(btVector3(0, 0, 0), 6, _world, offset, true);
    crtr->setAppearance(_nodeShader, _material, _nodeTexture);
    crtr->addToWorld();

    SimCreature* snake;
    snake = new SimCreature(btVector3(0, 0, 0), 0, _world, offset, false);
    snake->initSnake(btVector3(0, 0, 24), 32, 1.0f, 0.75f, true);
    snake->setAppearance(_nodeShader, _material, _nodeTexture);
    _debugSnakeCreature = snake;

    _simulationInstances.push_back(new SimInstance(0, crtr, canv, ofGetElapsedTimef(), 10.0f));

    bTestMode = true;
}

int SimulationManager::queueSimulationInstance(const GenomeBase& genome, float duration)
{
    // dont allow this when using test objects
    if (bTestMode) return -1;

    int ticket = simInstanceId;

    _simulationInstanceCallbackQueue.push_back(
        std::bind(&SimulationManager::runSimulationInstance, this, genome, ticket, duration)
    );

    // todo: prevent duplicates; requires a more sophisticated mechanism
    simInstanceId = (simInstanceId+1) % simInstanceLimit;

    return ticket;
}

int SimulationManager::runSimulationInstance(const GenomeBase& genome, int ticket, float duration)
{
    // may change depending on simulation id
    btVector3 position(0, 0, 0);

    // some fixed offset that works (may later be based on creature size)
    btVector3 offset(0, 1.0f, 0);

    // make sure to create, add to/remove from world and destroy properly
    SimCanvasNode* canv;
    canv = new SimCanvasNode(CanvasTag, canvasSize, _canvasRes.x, _canvasRes.y, _world);
    canv->setPosition(glm::vec3(position.x(), position.y() + 0.1f, position.z()));
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setShader(_unlitShader);
    canv->setMaterial(_material);
    canv->setTexture(_nodeTexture);
    canv->addToWorld();

    SimCreature* crtr;
    crtr = new SimCreature(position, 6, _world, offset, true);
    crtr->setAppearance(_nodeShader, _material, _nodeTexture);
    crtr->addToWorld();

    //std::vector<double> input;
    //std::vector<double> output = genome.activate(input);

    _simulationInstances.push_back(new SimInstance(ticket, crtr, canv, ofGetElapsedTimef(), duration));

    return ticket;
}

void SimulationManager::update(double timeStep)
{
    // create new simulation instances on main thread
    for (simRunCallback_t cb : _simulationInstanceCallbackQueue) {
        cb(0);
    }
    _simulationInstanceCallbackQueue.clear();

    // close simulation instances that are finished
    for (int i=0; i<_simulationInstances.size(); i++) {
        if (ofGetElapsedTimef() - _simulationInstances[i]->startTime > _simulationInstances[i]->duration) {

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
            saveToDisk(pix);
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
    _terrainShader->begin();
    _terrainShader->setUniform3f("light_pos", lightPosition);
    _terrainShader->setUniform3f("light_dir", lightDirection);
    _terrainShader->end();

    _nodeShader->begin();
    _nodeShader->setUniform3f("light_pos", lightPosition);
    _nodeShader->setUniform3f("light_dir", lightDirection);
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
    }
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
