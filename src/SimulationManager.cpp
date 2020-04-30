#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"
#include "DirectedGraph.h"

#define NTRS_ARTWORK_PATH "output/artworks/"

void worldUpdateCallback(btDynamicsWorld* world, btScalar timeStep);

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

    lightPosition = glm::vec3(0.0f, 20.0f, 0.0f);
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

    if (ofGetTargetFrameRate()) {
        _targetFrameTimeMillis = 1000.0f/ofGetTargetFrameRate();
    }
    else {
        _targetFrameTimeMillis = _fixedTimeStepMillis;
    }

    _testMorphologyGenome.unwrap();
    bInitialized = true;
}

void SimulationManager::startSimulation()
{
    if (bInitialized) {
        _startTime = _clock.getTimeMilliseconds();
        _clock.reset();
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
        btIDebugDraw::DBG_DrawConstraintLimits
    );
    
    _world = new btDiscreteDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfiguration);
    _world->setGravity(btVector3(0, -9.81, 0));
    _world->setDebugDrawer(_dbgDrawer);
    _world->setWorldUserInfo(this);
    _world->setInternalTickCallback(&worldUpdateCallback, this, false);
}

void SimulationManager::initTerrain()
{
    _terrainNode = new SimNode(TerrainTag, _world);
    _terrainNode->initPlane(btVector3(0, 0, 0), terrainSize, 0);
    _terrainNode->setAppearance(_terrainShader, _terrainMaterial, _terrainTexture);
    _terrainNode->setLight(_light);

    _world->addRigidBody(_terrainNode->getRigidBody());
}

void SimulationManager::initTestEnvironment()
{
    SimCanvasNode* canv;
    canv = new SimCanvasNode(btVector3(0, .1, 0), CanvasTag, canvasSize, canvasSize/2, _canvasRes.x, _canvasRes.y, _world);
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setAppearance(_terrainShader, _nodeMaterial, _nodeTexture);
    canv->setLight(_light);
    _world->addRigidBody(canv->getRigidBody());

    btVector3 offset(0, 1.0f, 0);

    SimCreature* crtr;
    crtr = new SimCreature(btVector3(0, 0, 0), 6, _world, true);
    crtr->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    crtr->addToWorld();

    SimCreature* snake;
    snake = new SimCreature(btVector3(0, 0, 0), 0, _world, false);
    snake->initSnake(btVector3(0, 0, 24), 32, 1.0f, 0.75f, true);
    snake->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);

    _simulationInstances.push_back(new SimInstance(0, crtr, canv, _simulationTime, 10.0f));

    bTestMode = true;
}

int SimulationManager::queueSimulationInstance(const GenomeBase& genome, float duration, bool bMultiEval)
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
    // Set back to zero after a single non-parallell evaluation
    simInstanceId = (bMultiEval) ? (simInstanceId + 1) % simInstanceLimit : 0;

    return ticket;
}

int SimulationManager::runSimulationInstance(GenomeBase& genome, int ticket, float duration)
{
    int grid_x = ticket % simInstanceGridSize;
    int grid_z = ticket / simInstanceGridSize;

    btScalar spaceExt = canvasSize/2.0;
    btScalar stride = canvasSize * 2.0 + spaceExt * 2.0;
    btScalar xpos = (grid_x - simInstanceGridSize / 2) * stride + grid_x * canvasMargin;
    btScalar zpos = (grid_z - simInstanceGridSize / 2) * stride + grid_z * canvasMargin;

    btVector3 position(xpos, 0, zpos);
    btVector3 canvasPos = position + btVector3(0 , 0.05f, 0);

    SimCanvasNode* canv;
    canv = new SimCanvasNode(canvasPos, CanvasTag, canvasSize, spaceExt, _canvasRes.x, _canvasRes.y, _world);
    canv->setAppearance(_nodeShader, _terrainMaterial, _nodeTexture);
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->enableBounds();
    canv->addToWorld();

    SimCreature* crtr = (bUseMorphologyGenomes) ?
        new SimCreature(position, _testMorphologyGenome, _world) :
        new SimCreature(position, _numWalkerLegs, _world, true);

    crtr->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    crtr->setControlPolicyGenome(genome);
    crtr->addToWorld();

    _simulationInstances.push_back(new SimInstance(ticket, crtr, canv, _simulationTime, duration));

    return ticket;
}

void SimulationManager::handleCollisions(btDynamicsWorld* worldPtr)
{
    int numManifolds = worldPtr->getDispatcher()->getNumManifolds();

    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = worldPtr->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* o1 = (btCollisionObject*)(contactManifold->getBody0());
        btCollisionObject* o2 = (btCollisionObject*)(contactManifold->getBody1());

        for (int j = 0; j < contactManifold->getNumContacts(); j++)
        {
            // collision with something other than itself
            if ((o1->getUserIndex() == BodyTag && o2->getUserIndex() != BodyTag) ||
                (o1->getUserIndex() != BodyTag && o2->getUserIndex() == BodyTag))
            {
                SimCreature* creaturePtr = nullptr;

                // brushtag should always have a simcreature as user pointer
                if (o1->getUserIndex() == BodyTag) {
                    creaturePtr = (SimCreature*)o1->getUserPointer();
                    creaturePtr->setTouchSensor(o1);
                }
                else if (o2->getUserIndex() == BodyTag) {
                    creaturePtr = (SimCreature*)o2->getUserPointer();
                    creaturePtr->setTouchSensor(o2);
                }
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (worldPtr->getDebugDrawer() != NULL) {
                    worldPtr->getDebugDrawer()->drawSphere(pt.getPositionWorldOnA(), 10.0, btVector3(1., 0., 0.));
                }
            }
            // todo: Currently using bodytag because of the added complexity of also checking for brushtags. Reconsideration is required.
            if ((o1->getUserIndex() == BodyTag && o2->getUserIndex() == CanvasTag) ||
                (o1->getUserIndex() == CanvasTag && o2->getUserIndex() == BodyTag))
            {
                SimCanvasNode* canvasPtr = nullptr;

                // brushtag should always have a simcreature as user pointer
                if (o1->getUserIndex() == CanvasTag) {
                    canvasPtr = (SimCanvasNode*)o1->getUserPointer();
                }
                else if (o2->getUserIndex() == CanvasTag) {
                    canvasPtr = (SimCanvasNode*)o2->getUserPointer();
                }
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                btVector3 localPt = pt.getPositionWorldOnA() - SimUtils::glmToBullet(canvasPtr->getPosition());
                canvasPtr->addBrushStroke(localPt, pt.getAppliedImpulse());
            }
        }
        //contactManifold->clearManifold();	
    }
}

void SimulationManager::updateTime()
{
    _time = _clock.getTimeMilliseconds();
    _frameTime = _time - _prevTime;
    _prevTime = _time;

    _runTime = _time - _startTime;
    _simulationSpeed = simulationSpeed;

    // prevents physics time accumulator from summing up too much time (i.e. when debugging)
    if (_frameTime > _targetFrameTimeMillis) {
        _frameTime = _targetFrameTimeMillis;  
    }
    _frameTimeAccumulator += _frameTime;
    int steps = floor((_frameTimeAccumulator / _fixedTimeStepMillis) + 0.5);

    if (steps > 0) {
        btScalar timeToProcess = steps * _frameTime * simulationSpeed;
        while (timeToProcess >= 0.01) {
            performTrueSteps(_fixedTimeStep);
            timeToProcess -= _fixedTimeStepMillis;
        }
        // Residual value carries over to next frame
        _frameTimeAccumulator -= _frameTime * steps;
    }
}

void SimulationManager::performTrueSteps(btScalar timeStep)
{
    _world->stepSimulation(timeStep, 1, _fixedTimeStep);
    _simulationTime += timeStep;
}

void worldUpdateCallback(btDynamicsWorld* world, btScalar timeStep)
{
    SimulationManager* sim = (SimulationManager*)world->getWorldUserInfo();
    sim->updateSimInstances(timeStep);
}

void SimulationManager::updateSimInstances(double timeStep)
{
    handleCollisions(_world);

    for (auto& s : _simulationInstances) {
        s->getCreature()->update(timeStep);
        s->getCanvas()->update();
    }

    // scope for lock_guard
    {
        // create new simulation instances on main thread
        std::lock_guard<std::mutex> guard(_cbQueueMutex);
        for (simRunCallback_t cb : _simulationInstanceCallbackQueue) {
            cb(0);
        }
        _simulationInstanceCallbackQueue.clear();
    }

    // close simulation instances that are finished
    for (int i = 0; i < _simulationInstances.size(); i++) {
        if (_simulationInstances[i]->IsAborted() ||
            _simulationTime - _simulationInstances[i]->getStartTime() > _simulationInstances[i]->getDuration()) {

            ofLog() << "ended : " << _simulationInstances[i]->getID();

            ofPixels pix;
            writeToPixels(_simulationInstances[i]->getCanvas()->getCanvasFbo()->getTexture(), pix);

            double total = 0.0;
            for (ofPixels::ConstPixel p : pix.getConstPixelsIter()) {
                // black paint so calculate inverse color
                total += 1.0 - (p[0] + p[1] + p[2]) / (3.0 * 255.0);
            }
            total /= double(_canvasRes.x * _canvasRes.y);

            SimResult result;
            result.instanceId = _simulationInstances[i]->getID();
            result.fitness = total;
            onSimulationInstanceFinished.notify(result);

            delete _simulationInstances[i];
            _simulationInstances.erase(_simulationInstances.begin() + i);

            // optional
            //saveToDisk(pix);
        }
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


    _terrainNode->draw();     
    for (auto &s : _simulationInstances) {
        s->getCanvas()->draw();
        s->getCreature()->draw();
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

float SimulationManager::getSimulationTime()
{
    return _simulationTime;
}

SimCreature* SimulationManager::getFocusCreature()
{
    if (!_simulationInstances.empty()) {
        return _simulationInstances[focusIndex]->getCreature();
    }
    else return nullptr;
}

glm::vec3 SimulationManager::getFocusOrigin()
{
    if (!_simulationInstances.empty()) {
        return SimUtils::bulletToGlm(_simulationInstances[focusIndex]->getCreature()->getCenterOfMassPosition());
    }
    else return glm::vec3(0);
}

ofFbo* SimulationManager::getCanvasFbo()
{
    if (!_simulationInstances.empty()) {
        _simulationInstances[focusIndex]->getCanvas()->getCanvasFbo();
    }
    else return nullptr;
}

void SimulationManager::shiftFocus()
{
    focusIndex = (focusIndex + 1) % _simulationInstances.size();
}

void SimulationManager::nextSimulation()
{
    for (SimInstance* i : _simulationInstances) {
        i->abort();
    }
}

// As morphology is fixed this will only work for the walker creature for now
MorphologyInfo SimulationManager::getWalkerMorphologyInfo()
{
    int numBodyParts = 2 * _numWalkerLegs + 1;
    int numJoints = numBodyParts - 1;

    return MorphologyInfo(numBodyParts, numJoints);
}

DirectedGraph SimulationManager::getMorphologyGenome()
{
    return _testMorphologyGenome;
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
    return bInitialized;
}

bool SimulationManager::isSimulationInstanceActive()
{
    return !_simulationInstances.empty();
}

void SimulationManager::setMaxParallelSims(int max)
{
    simInstanceLimit = max;
    simInstanceGridSize = sqrt(max);
}

void SimulationManager::setMorphologyGenomeModeEnabled(bool b)
{
    bUseMorphologyGenomes = b;
}

bool SimulationManager::IsMorphologyGenomeModeEnabled()
{
    return bUseMorphologyGenomes;
}

void SimulationManager::dealloc()
{
    if (_terrainNode)           delete _terrainNode;

    for (auto &s : _simulationInstances) {
        delete s;
    }

    delete _world;
    delete _solver;
    delete _collisionConfiguration;
    delete _dispatcher;
    delete _broadphase;
}
