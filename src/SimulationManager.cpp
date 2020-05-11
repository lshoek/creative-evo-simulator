#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"
#include "DirectedGraph.h"
#include "SimDefines.h"

void worldUpdateCallback(btDynamicsWorld* world, btScalar timeStep);

void SimulationManager::init()
{
    _canvasRes = glm::ivec2(128, 128);
    _simulationInstances.reserve(simInstanceLimit);
    _simulationInstanceCallbackQueue.reserve(simInstanceLimit);

    // rendering -- load shaders and texture data from disk
    loadShaders();

    cam.setNearClip(0.01f);
    cam.setFarClip(1000.0f);
    cam.setPosition(8.0f, 8.0f, 4.0f);
    cam.setFixUpDirectionEnabled(true);
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

    lightPosition = MathUtils::randomPointOnSphere() * 8.0f;
    lightPosition.y = 20.0f;

    _light = std::make_shared<ofLight>();
    _light->setDirectional();
    _light->setAmbientColor(ofColor::white);
    _light->setDiffuseColor(ofColor::white);
    _light->setSpecularColor(ofColor::white);
    _light->setPosition(lightPosition);

    // physics -- init
    initPhysics();
    initTerrain();

    // pixel buffer object for writing image data to disk fast
    _imageSaverThread.setup(NTRS_ARTIFACTS_PATH);
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

    simulationSpeed = 0;
    _selectedBodyGenome = std::make_shared<DirectedGraph>(true);
    _selectedBodyGenome->unfold();
    _testCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedBodyGenome, _world);
    _testCreature->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    _testCreature->addToWorld();

    bInitialized = true;
}

void SimulationManager::startSimulation()
{
    if (bInitialized && !bSimulationActive) {
        bSimulationActive = true;
        if (_testCreature) {
            _testCreature->removeFromWorld();
        }
        simulationSpeed = 1.0; 

        // Reset timing
        _startTime = _clock.getTimeMilliseconds();
        _frameTimeAccumulator = 0.0;
        _simulationTime = 0.0;
        _clock.reset();
    }
}

void SimulationManager::stopSimulation()
{
    if (bInitialized && bSimulationActive) {
        bStopSimulationQueued = true;
        abortSimInstances();
    }
}

void SimulationManager::initPhysics()
{
    _broadphase = new btDbvtBroadphase();
    _collisionConfig = new btDefaultCollisionConfiguration();

    bool bMultiThreading = true;
    if (!bMultiThreading) {
        _dispatcher = new btCollisionDispatcher(_collisionConfig);
        _solver = new btSequentialImpulseConstraintSolver();
        _world = new btDiscreteDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfig);
    }
    else {
        _dispatcher = new btCollisionDispatcherMt(_collisionConfig);
        _solverPool = new btConstraintSolverPoolMt(BT_MAX_THREAD_COUNT);
        _solver = new btSequentialImpulseConstraintSolverMt();
        _world = new btDiscreteDynamicsWorldMt(_dispatcher, _broadphase, _solverPool, _solver, _collisionConfig);
        btSetTaskScheduler(btCreateDefaultTaskScheduler());
    }

    _dbgDrawer = new SimDebugDrawer();
    _dbgDrawer->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe | 
        btIDebugDraw::DBG_DrawConstraints
        //btIDebugDraw::DBG_DrawConstraintLimits
    );
    
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

    ofLog() << "queued sim instance " << ticket;
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

    SimCreature* crtr = (bUseBodyGenomes) ?
        new SimCreature(position, _selectedBodyGenome, _world) :
        new SimCreature(position, _numWalkerLegs, _world, true);

    crtr->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    crtr->setControlPolicyGenome(genome);
    crtr->addToWorld();

    _simulationInstances.push_back(new SimInstance(ticket, crtr, canv, _simulationTime, duration));

    ofLog() << "running sim instance " << ticket;
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
            if (o1->getUserIndex() & BodyTag && o2->getUserIndex() & ~BodyTag ||
                o1->getUserIndex() & ~BodyTag && o2->getUserIndex() & BodyTag)
            {
                SimCreature* creaturePtr = nullptr;

                // brushtag should always have a simcreature as user pointer
                if (o1->getUserIndex() & BodyTag) {
                    creaturePtr = ((SimNode*)o1->getUserPointer())->getCreaturePtr();
                    creaturePtr->setTouchSensor(o1);
                }
                else {
                    creaturePtr = ((SimNode*)o2->getUserPointer())->getCreaturePtr();
                    creaturePtr->setTouchSensor(o2);
                }
                btManifoldPoint& pt = contactManifold->getContactPoint(j);
                if (worldPtr->getDebugDrawer() != NULL) {
                    worldPtr->getDebugDrawer()->drawSphere(pt.getPositionWorldOnA(), 10.0, btVector3(1., 0., 0.));
                }
            }
            if ((o1->getUserIndex() & BrushTag && o2->getUserIndex() & CanvasTag) ||
                (o1->getUserIndex() & CanvasTag && o2->getUserIndex() & BrushTag))
            {
                SimCanvasNode* canvasPtr = nullptr;
                SimNode* brushNodePtr = nullptr;

                if (o1->getUserIndex() & CanvasTag) {
                    canvasPtr = (SimCanvasNode*)o1->getUserPointer();
                    brushNodePtr = (SimNode*)o2->getUserPointer();
                }
                else {
                    canvasPtr = (SimCanvasNode*)o2->getUserPointer();
                    brushNodePtr = (SimNode*)o1->getUserPointer();
                }

                if (brushNodePtr->isBrushActivated()) {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);
                    btVector3 localPt = pt.getPositionWorldOnA() - canvasPtr->getPosition();
                    canvasPtr->addBrushStroke(localPt, brushNodePtr->getBrushPressure());
                }
            }
        }
        //contactManifold->clearManifold();
    }
}

void SimulationManager::updateTime()
{
    if (bSimulationActive) {
        _time = _clock.getTimeMilliseconds();
        _frameTime = _time - _prevTime;
        _prevTime = _time;

        _runTime = _time - _startTime;
        _simulationSpeed = simulationSpeed;

        // prevents physics time accumulator from summing up too much time (i.e. when debugging)
        if (_frameTime > _targetFrameTimeMillis) {
            _frameTime = _targetFrameTimeMillis;
        }
        else if (_frameTime < .0) {
            _frameTime = .0;
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

            if (bSaveArtifactsToDisk) {
                saveToDisk(pix);
            }
        }
    }
    if (bStopSimulationQueued && !isSimulationInstanceActive()) {
        bSimulationActive = false;
        bStopSimulationQueued = false;
        simulationSpeed = 0;
        if (_testCreature) {
            _testCreature->addToWorld();
        }
    }
}

void SimulationManager::draw()
{
    if (bCameraSnapFocus) {
        if (getFocusCreature() != nullptr) {
            cam.lookAt(getFocusOrigin());
        }
    }
    cam.begin();

    if (!bDebugDraw) {
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
        if (bSimulationActive) {
            for (auto& s : _simulationInstances) {
                s->getCanvas()->draw();
                s->getCreature()->draw();
            }
        }
        else if (_testCreature) {
            _testCreature->draw();
        }
    }
    else {
        _world->debugDrawWorld();
        //ofDrawIcoSphere(lightPosition, 1.0f);
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
    if (!_simulationInstances.empty() && focusIndex < _simulationInstances.size()) {
        return _simulationInstances[focusIndex]->getCreature();
    }
    else return nullptr;
}

glm::vec3 SimulationManager::getFocusOrigin()
{
    if (!_simulationInstances.empty() && focusIndex < _simulationInstances.size()) {
        return SimUtils::bulletToGlm(_simulationInstances[focusIndex]->getCreature()->getCenterOfMassPosition());
    }
    else return cam.getGlobalPosition() + cam.getLookAtDir();
}

ofFbo* SimulationManager::getCanvasFbo()
{
    if (!_simulationInstances.empty() && focusIndex < _simulationInstances.size()) {
        _simulationInstances[focusIndex]->getCanvas()->getCanvasFbo();
    }
    else return nullptr;
}

void SimulationManager::shiftFocus()
{
    if (!_simulationInstances.empty() && focusIndex < _simulationInstances.size()) {
        focusIndex = (focusIndex + 1) % _simulationInstances.size();
    }
}

void SimulationManager::abortSimInstances()
{
    for (SimInstance* i : _simulationInstances) {
        i->abort();
    }
}

// As morphology is fixed this will only work for the walker creature for now
MorphologyInfo SimulationManager::getWalkerBodyInfo()
{
    int numBodyParts = 2 * _numWalkerLegs + 1;
    int numJoints = numBodyParts - 1;

    return MorphologyInfo(numBodyParts, numJoints);
}

std::shared_ptr<DirectedGraph> SimulationManager::getBodyGenome()
{
    return _selectedBodyGenome;
}

void SimulationManager::loadBodyGenomeFromDisk(std::string filename)
{
    _selectedBodyGenome = std::make_shared<DirectedGraph>(true);
    _selectedBodyGenome->load(filename);
    _selectedBodyGenome->unfold();
    _selectedBodyGenome->print();

    _testCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedBodyGenome, _world);
    _testCreature->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    _testCreature->addToWorld();
}

// Try to build a feasible creature genome
void SimulationManager::generateRandomBodyGenome()
{
    if (bUseBodyGenomes) {
        bool bNoFeasibleCreatureFound = true;
        int attempts = 0;

        ofLog() << "building feasible creature genome...";
        _selectedBodyGenome = std::make_shared<DirectedGraph>(true);
        _selectedBodyGenome->unfold();

        std::shared_ptr<SimCreature> tempCreature;
        while (bNoFeasibleCreatureFound && attempts < maxGenGenomeAttempts) {
            std::shared_ptr<SimCreature> tempCreature = std::make_shared<SimCreature>(
                btVector3(0, 2.0, 0), _selectedBodyGenome, _world
            );
            bool bSuccess = bFeasibilityChecks ? tempCreature->feasibilityCheck() : true;
            if (bSuccess) {
                bNoFeasibleCreatureFound = false;
                _selectedBodyGenome->print();

                _testCreature = tempCreature;
                _testCreature->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
                _testCreature->addToWorld();
            }
            else {
                // Replace the managed object
                _selectedBodyGenome = std::make_shared<DirectedGraph>(true);
                _selectedBodyGenome->unfold();
            }
            attempts++;
        }
        if (bFeasibilityChecks) {
            std::string logMsg = (bNoFeasibleCreatureFound) ?
                "failed! reach max attempts: " + ofToString(attempts) :
                "success! attempts: " + ofToString(attempts);
            ofLog() << logMsg;
        }
    }
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

bool SimulationManager::isSimulationActive()
{
    return bSimulationActive;
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

void SimulationManager::dealloc()
{
    if (_terrainNode) delete _terrainNode;

    for (auto &s : _simulationInstances) {
        delete s;
    }

    delete _world;
    delete _solver;
    delete _collisionConfig;
    delete _dispatcher;
    delete _broadphase;
}
