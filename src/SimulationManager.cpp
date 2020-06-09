#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"
#include "DirectedGraph.h"
#include "SimDefines.h"

void worldUpdateCallback(btDynamicsWorld* world, btScalar timeStep);

void SimulationManager::init(SimSettings settings)
{
    _canvasResolution = glm::ivec2(settings.canvasWidth, settings.canvasHeight);
    _simulationInstances.reserve(simInstanceLimit);
    _simulationInstanceCallbackQueue.reserve(simInstanceLimit);

    // rendering -- load shaders and texture data from disk
    loadShaders();

    // camera
    cam.setNearClip(0.01f);
    cam.setFarClip(1000.0f);
    cam.setPosition(8.0f, 8.0f, 4.0f);
    cam.setFixUpDirectionEnabled(true);
    cam.lookAt(glm::vec3(0));

    _nodeTexture = std::make_shared<ofTexture>();
    _terrainTexture = std::make_shared<ofTexture>();
    ofLoadImage(*_nodeTexture, "textures/white.png");
    ofLoadImage(*_terrainTexture, "textures/checkers_64.png");

    // rendering -- configure appearance
    _nodeTexture->generateMipmap();
    _nodeTexture->setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    _terrainTexture->generateMipmap();
    _terrainTexture->setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
    _terrainTexture->setTextureWrap(GL_REPEAT, GL_REPEAT);

    ofMaterialSettings mtlSettings;
    mtlSettings.ambient = ofFloatColor(0.375f, 1.0f);
    mtlSettings.diffuse = ofFloatColor(0.95f, 1.0f);
    mtlSettings.specular = ofFloatColor(1.0f, 1.0f);
    mtlSettings.emissive = ofFloatColor(0.0f, 1.0f);
    mtlSettings.shininess = 32;

    _nodeMaterial = std::make_shared<ofMaterial>();
    _nodeMaterial->setup(mtlSettings);

    mtlSettings.shininess = 64;
    _terrainMaterial = std::make_shared<ofMaterial>();
    _terrainMaterial->setup(mtlSettings);

    lightPosition = MathUtils::randomPointOnSphere() * 16.0f;
    lightPosition.y = _lightDistanceFromFocus;

    _light = std::make_shared<ofLight>();
    _light->setDirectional();
    _light->setAmbientColor(ofColor::white);
    _light->setDiffuseColor(ofColor::white);
    _light->setSpecularColor(ofColor::white);
    _light->setPosition(lightPosition);
    _light->lookAt(glm::vec3(0));

    //  shadows
    _shadowMap.setup(1024);

    // physics -- init
    initPhysics();
    initTerrain();

    // io 
    _bufferSender.setup(settings.host, settings.outPort);
    _bufferReceiver.setup(settings.inPort);
    _imageSaver.setup(_canvasResolution.x, _canvasResolution.y);

    if (ofGetTargetFrameRate()) {
        _targetFrameTimeMillis = 1000.0f/ofGetTargetFrameRate();
    }
    else {
        _targetFrameTimeMillis = _fixedTimeStepMillis;
    }
    simulationSpeed = 0;

    // creature
    if (bAutoLoadGenome) {
        loadBodyGenomeFromDisk(settings.genomeFile);
    }
    else {
        _selectedBodyGenome = std::make_shared<DirectedGraph>(true, bAxisAlignedAttachments);
        _selectedBodyGenome->unfold();
        _selectedBodyGenome->print();

        _testCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedBodyGenome, _world);
        _testCreature->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
        _testCreature->addToWorld();
    }

    bInitialized = true;
}

void SimulationManager::startSimulation(std::string id, EvaluationType evalType)
{
    if (bInitialized && !bSimulationActive) {
        bSimulationActive = true;
        
        _uniqueSimId = id;
        _simDir = NTRS_SIMS_DIR + '/' + id + '/';

        if (_testCreature) {
            _testCreature->removeFromWorld();
        }
        simulationSpeed = 1.0;

        // Reset timing
        _startTime = _clock.getTimeMilliseconds();
        _frameTimeAccumulator = 0.0;
        _simulationTime = 0.0;
        _clock.reset();

        // canvas
        evaluationType = evalType;

        _maskMat = cv::Mat(_canvasResolution.x, _canvasResolution.y, CV_8UC1);
        _maskMat = cv::Scalar(0);
        cv::circle(_maskMat, cv::Point(_maskMat.rows / 2, _maskMat.cols / 2), _maskMat.cols / 4, cv::Scalar(255), cv::FILLED);
        cv::bitwise_not(_maskMat, _invMaskMat);

        _rewardMaskPtr = &_maskMat;
        _penaltyMaskPtr = &_invMaskMat;
        if (evaluationType == EvaluationType::InverseCircleCoverage) {
            _rewardMaskPtr = &_invMaskMat;
            _penaltyMaskPtr = &_maskMat;
        }

        // Calculate maximum reward/fitness
        for (uint32_t i = 0; i < _rewardMaskPtr->total(); i++) {
            _maxReward += _rewardMaskPtr->at<uchar>(i);
        }

        // Debug Canvas Evaluation Mask
        _cvDebugImage.allocate(_maskMat.rows, _maskMat.cols);
        _cvDebugImage.setFromPixels(_maskMat.ptr<uchar>(), _maskMat.rows, _maskMat.cols);

        ofLog() << ">> Simulation ID: " << _uniqueSimId;
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
    _terrainNode->addToWorld();
} 

void SimulationManager::initTestEnvironment()
{
    SimCanvasNode* canv;
    canv = new SimCanvasNode(btVector3(0, 0, 0), canvasSize, canvasSize/2, 
        _canvasResolution.x, _canvasResolution.y, 
        _canvasNeuralInputResolution.x, _canvasNeuralInputResolution.y, _world
    );
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

    btScalar spaceExtent = canvasSize/2.0;
    btScalar stride = canvasSize * 2.0 + spaceExtent * 2.0;
    btScalar xpos = (grid_x - simInstanceGridSize / 2) * stride + grid_x * canvasMargin;
    btScalar zpos = (grid_z - simInstanceGridSize / 2) * stride + grid_z * canvasMargin;

    btVector3 position(xpos, 0, zpos);

    SimCanvasNode* canv;
    canv = new SimCanvasNode(position, canvasSize, spaceExtent, 
        _canvasResolution.x, _canvasResolution.y, 
        _canvasNeuralInputResolution.x, _canvasNeuralInputResolution.y, _world
    );
    canv->setAppearance(_nodeShader, _terrainMaterial, _nodeTexture);
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setCanvasColorizeShader(_canvasColorShader);
    canv->enableBounds();
    canv->addToWorld();

    SimCreature* crtr = (bUseBodyGenomes) ?
        new SimCreature(position, _selectedBodyGenome, _world) :
        new SimCreature(position, _numWalkerLegs, _world, true);

    crtr->setSensorMode(bCanvasInputNeurons ? SimCreature::Canvas : SimCreature::Touch);
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
            if (!bCanvasInputNeurons &&
                o1->getUserIndex() & BodyTag && o2->getUserIndex() & ~BodyTag ||
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

void SimulationManager::lateUpdate()
{
    if (bMouseLight) {
        float longitude = ofMap(ofGetMouseX(), 0, ofGetWidth(), -PI, PI);
        float latitude = ofMap(ofGetMouseY(), 0, ofGetHeight(), -HALF_PI, HALF_PI*0.25f);
        _light->orbitRad(longitude, latitude, _lightDistanceFromFocus);
    }
    else {
        _light->setGlobalPosition(lightPosition);
        _light->lookAt(glm::vec3(0));
    }
    if (bCameraSnapFocus) {
        cam.lookAt(getFocusOrigin());
    }
    setLightUniforms(_nodeShader);
    setLightUniforms(_terrainShader);
}

void SimulationManager::setLightUniforms(const std::shared_ptr<ofShader>& shader)
{
    shader->begin();
    shader->setUniform3f("light.position", _light->getPosition());
    shader->setUniform3f("light.direction", _light->getLookAtDir());
    shader->setUniform4f("light.ambient", _light->getAmbientColor());
    shader->setUniform4f("light.diffuse", _light->getDiffuseColor());
    shader->setUniform4f("light.specular", _light->getSpecularColor());
    shader->setUniform3f("eyePos", cam.getPosition());
    shader->end();
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
    if (sim->isSimulationActive()) {
        sim->updateSimInstances(timeStep);
    }
}

void SimulationManager::updateSimInstances(double timeStep)
{
    handleCollisions(_world);

    for (auto& s : _simulationInstances) {
        
        // Update canvas every timestep
        s->getCanvas()->update();

        bool bUpdateActuators = s->getCreature()->updateTimeStep(timeStep);
        if (bUpdateActuators) {
            if (bCanvasInputNeurons) {

                s->getCanvas()->updateNeuralInputBuffer(true);

                if (learningMode == LearningMode::External) {
                    uint64_t start = ofGetElapsedTimeMillis();

                    _bufferSender.send(s->getCanvas()->getNeuralInputPixelBuffer());
                    const std::vector<float> outputs = _bufferReceiver.receive();

                    uint64_t total = ofGetElapsedTimeMillis() - start;
                    ofLog() << "total latency: " << total << "ms";

                    s->getCreature()->setOutputs(outputs);
                }
                else {
                    double t = (_simulationTime - s->getStartTime()) / double(s->getDuration());
                    s->getCreature()->setCanvasSensors(s->getCanvas()->getNeuralInputsBufferDouble(), t);
                    s->getCreature()->activate();
                }
            } 
            else {
                s->getCreature()->activate();
            }
            s->getCreature()->update();
            s->getCanvas()->update();
        }
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

            uint32_t creatureId = _simulationInstances[i]->getCreature()->getControlPolicyGenome()->getGenome().GetID();
            double fitness = evaluateArtifact(_simulationInstances[i]);

            ofLog() << "ended (" << _simulationInstances[i]->getID() << ") id: " << creatureId << " fitness: " << fitness;

            if (bSaveArtifactsToDisk) {
                std::string path = _simDir + '/' + NTRS_ARTIFACTS_PREFIX + ofToString(creatureId) + '_' + ofToString(fitness, 2);
                _imageSaver.save(_simulationInstances[i]->getCanvas()->getCanvasFbo()->getTexture(), path);
            }

            SimResult result;
            result.instanceId = _simulationInstances[i]->getID();
            result.fitness = fitness;
            onSimulationInstanceFinished.notify(result);

            delete _simulationInstances[i];
            _simulationInstances.erase(_simulationInstances.begin() + i);
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

double SimulationManager::evaluateArtifact(SimInstance* instance)
{
    _imageSaver.copyToBuffer(instance->getCanvas()->getCanvasRawFbo()->getTexture(), [&](uint8_t* p) {
        _artifactMat = cv::Mat(_canvasResolution.x, _canvasResolution.y, CV_8UC1, p);
    });

    double total = 0.0;
    double fitness = 0.0;

    if (evaluationType == EvaluationType::Coverage) {
        for (uint32_t i = 0; i < _artifactMat.total(); i++) {
            total += _artifactMat.at<uchar>(i);
        }
        fitness = total / double(_artifactMat.total() * 255);
    }
    else {
        cv::bitwise_and(_artifactMat, *_rewardMaskPtr, _rewardMat);
        cv::bitwise_and(_artifactMat, *_penaltyMaskPtr, _penaltyMat);

        for (uint32_t i = 0; i < _artifactMat.total(); i++) {
            total += _rewardMat.at<uchar>(i);
            total -= _penaltyMat.at<uchar>(i);
        }
        fitness = total / _maxReward;

        if (bViewCanvasEvaluationMask) {
            _cvDebugImage.setFromPixels(_rewardMat.ptr<uchar>(), _artifactMat.cols, _artifactMat.rows);
        }
    }
    return fitness;
}

void SimulationManager::shadowPass()
{
    if (!bDebugDraw && bShadows) {
        _shadowMap.begin(*_light, 64.0f, -32.0f, 64.0f);

        // make an exception for the terrain
        glDisable(GL_CULL_FACE);
        _terrainNode->drawImmediate();

        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        if (bSimulationActive) {
            for (SimInstance* s : _simulationInstances)
                s->getCreature()->drawImmediate();
        }
        else if (_testCreature) {
            _testCreature->drawImmediate();
        }
        _shadowMap.end();
        _shadowMap.updateShader(_terrainShader);
        _shadowMap.updateShader(_nodeShader);
    }
}

void SimulationManager::draw()
{
    if (!bDebugDraw) {
        cam.begin();

        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        _terrainNode->draw();
        glDisable(GL_CULL_FACE);

        if (bSimulationActive) {

            glDisable(GL_DEPTH_TEST);

            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            for (SimInstance* s : _simulationInstances)
                s->getCanvas()->draw();

            glEnable(GL_DEPTH_TEST);

            for (SimInstance* s : _simulationInstances)
                s->getCreature()->draw();

            glDisable(GL_CULL_FACE);
        }
        else if (_testCreature) {
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            _testCreature->draw();
            glDisable(GL_CULL_FACE);
        }

        // gizmos
        if (bMouseLight) {
            ofDrawIcoSphere(_light->getGlobalPosition(), 2.0f);
            ofDrawArrow(_light->getGlobalPosition(), _light->getGlobalPosition() + _light->getLookAtDir() * 4.0f);
        }
        cam.end();

        if (bViewLightSpaceDepth) {
            glDisable(GL_DEPTH_TEST);
            _shadowMap.getDepthTexture().draw(ofGetWidth() - 256, 0, 256, 256);
            glEnable(GL_DEPTH_TEST);
        }
        if (bViewCanvasEvaluationMask) {
            _cvDebugImage.draw(ofGetWidth() - 256, ofGetHeight() - 256, 256, 256);
        }
    }
    else {
        cam.begin();
        _world->debugDrawWorld();
        cam.end();
    }
}

std::string SimulationManager::getUniqueSimId()
{
    return _uniqueSimId;
}

const std::string& SimulationManager::getStatus()
{
    return _status;
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

SimCanvasNode* SimulationManager::getFocusCanvas()
{
    if (!_simulationInstances.empty() && focusIndex < _simulationInstances.size()) {
        _simulationInstances[focusIndex]->getCanvas();
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

glm::ivec2 SimulationManager::getCanvasResolution()
{
    return _canvasResolution;
}

glm::ivec2 SimulationManager::getCanvasNeuronInputResolution()
{
    return _canvasNeuralInputResolution;
}

void SimulationManager::loadBodyGenomeFromDisk(std::string filename)
{
    _selectedBodyGenome = std::make_shared<DirectedGraph>(true, bAxisAlignedAttachments);
    _selectedBodyGenome->load(filename);
    _selectedBodyGenome->unfold();
    _selectedBodyGenome->print();

    _testCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedBodyGenome, _world);
    _testCreature->setAppearance(_nodeShader, _nodeMaterial, _nodeTexture);
    _testCreature->addToWorld();

    char label[256];
    sprintf_s(label, "Loaded genome '%s' with a total of %d nodes, %d joints, %d brushes, %d outputs", filename.c_str(),
        _selectedBodyGenome->getNumNodesUnfolded(),
        _selectedBodyGenome->getNumJointsUnfolded(),
        _selectedBodyGenome->getNumEndNodesUnfolded(),
        _selectedBodyGenome->getNumJointsUnfolded() + _selectedBodyGenome->getNumEndNodesUnfolded()
    );
    _status = label;
}

// Try to build a feasible creature genome
void SimulationManager::generateRandomBodyGenome()
{
    if (bUseBodyGenomes) {
        bool bNoFeasibleCreatureFound = true;
        int attempts = 0;

        ofLog() << "building feasible creature genome...";
        _selectedBodyGenome = std::make_shared<DirectedGraph>(true, bAxisAlignedAttachments);
        _selectedBodyGenome->unfold();

        std::shared_ptr<SimCreature> tempCreature;
        while (bNoFeasibleCreatureFound && attempts < maxGenGenomeAttempts) {

            tempCreature = std::make_shared<SimCreature>(
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
                _selectedBodyGenome = std::make_shared<DirectedGraph>(true, bAxisAlignedAttachments);
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

//void SimulationManager::writeToPixels(const ofTexture& tex, ofPixels& pix)
//{
//    tex.copyTo(*pboPtr);
//
//    pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);
//    unsigned char* p = pboPtr->map<unsigned char>(GL_READ_ONLY);
//    pix.setFromExternalPixels(p, _canvasResolution.x, _canvasResolution.y, OF_PIXELS_RGBA);
//    
//    pboPtr->unmap();
//    pboPtr->unbind(GL_PIXEL_UNPACK_BUFFER);
//
//    swapPbo();
//}

void SimulationManager::loadShaders()
{
    _terrainShader = std::make_shared<ofShader>();
    _nodeShader = std::make_shared<ofShader>();
    _canvasColorShader = std::make_shared<ofShader>();
    _canvasUpdateShader = std::make_shared<ofShader>();

    if (bShadows) {
        _terrainShader->load("shaders/checkersShadow");
        _nodeShader->load("shaders/phongShadow");
    } 
    else {
        _terrainShader->load("shaders/checkers");
        _nodeShader->load("shaders/phong");
    }
    _canvasColorShader->load("shaders/lum2col");
    _canvasUpdateShader->load("shaders/canvas");

    //_terrainShader->begin();
    //_terrainShader->setUniform4f("checkers_pos", ofColor::fromHsb(ofRandom(255), 0.75f * 255, 0.75f*255, 255));
    //_terrainShader->end();
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

void SimulationManager::setCanvasNeuronInputResolution(uint32_t width, uint32_t height)
{
    _canvasNeuralInputResolution = glm::ivec2(width, height);
    _bufferSender.allocate(width, height, OF_PIXELS_GRAY);
    _bufferReceiver.allocate(16);
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
