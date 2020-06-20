#include "SimulationManager.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "ofLog.h"
#include "ofMath.h"
#include "toolbox.h"
#include "DirectedGraph.h"
#include "SimDefines.h"
#include "OscProtocol.h"

void SimulationManager::init(SimSettings settings)
{
    // canvas
    _canvasResolution = glm::ivec2(settings.canvasSize, settings.canvasSize);
    _canvasConvResolution = glm::ivec2(settings.canvasSizeConv, settings.canvasSizeConv);
    bCanvasDownSampling = _canvasResolution.x != _canvasConvResolution.x;

    // parallellism
    _simInstanceLimit = settings.maxParallelSims;
    _simInstanceGridSize = sqrt(_simInstanceLimit);

    _simulationInstances.reserve(_simInstanceLimit);
    _simulationInstanceCallbackQueue.reserve(_simInstanceLimit);

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

    mtlSettings.shininess = 16;
    _canvasMaterial = std::make_shared<ofMaterial>();
    _canvasMaterial->setup(mtlSettings);

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

    // preview world
    _previewWorld = new SimWorld();
    _previewWorld->getTerrainNode()->setAppearance(_terrainShader, _terrainMaterial, _terrainTexture);
    _previewWorld->getTerrainNode()->setLight(_light);

    // time
    if (ofGetTargetFrameRate()) {
        _targetFrameTimeMillis = 1000.0f/ofGetTargetFrameRate();
    }
    else {
        _targetFrameTimeMillis = FixedTimeStepMillis;
    }
    simulationSpeed = 0;

    // creature
    bGenomeLoaded = false;
    if (bAutoLoadGenome) {
        bGenomeLoaded = loadBodyGenomeFromDisk(settings.genomeFile);
    }
    if (!bGenomeLoaded) {
        _selectedBodyGenome = std::make_shared<DirectedGraph>(true, bAxisAlignedAttachments);
        _selectedBodyGenome->unfold();
        _selectedBodyGenome->print();

        _previewCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedBodyGenome, _previewWorld->getBtWorld());
        _previewCreature->setMaterial(_nodeMaterial);
        _previewCreature->setShader(_nodeShader);
        _previewCreature->addToWorld();
    }

    // io 
    _networkManager.setup(settings.host, settings.inPort, settings.outPort);
    _imageSaver.setup(_canvasResolution.x, _canvasResolution.y);

    bInitialized = true;
}

void SimulationManager::startSimulation(std::string id, EvaluationType evalType)
{
    if (bInitialized && !bSimulationActive) {
        bSimulationActive = true;
        
        _uniqueSimId = id;
        _simDir = NTRS_SIMS_DIR + '/' + id + '/';

        if (_previewCreature) {
            _previewCreature->removeFromWorld();
        }
        simulationSpeed = 1.0;

        // Reset timing
        _startTime = _clock.getTimeMilliseconds();
        _frameTimeAccumulator = 0.0;
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

        // Register network event listeners
        _connectionEstablishedListener = _networkManager.onConnectionEstablished.newListener([this] {
            setStatus("Connection with evolution module established!");
            uint32_t numJoints = _selectedBodyGenome->getNumJointsUnfolded();

            _networkManager.send(OSC_INFO + '/' +
                _selectedBodyGenome->getName() + '/' +
                ofToString(numJoints) + '/' +
                ofToString(numJoints+1) + '/' +
                ofToString(_canvasConvResolution.x)
            );
        });
        _infoReceivedListener = _networkManager.onInfoReceived.newListener([this] (NetworkManager::SimInfo info) {
            queueSimInstance(info.id, info.generation, info.duration);
        });
        _connectionClosedListener = _networkManager.onConnectionClosed.newListener([this] {
            stopSimulation();
        });
        uint32_t numJoints = _selectedBodyGenome->getNumJointsUnfolded();
        _networkManager.allocate(numJoints, numJoints + 1, _canvasConvResolution.x, _canvasConvResolution.y, OF_PIXELS_GRAY);
        _networkManager.search();

        ofLog() << ">> Simulation ID: " << _uniqueSimId;
        setStatus("Awaiting evolution module input...");
    }
}

void SimulationManager::stopSimulation()
{
    if (bInitialized && bSimulationActive) {
        bStopSimulationQueued = true;
        abortSimInstances();

        _networkManager.close();
        _connectionEstablishedListener.unsubscribe();
        _connectionClosedListener.unsubscribe();
        _infoReceivedListener.unsubscribe();
    }
}

int SimulationManager::queueSimInstance(int localId, int generation, float duration)
{
    {
        std::lock_guard<std::mutex> guard(_cbQueueMutex);
        _simulationInstanceCallbackQueue.push_back(
            std::bind(&SimulationManager::createSimInstance, this, localId, generation, duration)
        );
    }

    if (bMultiEval) {
        // Set simInstanceIdCounter back to zero after a single non-parallell evaluation
        int worldId = _simInstanceIdCounter; // Not really using this anymore
        _simInstanceIdCounter = (bMultiEval) ? (_simInstanceIdCounter + 1) % _simInstanceLimit : 0;
    }

    setStatus("Queued simulation instance. /Global id: " + ofToString(localId) + " /Local id: " + ofToString(localId));
    return localId;
}

int SimulationManager::createSimInstance(int localId, int generation, float duration)
{
    int grid_x = localId % _simInstanceGridSize;
    int grid_z = localId / _simInstanceGridSize;

    btScalar spaceExtent = canvasSize/2.0;
    btScalar stride = canvasSize * 2.0 + spaceExtent * 2.0;
    btScalar xpos = (grid_x - _simInstanceGridSize / 2) * stride + grid_x * canvasMargin;
    btScalar zpos = (grid_z - _simInstanceGridSize / 2) * stride + grid_z * canvasMargin;
    btVector3 position = (bMultiEval) ? btVector3(xpos, 0, zpos) : btVector3(0, 0, 0);

    SimWorld* world = new SimWorld();

    SimCanvasNode* canv;
    canv = new SimCanvasNode(position, canvasSize, spaceExtent, 
        _canvasResolution.x, _canvasResolution.y, 
        _canvasConvResolution.x, _canvasConvResolution.y, 
        bCanvasDownSampling, world->getBtWorld()
    );
    canv->setMaterial(_canvasMaterial);
    canv->setShader(_canvasShader);
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setCanvasColorizeShader(_canvasColorShader);
    canv->enableBounds();
    canv->addToWorld();

    SimCreature* crtr = new SimCreature(position, _selectedBodyGenome, world->getBtWorld());

    crtr->setSensorMode(bCanvasSensors ? SimCreature::Canvas : SimCreature::Touch);
    crtr->setMaterial(_nodeMaterial);
    crtr->setShader(_nodeShader);
    crtr->addToWorld();

    SimInstance* instance = new SimInstance(localId, generation, world, crtr, canv, duration);
    _networkManager.sendState(instance);
    _simulationInstances.push_back(instance);

    setStatus("Running simulation instance [GEN:" + ofToString(generation) + "] [ID:" + ofToString(localId) + "]");
    return localId;
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
    setLightUniforms(_canvasShader);
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

void SimulationManager::update()
{
    _networkManager.receive();

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
        int steps = floor((_frameTimeAccumulator / FixedTimeStepMillis) + 0.5);

        if (steps > 0) {
            btScalar timeToProcess = steps * _frameTime * simulationSpeed;
            while (timeToProcess >= 0.01) {
                performTrueSteps(FixedTimeStep);
                timeToProcess -= FixedTimeStepMillis;
            }
            // Residual value carries over to next frame
            _frameTimeAccumulator -= _frameTime * steps;
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
    int i = 0;
    bool bInstanceDestroyed = false;
    for (auto& instance : _simulationInstances) {
        if (instance->isFinished()) {

            uint32_t id = instance->getID();
            double fitness = evaluateArtifact(instance);

            ofLog() << "Ended (" << instance->getID() << ") id: " << id << " fitness: " << fitness;

            if (bSaveArtifactsToDisk) {
                std::string path = _simDir + '/' + NTRS_ARTIFACTS_PREFIX + ofToString(id) + '_' + ofToString(fitness, 2);
                _imageSaver.save(instance->getCanvas()->getCanvasFbo()->getTexture(), path);
            }
            _networkManager.send(OSC_FITNESS + '/' + ofToString(id), fitness);

            delete instance;
            _simulationInstances.erase(_simulationInstances.begin() + i);
            bInstanceDestroyed = true;
            
            i++;
        }
    }
    // quick and dirty
    if (bInstanceDestroyed) {
        _networkManager.search();
    }
    if (bStopSimulationQueued && !isSimulationInstanceActive()) {
        bSimulationActive = false;
        bStopSimulationQueued = false;
        simulationSpeed = 0;
        if (_previewCreature) {
            _previewCreature->addToWorld();
        }
    }
}

void SimulationManager::performTrueSteps(btScalar timeStep)
{
    for (auto& instance : _simulationInstances) {
        updateSimInstance(instance, timeStep);
    }
}

void SimulationManager::updateSimInstance(SimInstance* instance, double timeStep)
{
    // Update siminstance timestep if the creature is not waiting
    bool bTimeStepUpdate = true;
    if (instance->isEffectorUpdateRequired()) {
        bTimeStepUpdate = false;
        if (_networkManager.isAgentOutputQueued()) {
            if (_networkManager.getQueuedAgentId() == instance->getID()) {

                // update effectors
                instance->getCreature()->setOutputs(_networkManager.popOutputBuffer());

                // send new observation
                instance->getCanvas()->updateConvPixelBuffer();
                _networkManager.sendState(instance);
                bTimeStepUpdate = true;
            }
        }
    }
    if (bTimeStepUpdate) {
        instance->updateTimeStep(timeStep);
        instance->update();
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
        _previewWorld->getTerrainNode()->drawImmediate();

        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        if (bSimulationActive) {
            for (auto& instance : _simulationInstances)
                instance->getCreature()->drawImmediate();
        }
        else if (_previewCreature) {
            _previewCreature->drawImmediate();
        }
        _shadowMap.end();
        _shadowMap.updateShader(_terrainShader);
        _shadowMap.updateShader(_nodeShader);
        _shadowMap.updateShader(_canvasShader);
    }
}

void SimulationManager::draw()
{
    if (!bDebugDraw) {
        cam.begin();

        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        _previewWorld->getTerrainNode()->draw();
        glDisable(GL_CULL_FACE);

        if (bSimulationActive) {

            glDisable(GL_DEPTH_TEST);

            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            for (auto& instance : _simulationInstances)
                instance->getCanvas()->draw();

            glEnable(GL_DEPTH_TEST);

            for (auto& instance : _simulationInstances)
                instance->getCreature()->draw();

            glDisable(GL_CULL_FACE);
        }
        else if (_previewCreature) {
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            _previewCreature->draw();
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
        _previewWorld->getBtWorld()->debugDrawWorld();
        cam.end();
    }
}

std::string SimulationManager::getUniqueSimId()
{
    return _uniqueSimId;
}

void SimulationManager::setStatus(std::string msg)
{
    _status = msg;
}

const std::string& SimulationManager::getStatus()
{
    return _status;
}

ofxGrabCam* SimulationManager::getCamera()
{
    return &cam;
}

SimCreature* SimulationManager::getFocusCreature()
{
    if (!_simulationInstances.empty() && _focusIndex < _simulationInstances.size()) {
        return _simulationInstances[_focusIndex]->getCreature();
    }
    else return nullptr;
}

SimCanvasNode* SimulationManager::getFocusCanvas()
{
    if (!_simulationInstances.empty() && _focusIndex < _simulationInstances.size()) {
        _simulationInstances[_focusIndex]->getCanvas();
    }
    else return nullptr;
}

glm::vec3 SimulationManager::getFocusOrigin()
{
    if (!_simulationInstances.empty() && _focusIndex < _simulationInstances.size()) {
        return SimUtils::bulletToGlm(_simulationInstances[_focusIndex]->getCreature()->getCenterOfMassPosition());
    }
    else return cam.getGlobalPosition() + cam.getLookAtDir();
}

std::string SimulationManager::getFocusInfo()
{
    if (!_simulationInstances.empty() && _focusIndex < _simulationInstances.size()) {
        SimInstance* instance = _simulationInstances[_focusIndex];
        return ofToString(instance->getElapsedTime(), 2) + '/' + ofToString(instance->getDuration(), 2);
    }
    else return "NA";
}

void SimulationManager::shiftFocus()
{
    if (!_simulationInstances.empty() && _focusIndex < _simulationInstances.size()) {
        _focusIndex = (_focusIndex + 1) % _simulationInstances.size();
    }
}

void SimulationManager::abortSimInstances()
{
    for (auto& instance : _simulationInstances) {
        instance->abort();
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

glm::ivec2 SimulationManager::getCanvasConvResolution()
{
    return _canvasConvResolution;
}

bool SimulationManager::loadBodyGenomeFromDisk(std::string filename)
{
    _selectedBodyGenome = std::make_shared<DirectedGraph>(true, bAxisAlignedAttachments);

    if (_selectedBodyGenome->load(filename)) {
        _selectedBodyGenome->unfold();
        _selectedBodyGenome->print();

        _previewCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedBodyGenome, _previewWorld->getBtWorld());
        _previewCreature->setMaterial(_nodeMaterial);
        _previewCreature->setShader(_nodeShader);
        _previewCreature->addToWorld();

        char label[256];
        sprintf_s(label, "Loaded genome '%s' with a total of %d nodes, %d joints, %d brushes, %d outputs.", filename.c_str(),
            _selectedBodyGenome->getNumNodesUnfolded(),
            _selectedBodyGenome->getNumJointsUnfolded(),
            _selectedBodyGenome->getNumEndNodesUnfolded(),
            _selectedBodyGenome->getNumJointsUnfolded() + _selectedBodyGenome->getNumEndNodesUnfolded()
        );
        setStatus(label);
        return true;
    }
    else {
        setStatus("Failed to load genome '" + filename + "'.");
        return false;
    }
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
        while (bNoFeasibleCreatureFound && attempts < _maxGenGenomeAttempts) {

            tempCreature = std::make_shared<SimCreature>(
                btVector3(0, 2.0, 0), _selectedBodyGenome, _previewWorld->getBtWorld()
            );
            bool bSuccess = bFeasibilityChecks ? tempCreature->feasibilityCheck() : true;
            if (bSuccess) {
                bNoFeasibleCreatureFound = false;
                _selectedBodyGenome->print();

                _previewCreature = tempCreature;
                _previewCreature->setMaterial(_nodeMaterial);
                _previewCreature->setShader(_nodeShader);
                _previewCreature->addToWorld();
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

void SimulationManager::loadShaders()
{
    _terrainShader = std::make_shared<ofShader>();
    _nodeShader = std::make_shared<ofShader>();
    _canvasShader = std::make_shared<ofShader>();
    _canvasColorShader = std::make_shared<ofShader>();
    _canvasUpdateShader = std::make_shared<ofShader>();

    if (bShadows) {
        _terrainShader->load("shaders/checkersPhongShadow");
        _nodeShader->load("shaders/framePhongShadow");
        _canvasShader->load("shaders/texturePhongShadow");
    }
    else {
        _terrainShader->load("shaders/checkers");
        _nodeShader->load("shaders/phong");
        _canvasShader->load("shaders/phong");
    }
    _canvasColorShader->load("shaders/lum2col");
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

void SimulationManager::dealloc()
{
    delete _previewWorld;

    for (auto &instance : _simulationInstances) {
        delete instance;
    }
    _networkManager.close();
}
