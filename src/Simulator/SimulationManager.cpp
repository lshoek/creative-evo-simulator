#include "Simulator/SimulationManager.h"
#include "Utils/SimUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/toolbox.h"
#include "Genome/DirectedGraph.h"
#include "Simulator/SimDefines.h"
#include "Networking/OscProtocol.h"
#include "ofLog.h"
#include "ofMath.h"

void SimulationManager::init(SimSettings settings)
{
    // canvas
    _canvasResolution = glm::ivec2(settings.canvasSize, settings.canvasSize);
    _canvasConvResolution = glm::ivec2(settings.canvasSizeConv, settings.canvasSizeConv);
    
    if (!bCanvasLocalVisionMode) {
        bCanvasDownSampling = _canvasResolution.x != _canvasConvResolution.x;
    }

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
        bGenomeLoaded = loadGenomeFromDisk(settings.genomeFile);
    }
    if (!bGenomeLoaded) {
        _selectedGenome = std::make_shared<DirectedGraph>(genomeGenMinNumNodes, genomeGenMinNumConns, bAxisAlignedAttachments);
        _selectedGenome->unfold();
        _selectedGenome->print();

        _previewCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedGenome, _previewWorld->getBtWorld());
        _previewCreature->setMaterial(_nodeMaterial);
        _previewCreature->setShader(_nodeShader);
        _previewCreature->addToWorld();
    }

    // io 
    _imageSaver.setup(_canvasResolution.x, _canvasResolution.y);
    _cpgQueue.allocate(32);

    // eval
    //cv::Mat testImg = cv::imread("data/lenna.bmp", cv::ImreadModes::IMREAD_GRAYSCALE);
    //_evaluator.setup();
    //_evaluator.evaluate(testImg);

    _settings = settings;
    bInitialized = true;
}

void SimulationManager::startSimulation(std::string id)
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

        _maskMat = cv::Mat(_canvasResolution.x, _canvasResolution.y, CV_8UC1);
        _maskMat = cv::Scalar(0);
        cv::circle(_maskMat, cv::Point(_maskMat.rows / 2, _maskMat.cols / 2), _maskMat.cols / 4, cv::Scalar(255), cv::FILLED);
        cv::bitwise_not(_maskMat, _invMaskMat);

        _rewardMaskPtr = &_maskMat;
        _penaltyMaskPtr = &_invMaskMat;
        if (_evaluationType == EvaluationType::InverseCircleCoverage) {
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
        _networkManager.setup(_settings.host, _settings.inPort, _settings.outPort);
        _connectionEstablishedListener = _networkManager.onConnectionEstablished.newListener([this] {
            setStatus("Connection with evolution module established!");
            uint32_t numJoints = _selectedGenome->getNumJointsUnfolded();

            _networkManager.send(OSC_INFO + '/' +
                _selectedGenome->getName() + '/' +
                ofToString(numJoints) + '/' +
                ofToString(numJoints+1) + '/' +
                ofToString(_canvasConvResolution.x)
            );
        });
        _infoReceivedListener = _networkManager.onInfoReceived.newListener([this](SimInfo info) {
            queueSimInstance(info);
        });
        _pulseReceivedListener = _networkManager.onPulseReceived.newListener([this](float pulse) {
            _cpgQueue.push(pulse);
        });
        _connectionClosedListener = _networkManager.onConnectionClosed.newListener([this] {
            stopSimulation();
        });
        uint32_t numJoints = _selectedGenome->getNumJointsUnfolded();
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
        terminateSimInstances();

        _connectionEstablishedListener.unsubscribe();
        _connectionClosedListener.unsubscribe();
        _infoReceivedListener.unsubscribe();
        _pulseReceivedListener.unsubscribe();
        _networkManager.close();
    }
}

int SimulationManager::queueSimInstance(SimInfo info)
{
    {
        std::lock_guard<std::mutex> guard(_cbQueueMutex);
        _simulationInstanceCallbackQueue.push_back(
            std::bind(&SimulationManager::createSimInstance, this, info)
        );
    }

    if (bMultiEval) {
        // Set simInstanceIdCounter back to zero after a single non-parallell evaluation
        int worldId = _simInstanceIdCounter; // Not really using this anymore
        _simInstanceIdCounter = (bMultiEval) ? (_simInstanceIdCounter + 1) % _simInstanceLimit : 0;
    }

    setStatus("Queued simulation instance. /Global id: " + ofToString(info.generation) + 
        " /Local id: " + ofToString(info.id) + 
        " /Evaluation id: " + ofToString(info.evalId)
    );
    return info.id;
}

int SimulationManager::createSimInstance(SimInfo info)
{
    int grid_x = info.id % _simInstanceGridSize;
    int grid_z = info.id / _simInstanceGridSize;

    btScalar spaceExtent = canvasSize/2.0;
    btScalar stride = canvasSize * 2.0 + spaceExtent * 2.0;
    btScalar xpos = (grid_x - _simInstanceGridSize / 2) * stride + grid_x * canvasMargin;
    btScalar zpos = (grid_z - _simInstanceGridSize / 2) * stride + grid_z * canvasMargin;
    btVector3 position = (bMultiEval) ? btVector3(xpos, 0, zpos) : btVector3(0, 0, 0);

    SimWorld* world = new SimWorld();

    SimCreature* crtr = new SimCreature(position, _selectedGenome, world->getBtWorld());
    crtr->setSensorMode(bCanvasSensors ? SimCreature::Canvas : SimCreature::Touch);
    crtr->setMaterial(_nodeMaterial);
    crtr->setShader(_nodeShader);
    crtr->addToWorld();

    SimCanvasNode* canv = new SimCanvasNode(position, canvasSize, spaceExtent, 
        _canvasResolution.x, _canvasResolution.y, 
        _canvasConvResolution.x, _canvasConvResolution.y, 
        bCanvasLocalVisionMode, bCanvasDownSampling, world->getBtWorld()
    );
    canv->setMaterial(_canvasMaterial);
    canv->setShader(_canvasShader);
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setCanvasColorizeShader(_canvasColorShader);
    canv->setSubTextureShader(_canvasSubTextureShader);
    canv->enableBounds();
    canv->addToWorld();

    SimInstance* instance = new SimInstance(info.id, info.generation, world, crtr, canv, info.duration);
    _networkManager.sendState(instance);
    _simulationInstances.push_back(instance);

    setStatus("Running simulation instance [GEN:" + ofToString(info.generation) + "] [ID:" + ofToString(info.id) + "] [EVAL:" + ofToString(info.evalId) + "]");
    return info.id;
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

        _timeStepsPerUpdate = 0;
        if (steps > 0) {
            btScalar timeToProcess = steps * _frameTime * _simulationSpeed;
            while (timeToProcess >= 0.01) {
                performTrueSteps(FixedTimeStep);
                timeToProcess -= FixedTimeStepMillis;
                _timeStepsPerUpdate++;
            }
            // Residual value carries over to next frame
            _frameTimeAccumulator -= _frameTime * steps;
        }
    }

    // scope for lock_guard
    {
        // create new simulation instances on main thread
        std::lock_guard<std::mutex> guard(_cbQueueMutex);
        for (const auto& cb : _simulationInstanceCallbackQueue) {
            cb(0);
        }
        _simulationInstanceCallbackQueue.clear();
    }

    // close simulation instances that are finished
    int i = 0;
    bool bInstanceDestroyed = false;
    for (auto& instance : _simulationInstances) {
        if (instance->isFinished() || instance->isTerminated()) {

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
    if (bInstanceDestroyed && !bStopSimulationQueued) {
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
    instance->updateTimeStep(timeStep);

    if (instance->isEffectorUpdateRequired()) {
        bool bEffectorsQueued =
            _networkManager.isAgentOutputQueued() &&
            _networkManager.getQueuedAgentId() == instance->getID();

        if (bEffectorsQueued) {
            // update effectors
            instance->getCreature()->updateOutputs(_networkManager.popOutputBuffer());
            instance->update();

            // send new observation
            instance->getCanvas()->updateConvPixelBuffer();
            _networkManager.sendState(instance);
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

    if (_evaluationType == EvaluationType::Coverage) {
        total = cv::sum(_artifactMat)[0];
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
            for (const auto& instance : _simulationInstances)
                instance->getCanvas()->draw();

            glEnable(GL_DEPTH_TEST);

            for (const auto& instance : _simulationInstances)
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
        for (const auto& instance : _simulationInstances) {
            instance->getWorld()->getBtWorld()->debugDrawWorld();
        }
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

void SimulationManager::terminateSimInstances()
{
    for (auto &instance : _simulationInstances) {
        instance->terminate();
    }
}

const std::shared_ptr<DirectedGraph>& SimulationManager::getSelectedGenome()
{
    return _selectedGenome;
}

glm::ivec2 SimulationManager::getCanvasResolution()
{
    return _canvasResolution;
}

glm::ivec2 SimulationManager::getCanvasConvResolution()
{
    return _canvasConvResolution;
}

uint32_t SimulationManager::getTimeStepsPerUpdate()
{
    return _timeStepsPerUpdate;
}

const std::vector<float> SimulationManager::getCPGBuffer()
{
    return _cpgQueue.getBuffer();
}

SimulationManager::EvaluationType SimulationManager::getEvaluationType()
{
    return _evaluationType;
}

std::string SimulationManager::getEvaluationTypeStr()
{
    switch(_evaluationType) {
        case Coverage: return "Coverage"; break;
        case CircleCoverage: return "CircleCoverage"; break;
        case InverseCircleCoverage: return "InverseCircleCoverage"; break;
        default: return "NA";
    }
}

bool SimulationManager::loadGenomeFromDisk(std::string filename)
{
    _selectedGenome = std::make_shared<DirectedGraph>();

    if (_selectedGenome->load(filename)) {
        _selectedGenome->unfold();

        _previewCreature = std::make_shared<SimCreature>(btVector3(.0, 2.0, .0), _selectedGenome, _previewWorld->getBtWorld());
        _previewCreature->setMaterial(_nodeMaterial);
        _previewCreature->setShader(_nodeShader);
        _previewCreature->addToWorld();

        char label[256];
        sprintf_s(label, "Loaded genome '%s' with a total of% d node(s), % d joint(s), % d end(s), % d brush(es), % d output(s) in %d attempt(s)", filename.c_str(),
            _selectedGenome->getNumNodesUnfolded(),
            _selectedGenome->getNumJointsUnfolded(),
            _selectedGenome->getNumEndNodesUnfolded(),
            _selectedGenome->getNumBrushes(),
            _selectedGenome->getNumJointsUnfolded() + _selectedGenome->getNumBrushes()
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
void SimulationManager::generateRandomGenome()
{
    bool bNoFeasibleCreatureFound = true;
    int attempts = 0;

    ofLog() << "Generating genome...";
    _selectedGenome = std::make_shared<DirectedGraph>(genomeGenMinNumNodes, genomeGenMinNumConns, bAxisAlignedAttachments);
    _selectedGenome->unfold();

    std::shared_ptr<SimCreature> tempCreature;
    while (bNoFeasibleCreatureFound && attempts < _maxGenGenomeAttempts) {

        tempCreature = std::make_shared<SimCreature>(
            btVector3(0, 2.0, 0), _selectedGenome, _previewWorld->getBtWorld()
        );
        bool bSuccess = bFeasibilityChecks ? tempCreature->feasibilityCheck() : true;
        if (bSuccess) {
            bNoFeasibleCreatureFound = false;
            _selectedGenome->print();

            _previewCreature = tempCreature;
            _previewCreature->setMaterial(_nodeMaterial);
            _previewCreature->setShader(_nodeShader);
            _previewCreature->addToWorld();
        }
        else {
            // Replace the managed object
            _selectedGenome = std::make_shared<DirectedGraph>(genomeGenMinNumNodes, genomeGenMinNumConns, bAxisAlignedAttachments);
            _selectedGenome->unfold();
        }
        attempts++;
    }
    if (bFeasibilityChecks) {
        char label[256];
        if (!bNoFeasibleCreatureFound) {
            sprintf_s(label, "Generated genome with a total of %d node(s), %d joint(s), %d end(s), %d brush(es), %d output(s) in %d attempt(s).",
                _selectedGenome->getNumNodesUnfolded(),
                _selectedGenome->getNumJointsUnfolded(),
                _selectedGenome->getNumEndNodesUnfolded(),
                _selectedGenome->getNumBrushes(),
                _selectedGenome->getNumJointsUnfolded() + _selectedGenome->getNumBrushes(), attempts
            );
        }
        else {
            sprintf_s(label, "Failed to generate a feasible genome within %d attempts.", attempts);
        }
        setStatus(label);
    }
}

void SimulationManager::loadShaders()
{
    _terrainShader = std::make_shared<ofShader>();
    _nodeShader = std::make_shared<ofShader>();
    _canvasShader = std::make_shared<ofShader>();
    _canvasUpdateShader = std::make_shared<ofShader>();
    _canvasColorShader = std::make_shared<ofShader>();
    _canvasSubTextureShader = std::make_shared<ofShader>();

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
    _canvasUpdateShader->load("shaders/canvas");
    _canvasColorShader->load("shaders/lum2col");
    _canvasSubTextureShader->load("shaders/subtexture");
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
