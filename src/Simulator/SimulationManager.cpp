#include "Simulator/SimulationManager.h"
#include "Utils/SimUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/VectorUtils.h"
#include "Utils/OFUtils.h"
#include "Genome/DirectedGraph.h"
#include "Simulator/SimDefines.h"
#include "Networking/OscProtocol.h"
#include "ofLog.h"
#include "ofMath.h"

void SimulationManager::init(SimSettings settings)
{
    // canvas
    _canvasResolution = glm::ivec2(settings.canvasResolution, settings.canvasResolution);
    _canvasConvResolution = glm::ivec2(settings.canvasConvResolution, settings.canvasConvResolution);

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
    cam.lookAt(glm::vec3(0));

    // Load terrain material
    _terrainMaterial = std::make_shared<PBRMaterial>(ofFloatColor::white, 0.0625f, 0.125f, 1.0f);
    std::shared_ptr<PBRMaterial> _terrainMtlAccessPtr = std::dynamic_pointer_cast<PBRMaterial>(_terrainMaterial);
    _terrainMtlAccessPtr->setup(
        "materials/wood/WoodFloor024_1K_Color.jpg",
        "materials/wood/WoodFloor024_1K_Normal.jpg",
        "",
        "materials/wood/WoodFloor024_1K_Roughness.jpg",
        "materials/wood/WoodFloor024_1K_AmbientOcclusion.jpg"
    );
    _terrainMtlAccessPtr->setNormalMapMult(0.5f);
    _terrainMtlAccessPtr->setMetallic(0.0f);

    // Load canvas material
    _canvasMaterial = std::make_shared<PBRMaterial>(ofFloatColor::white, 0.125f, 0.75f, 1.0f);
    std::shared_ptr<PBRMaterial> _canvasMtlAccessPtr = std::dynamic_pointer_cast<PBRMaterial>(_canvasMaterial);
    
    _canvasMtlAccessPtr->setup(
        "materials/paper/paper001_1K_Color.png",
        "materials/paper/paper002_1K_Normal.png",
        "",
        "materials/paper/paper002_1K_Roughness.png",
        ""
    );
    _canvasMtlAccessPtr->setNormalMapMult(1.0f);
    _canvasMtlAccessPtr->setMetallic(0.0f);

    // Load node material
    _nodeMaterial = std::make_shared<PBRMaterial>(ofFloatColor::white, 0.125f, 0.5f, 1.0f);
    std::shared_ptr<PBRMaterial> _nodeMtlAccessPtr = std::dynamic_pointer_cast<PBRMaterial>(_nodeMaterial);

    _nodeMtlAccessPtr->setup(
        "",
        "materials/bevel/Bevel_Normal.png",
        "",
        "materials/bevel/Bevel_Roughness.png",
        ""
    );
    _nodeMtlAccessPtr->setNormalMapMult(1.0f);
    _nodeMtlAccessPtr->setMetallic(0.0f);

    // Light
    _light = std::make_shared<ofLight>();

    float lon = ofRandom(-1.0f, 1.0f) * glm::pi<float>();
    float lat = ofMap(ofRandom(1.0f), 0.0f, 1.0f, glm::pi<float>(), glm::two_pi<float>());
    _light->orbitRad(lon, lat, _lightDistanceFromFocus);

    _light->setDirectional();
    _light->setAmbientColor(ofFloatColor(1.0f, 1.0f));
    _light->setDiffuseColor(ofFloatColor(1.0f, 1.0f));
    _light->setSpecularColor(ofFloatColor(1.0f, 1.0f));
    _light->lookAt(glm::vec3(0));

    lightPosition = _light->getPosition();
    //ofColor::fromHex(ofHexToInt());

    //  shadows
    _shadowMap.setup(1024);

    // preview world
    _previewWorld = new SimWorld();
    _previewWorld->getTerrainNode()->setAppearance(_terrainShader, _terrainMaterial);
    _previewWorld->getTerrainNode()->setLight(_light);
    _previewWorld->getTerrainNode()->getRigidBody()->setCollisionFlags(btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);

    // preview canvas
    _previewCanvas = std::make_unique<SimCanvasNode>(btVector3(0, 0, 0), 
        _settings.canvasSize, _settings.canvasViewSize, _settings.canvasMargin,
        _canvasResolution.x, _canvasResolution.y,
        _canvasConvResolution.x, _canvasConvResolution.y,
        _previewWorld->getBtWorld()
    );
    _previewCanvas->setMaterial(_canvasMaterial);
    _previewCanvas->setShader(_canvasShader);
    _previewCanvas->setCanvasUpdateShader(_canvasUpdateShader);
    _previewCanvas->setCanvasColorizeShader(_canvasColorShader);
    _previewCanvas->setSubTextureShader(_canvasSubTextureShader);
    _previewCanvas->addToWorld();

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

    // texture
    if (bStoreLastArtifact) {
        _prevArtifactTexture.allocate(_canvasResolution.x, _canvasResolution.y, GL_RGBA);
        _artifactCopyBuffer.allocate(_canvasResolution.x * _canvasResolution.y * 4, GL_DYNAMIC_COPY);
    }

    // eval
    _evaluationType = settings.evalType;
    _evaluationDispatcher.setup(_evaluationType, _canvasResolution.x, _canvasResolution.y);

    // test
    //cv::Mat testImage = cv::imread("data/keep/circle.bmp", cv::ImreadModes::IMREAD_GRAYSCALE);
    //_evaluationDispatcher.queue(testImage, 0, 0, false);
    //_evaluationDispatcher.queue(cv::imread("data/keep/pollock2_highcontrast.bmp", cv::ImreadModes::IMREAD_GRAYSCALE), 0, 0, false);

    _settings = settings;
    bInitialized = true;
}

void SimulationManager::startSimulation()
{
    if (bInitialized && !bSimulationActive) {
        bSimulationActive = true;

        if (_previewCreature) {
            _previewCreature->removeFromWorld();
            _previewCanvas->removeFromWorld();
        }
        simulationSpeed = 1.0;

        // Reset timing
        _startTime = _clock.getTimeMilliseconds();
        _frameTimeAccumulator = 0.0;
        _clock.reset();

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
            if (!bHasSimulationId) {
                _uniqueSimId = info.ga_id;
                _simDir = NTRS_SIMS_DIR + '/' + _uniqueSimId + '/';
                bHasSimulationId = true;
                ofLog() << ">> Simulation ID: " << _uniqueSimId;
            }
            else if (_uniqueSimId != info.ga_id) {
                ofLog() << "Simulation ID mismatch: " << _uniqueSimId << " / " << info.ga_id;
            }
            queueSimInstance(info);
        });
        _pulseReceivedListener = _networkManager.onPulseReceived.newListener([this](float pulse) {
            _cpgQueue.push(pulse);
        });
        _fitnessRequestReceivedListener = _networkManager.onFitnessRequestReceived.newListener([this] {
            _evaluationDispatcher.queueResponse();
        });
        _fitnessResponseReadyListener = _evaluationDispatcher.onFitnessResponseReady.newListener([this](const std::vector<std::vector<double>>& fitness) {
            int numEntries = fitness.size();
            int numStats = fitness[0].size();
            std::vector<double> flatResults = VectorUtils::flatten(fitness);
            _networkManager.send(OSC_FITNESS + '/' + ofToString(numEntries)  + '/' + ofToString(numStats), flatResults);
        });
        _connectionClosedListener = _networkManager.onConnectionClosed.newListener([this] {
            stopSimulation();
        });
        uint32_t numJoints = _selectedGenome->getNumJointsUnfolded();
        uint32_t numBrushes = 1; // This is 1 & fixed with regard to the local perception method

        _networkManager.allocate(numJoints, numJoints + numBrushes, _canvasConvResolution.x, _canvasConvResolution.y, OF_PIXELS_GRAY);
        _networkManager.search();
       
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
    setStatus("Queued simulation instance. /Generation id: " + ofToString(info.generation) + " /Local id: " + ofToString(info.candidate_id));
    return info.candidate_id;
}

int SimulationManager::createSimInstance(SimInfo info)
{
    int grid_x = info.candidate_id % _simInstanceGridSize;
    int grid_z = info.candidate_id / _simInstanceGridSize;

    btScalar stride = _settings.canvasSize * 2.0 + _settings.canvasMargin * 2.0;
    btScalar xpos = (grid_x - _simInstanceGridSize / 2) * stride + grid_x * _settings.canvasMargin;
    btScalar zpos = (grid_z - _simInstanceGridSize / 2) * stride + grid_z * _settings.canvasMargin;
    btVector3 position = (bMultiEval) ? btVector3(xpos, 0, zpos) : btVector3(0, 0, 0);

    SimWorld* world = new SimWorld();
    world->getTerrainNode()->getRigidBody()->setCollisionFlags(btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);

    SimCreature* crtr = new SimCreature(position, _selectedGenome, world->getBtWorld());
    crtr->setSensorMode(bCanvasSensors ? SimCreature::Canvas : SimCreature::Touch);
    crtr->setMaterial(_nodeMaterial);
    crtr->setShader(_nodeShader);
    crtr->addToWorld();

    SimCanvasNode* canv = new SimCanvasNode(position, _settings.canvasSize, _settings.canvasViewSize, _settings.canvasMargin,
        _canvasResolution.x, _canvasResolution.y,
        _canvasConvResolution.x, _canvasConvResolution.y, 
        world->getBtWorld()
    );
    canv->setMaterial(_canvasMaterial);
    canv->setShader(_canvasShader);
    canv->setCanvasUpdateShader(_canvasUpdateShader);
    canv->setCanvasColorizeShader(_canvasColorShader);
    canv->setSubTextureShader(_canvasSubTextureShader);
    canv->spawnBounds(true);
    canv->addToWorld();

    SimInstance* instance = new SimInstance(info.candidate_id, info.generation, world, crtr, canv, info.duration);
    _networkManager.sendState(instance);
    _simulationInstances.push_back(instance);

    setStatus("Running simulation instance [GEN:" + ofToString(info.generation) + "] [ID:" + ofToString(info.candidate_id) + "]");
    return info.candidate_id;
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
    shader->setUniform1f("lightIntensity", lightIntensity);
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

            _imageSaver.copyToBuffer(instance->getCanvas()->getPaintMap()->getTexture(), [&](uint8_t* p) {
                _artifactMat = cv::Mat(_canvasResolution.x, _canvasResolution.y, CV_8UC1, p);
            });
            _evaluationDispatcher.queue(_artifactMat, instance->getGeneration(), instance->getID());

            if (bSaveArtifactsToDisk) {
                std::string path = _simDir + '/' + NTRS_ARTIFACTS_PREFIX + ofToString(instance->getGeneration()) + '_' + ofToString(instance->getID());
                _imageSaver.save(instance->getCanvas()->getPaintMapRGBA()->getTexture(), path);
            }
            _networkManager.send(OSC_END_ROLLOUT + '/' + ofToString(instance->getID()));

            if (bStoreLastArtifact) {
                instance->getCanvas()->getPaintMapRGBA()->getTexture().copyTo(_artifactCopyBuffer);
                _prevArtifactTexture.loadData(_artifactCopyBuffer, GL_RGBA, GL_UNSIGNED_BYTE);
            }

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
            _previewCanvas->addToWorld();
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

    bool bEffectorsQueued = false;
    if (instance->isEffectorUpdateRequired()) {
        bEffectorsQueued =
            _networkManager.isAgentOutputQueued() &&
            _networkManager.getQueuedAgentId() == instance->getID();

        if (bEffectorsQueued) {
            // update effectors
            instance->getCreature()->updateOutputs(_networkManager.popOutputBuffer());
            instance->updateCreature();
        }
    }
    instance->updateCanvas();

    if (bEffectorsQueued) {
        // send new observation
        _networkManager.sendState(instance);
    }
}

void SimulationManager::shadowPass()
{
    if (!bDebugDraw && bShadows) {
        _shadowMap.begin(*_light, 32.0f, -32.0f, 64.0f);

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

            // TESTCODE
            //glDisable(GL_DEPTH_TEST);
            //ofSetHexColor(0xff0000);
            //for (const auto& instance : _simulationInstances) {
            //    glm::vec3 pos = SimUtils::bulletToGlm(instance->getCreature()->getCenterOfMassPosition());
            //    glm::vec3 dir = -SimUtils::bulletToGlm(btTransform(instance->getCreature()->getRootNodeRotation()) * btVector3(1, 0, 0));
            //    ofDrawArrow(pos, pos + 2.0*dir);
            //}
            //ofSetHexColor(0xffffff);
            //glEnable(GL_DEPTH_TEST);

            glDisable(GL_CULL_FACE);
        }
        else if (_previewCreature) {
            glDisable(GL_DEPTH_TEST);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            _previewCanvas->draw();

            glEnable(GL_DEPTH_TEST);
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
    if (bHasSimulationId) {
        return _uniqueSimId;
    }
    return "NA";
}

const std::string& SimulationManager::getStatus()
{
    return _status;
}

void SimulationManager::setStatus(std::string msg)
{
    _status = msg;
}

const SimulationManager::SimSettings& SimulationManager::getSettings()
{
    return _settings;
}

ofEasyCam* SimulationManager::getCamera()
{
    return &cam;
}

ofTexture* SimulationManager::getPrevArtifactTexture()
{
    return &_prevArtifactTexture;
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
        return _simulationInstances[_focusIndex]->getCanvas();
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

EvaluationType SimulationManager::getEvaluationType()
{
    return _evaluationType;
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
        sprintf_s(label, "Loaded genome '%s' with a total of% d node(s), % d joint(s), % d end(s), % d brush(es), % d output(s)", filename.c_str(),
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
        _terrainShader->load("shaders/checkersPhongShadowPBR");
        _nodeShader->load("shaders/framePhongShadowPBR");
        _canvasShader->load("shaders/texturePhongShadowPBR");
    }
    else {
        _terrainShader->load("shaders/checkers");
        _nodeShader->load("shaders/phong");
        _canvasShader->load("shaders/phong");
    }
    _canvasUpdateShader->load("shaders/canvas_pressure"); 
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
