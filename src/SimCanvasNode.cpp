#include "SimCanvasNode.h"
#include "SimDefines.h"
#include "SimUtils.h"
#include "toolbox.h"

// Maximum number of contactpoints registered and applied to canvas per frame.
// This number should be synced with BRUSH_COORD_BUF_MAXSIZE in canvas.frag.
#define BRUSH_COORD_BUF_MAXSIZE 8

SimCanvasNode::SimCanvasNode(btVector3 position, float size, float extraBounds, int xRes, int yRes, int xNeuralInput, int yNeuralInput, btDynamicsWorld* ownerWorld) :
    SimNodeBase(CanvasTag, ownerWorld), _canvasSize(size), _margin(extraBounds)
{
    _color = ofColor::white;
    _brushColor = ofColor::black;

    _canvasRes = glm::ivec2(xRes, yRes);
    _canvasNeuralInputRes = glm::vec2(xNeuralInput, yNeuralInput);
    _canvasDrawQuad = tb::rectMesh(0, 0, _canvasRes.x, _canvasRes.y, true);
    _canvasLowResDrawQuad = tb::rectMesh(0, 0, _canvasNeuralInputRes.x, _canvasNeuralInputRes.y, true);

    initPlane(position, _canvasSize);
    
    // Single-channel canvas 'height map'
    for (int i = 0; i < 2; i++) {
        _canvasFbo[i].allocate(_canvasRes.x, _canvasRes.y, GL_R8);
    }
    // Reduced resolution neural input buffer
    ofFboSettings fboSettings;
    fboSettings.width = _canvasNeuralInputRes.x;
    fboSettings.height = _canvasNeuralInputRes.y;
    fboSettings.internalformat = GL_R8;
    fboSettings.minFilter = GL_NEAREST;
    fboSettings.maxFilter = GL_NEAREST;
    _canvasNeuralInputFbo.allocate(fboSettings);

    // Canvas color and alpha separated
    _canvasColorFbo.allocate(_canvasRes.x, _canvasRes.y, GL_RGBA);

    _canvasColorFbo.begin();
    ofClear(_color);
    _canvasColorFbo.end();

    _brushCoordQueue.resize(BRUSH_COORD_BUF_MAXSIZE);
    for (int i = 0; i < BRUSH_COORD_BUF_MAXSIZE; i++) {
        _brushCoordQueue[i].coord = glm::vec2(0);
        _brushCoordQueue[i].pressure = -1.0f;
        _brushCoordQueue[i].active = 0.0f;
    }
    _brushCoordBuffer.allocate();
    _brushCoordBuffer.setData(BrushCoord::size()*BRUSH_COORD_BUF_MAXSIZE, NULL, GL_DYNAMIC_DRAW);

    // Neural input
    for (int i = 0; i < 2; i++) {
        _pixelWriteBuffers[i].allocate(_canvasRes.x * _canvasRes.y, GL_DYNAMIC_READ);
    }
    iPbo = 0;
    _pboPtr = &_pixelWriteBuffers[iPbo];
}

void SimCanvasNode::initPlane(btVector3 position, float size)
{
    _mesh = std::make_shared<ofMesh>(tb::gridMesh(2, 2, size * 2, true));

    btCollisionShape* shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    createBody(position, shape, 0, this);
}

void SimCanvasNode::update()
{
    if (_canvasUpdateShader && _brushQueueSize > 0)
    {
        // bind coord buffers
        _brushCoordBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
        _brushCoordBuffer.updateData(0, _brushCoordQueue);

        // update shader
        iFbo = SWAP(iFbo);

        glEnable(GL_BLEND);
        glBlendEquation(GL_MAX);
        glBlendFunc(GL_ONE, GL_ONE);
        _canvasFbo[iFbo].begin();

        _canvasFbo[SWAP(iFbo)].draw(0, 0);

        _canvasUpdateShader->begin();
        _canvasUpdateShader->setUniform1i("brush_coords_bufsize", _brushQueueSize);

        _canvasDrawQuad.draw();
        _canvasUpdateShader->end();
        _canvasFbo[iFbo].end();

        glBlendEquation(GL_FUNC_ADD);
        glDisable(GL_BLEND);

        _brushCoordBuffer.unbindBase(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // This can be skipped in headless mode
    if (_canvasColorizeShader) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        _canvasColorFbo.begin();
        _canvasColorizeShader->begin();
        _canvasColorizeShader->setUniformTexture("tex", _canvasFbo[iFbo].getTexture(), 0);
        _canvasColorizeShader->setUniform4f("brush_color", _brushColor);
        _canvasDrawQuad.draw();
        _canvasColorizeShader->end();

        _canvasColorFbo.end();
        glDisable(GL_BLEND);
    }

    // invalidate values
    for (int i = 0; i < BRUSH_COORD_BUF_MAXSIZE; i++) {
        _brushCoordQueue[i].pressure = -1.0f;
        _brushCoordQueue[i].active = 0.0f;
    }
    _brushQueueSize = 0;
}

void SimCanvasNode::updateNeuralInputBuffer()
{
    // Reduce height map resolution for neural inputs
    _canvasNeuralInputFbo.begin();
    _canvasFbo[iFbo].draw(0, 0, _canvasNeuralInputRes.x, _canvasNeuralInputRes.y);
    _canvasNeuralInputFbo.end();

    _canvasFbo[iFbo].copyTo(_pixelWriteBuffers[iPbo]);
    _pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);

    unsigned char* bytesPtr = _pboPtr->map<unsigned char>(GL_READ_ONLY);
    _neuralInputMat = cv::Mat(_canvasNeuralInputRes.x, _canvasNeuralInputRes.y, CV_8UC1, bytesPtr);

    _pboPtr->unmap();
    _pboPtr->unbind(GL_PIXEL_UNPACK_BUFFER);

    iPbo = SWAP(iPbo);
}

void SimCanvasNode::draw()
{
    if (_shader) {
        ofPushMatrix();
        ofMultMatrix(SimUtils::bulletToGlm(getTransform()));

        _shader->begin();
        if (bUseTexture) {
            //_shader->setUniformTexture("tex", _canvasFbo[iFbo].getTexture(), 0);
            _shader->setUniformTexture("tex", _canvasColorFbo.getTexture(), 0);
        }
        _shader->setUniform4f("color", ofColor::white);
        _shader->setUniform4f("mtl.ambient", _material->getAmbientColor());
        _shader->setUniform4f("mtl.diffuse", _material->getDiffuseColor());
        _shader->setUniform4f("mtl.specular", _material->getSpecularColor());
        _shader->setUniform4f("mtl.emission", _material->getEmissiveColor());
        _shader->setUniform1f("mtl.shininess", _material->getShininess());

        _mesh->draw();
        _shader->end();

        ofPopMatrix();
    }
}

void SimCanvasNode::drawImmediate()
{
    ofPushMatrix();
    ofMultMatrix(SimUtils::bulletToGlm(getTransform()));
    _mesh->draw();
    ofPopMatrix();
}

void SimCanvasNode::addBrushStroke(btVector3 location, float pressure)
{
    if (_brushQueueSize < BRUSH_COORD_BUF_MAXSIZE)
    {
        glm::vec3 loc = SimUtils::bulletToGlm(location);

        // convert to normalized texture coordinates
        glm::vec2 px = (glm::vec2(loc.x, loc.z) + glm::vec2(_canvasSize)) / glm::vec2(_canvasSize * 2);
        if (px.x > 1.0f || px.x < 0 || px.y > 1.0f || px.y < 0) {
            return;
        }

        // add brushstroke
        _brushCoordQueue[_brushQueueSize].coord = px;
        _brushCoordQueue[_brushQueueSize].pressure = pressure;
        _brushQueueSize++;
    }
}

unsigned char* SimCanvasNode::getNeuralInputsBuffer() const
{
    return _neuralInputMat.data;
}

ofFbo* SimCanvasNode::getCanvasNeuralInputRawFbo()
{
    return &_canvasNeuralInputFbo;
}

ofFbo* SimCanvasNode::getCanvasRawFbo()
{
    return &_canvasFbo[iFbo];
}

ofFbo* SimCanvasNode::getCanvasFbo()
{
    return &_canvasColorFbo;
}

glm::ivec2 SimCanvasNode::getCanvasResolution()
{
    return _canvasRes;
}

void SimCanvasNode::setCanvasUpdateShader(std::shared_ptr<ofShader> shader) {
    _canvasUpdateShader = shader; 
}

void SimCanvasNode::setCanvasColorizeShader(std::shared_ptr<ofShader> shader) {
    _canvasColorizeShader = shader;
}

void SimCanvasNode::enableBounds()
{
    if (_bBounds) {
        return;
    }
    else {
        float thickness = 1.0f;
        glm::vec3 fromCenter = glm::vec3(0, 0, _canvasSize + _margin + thickness);
        glm::vec3 ax = glm::vec3(0, 0, 1);

        for (int i = 0; i < 4; i++) {
            _bounds[i] = std::make_unique<SimNode>(BoundsTag, _ownerWorld);
            _bounds[i]->initBox(btVector3(0, 0, 0), btVector3(_canvasSize+_margin, _canvasSize, thickness), 0);

            glm::quat rot = glm::rotation(ax, glm::rotate(ax, float(HALF_PI) * i, glm::vec3(0, 1, 0)));
            _bounds[i]->setRotation(SimUtils::glmToBullet(rot));
            _bounds[i]->setPosition((getPosition() + SimUtils::glmToBullet(glm::vec3(0, _canvasSize, 0) + rot * fromCenter)));
            _bounds[i]->bRender = false;
        }
        _bBounds = true;
    }
}

void SimCanvasNode::addToWorld()
{
    SimNodeBase::addToWorld();

    if (_bBounds) {
        for (int i = 0; i < 4; i++) {
            _bounds[i]->addToWorld();
        }
    }
}

void SimCanvasNode::removeFromWorld()
{
    SimNodeBase::removeFromWorld();

    if (_bBounds) {
        for (int i = 0; i < 4; i++) {
            _bounds[i]->removeFromWorld();
        }
    }
}

SimCanvasNode::~SimCanvasNode()
{

}
