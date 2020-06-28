#include "Simulator/SimCanvasNode.h"
#include "Simulator/SimDefines.h"
#include "Utils/SimUtils.h"
#include "Utils/MeshUtils.h"
#include "Utils/toolbox.h"

// Maximum number of contactpoints registered and applied to canvas per frame.
// This number should be synced with BRUSH_COORD_BUF_MAXSIZE in canvas.frag.
#define BRUSH_COORD_BUF_MAXSIZE 8

SimCanvasNode::SimCanvasNode(btVector3 position, float size, float extraBounds, int xRes, int yRes, int xConvRes, int yConvRes, bool bDownSample, btDynamicsWorld* ownerWorld) :
    SimNodeBase(CanvasTag, ownerWorld), _canvasSize(size), _margin(extraBounds)
{
    _color = ofColor::white;
    _brushColor = ofColor::black;

    _bDownSample = bDownSample;
    _canvasRes = glm::ivec2(xRes, yRes);
    _canvasConvRes = _bDownSample ? glm::vec2(xConvRes, yConvRes) : _canvasRes;
    _drawQuad = MeshUtils::rectMesh(0, 0, _canvasRes.x, _canvasRes.y, true);
    _drawQuadConv = MeshUtils::rectMesh(0, 0, _canvasConvRes.x, _canvasConvRes.y, true);

    initPlane(position, _canvasSize);
    
    // Single-channel canvas 'height map'
    for (int i = 0; i < 2; i++) {
        _fbo[i].allocate(_canvasRes.x, _canvasRes.y, GL_R8);
    }
    // Reduced resolution neural input buffer
    ofFboSettings fboSettings;
    fboSettings.width = _canvasConvRes.x;
    fboSettings.height = _canvasConvRes.y;
    fboSettings.internalformat = GL_R8;
    fboSettings.minFilter = GL_NEAREST;
    fboSettings.maxFilter = GL_NEAREST;
    _convFbo.allocate(fboSettings);

    // Canvas color and alpha separated
    _colorFbo.allocate(_canvasRes.x, _canvasRes.y, GL_RGBA);

    _colorFbo.begin();
    ofClear(_color.r, _color.b, _color.g, 0.0f);
    _colorFbo.end();

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
    _convPixelBuffer.allocate(_canvasConvRes.x, _canvasConvRes.y, 1);

    iPbo = 0;
    _pboPtr = &_pixelWriteBuffers[iPbo];
}

void SimCanvasNode::initPlane(btVector3 position, float size)
{
    _mesh = std::make_shared<ofMesh>(MeshUtils::gridMesh(2, 2, size*2, true));

    btCollisionShape* shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    createBody(position, shape, 0, this);
}

void SimCanvasNode::update()
{
    if (_updateShader && _brushQueueSize > 0)
    {
        // bind coord buffers
        _brushCoordBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
        _brushCoordBuffer.updateData(0, _brushCoordQueue);

        // update shader
        iFbo = tb::swap(iFbo);

        glEnable(GL_BLEND);
        glBlendEquation(GL_MAX);
        glBlendFunc(GL_ONE, GL_ONE);
        _fbo[iFbo].begin();

        _fbo[tb::swap(iFbo)].draw(0, 0);

        _updateShader->begin();
        _updateShader->setUniform1i("brush_coords_bufsize", _brushQueueSize);

        _drawQuad.draw();
        _updateShader->end();
        _fbo[iFbo].end();

        glBlendEquation(GL_FUNC_ADD);
        glDisable(GL_BLEND);

        _brushCoordBuffer.unbindBase(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // This is just for visualization so completely optional and can be skipped in headless mode
    if (_colorizeShader) {
        _colorFbo.begin();

        _colorizeShader->begin();
        _colorizeShader->setUniformTexture("tex", _fbo[iFbo].getTexture(), 0);
        _colorizeShader->setUniform4f("brush_color", _brushColor);
        _colorizeShader->setUniform4f("canvas_color", _color);
        _drawQuad.draw();
        _colorizeShader->end();

        _colorFbo.end();
    }

    // invalidate values
    for (int i = 0; i < BRUSH_COORD_BUF_MAXSIZE; i++) {
        _brushCoordQueue[i].pressure = -1.0f;
        _brushCoordQueue[i].active = 0.0f;
    }
    _brushQueueSize = 0;
}

void SimCanvasNode::updateConvPixelBuffer()
{
    // Reduce height map resolution for neural inputs
    if (_bDownSample) {
        _convFbo.begin();
        _fbo[iFbo].draw(0, 0, _canvasConvRes.x, _canvasConvRes.y);
        _convFbo.end();
    }
    ofFbo* fboPtr = _bDownSample ? &_convFbo : &_fbo[iFbo];

    fboPtr->copyTo(*_pboPtr);
    _pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);

    ofBufferObject* backBufPtr = &_pixelWriteBuffers[(iPbo + 1) % 2];
    unsigned char* bytesPtr = backBufPtr->map<unsigned char>(GL_READ_ONLY);

    _convPixelBuffer.setFromExternalPixels(bytesPtr, _canvasConvRes.x, _canvasConvRes.y, 1);

    backBufPtr->unmap();
    backBufPtr->unbind(GL_PIXEL_UNPACK_BUFFER);

    swapPbo();
}

void SimCanvasNode::draw()
{
    if (_shader) {
        ofPushMatrix();
        ofMultMatrix(SimUtils::bulletToGlm(getTransform()));

        _shader->begin();
        _shader->setUniformTexture("tex", _colorFbo.getTexture(), 0);
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

void SimCanvasNode::swapPbo()
{
    iPbo = (iPbo + 1) % 2;
    _pboPtr = &_pixelWriteBuffers[iPbo];
}

const ofPixels& SimCanvasNode::getConvPixelBuffer()
{
    return _convPixelBuffer;
}

ofFbo* SimCanvasNode::getConvFbo()
{
    return &_convFbo;
}

ofFbo* SimCanvasNode::getCanvasRawFbo()
{
    return &_fbo[iFbo];
}

ofFbo* SimCanvasNode::getCanvasFbo()
{
    return &_colorFbo;
}

glm::ivec2 SimCanvasNode::getCanvasResolution()
{
    return _canvasRes;
}

void SimCanvasNode::setCanvasUpdateShader(std::shared_ptr<ofShader> shader) {
    _updateShader = shader; 
}

void SimCanvasNode::setCanvasColorizeShader(std::shared_ptr<ofShader> shader) {
    _colorizeShader = shader;
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