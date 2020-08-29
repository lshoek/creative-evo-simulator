#include "Simulator/SimCanvasNode.h"
#include "Simulator/SimDefines.h"
#include "Utils/SimUtils.h"
#include "Utils/MeshUtils.h"
#include "Utils/OFUtils.h"
#include "ofMaterial.h"

// Maximum number of contactpoints registered and applied to canvas per frame.
// This number should be synced with BRUSH_COORD_BUF_MAXSIZE in canvas.frag.
#define BRUSH_COORD_BUF_MAXSIZE 16

SimCanvasNode::SimCanvasNode(btVector3 position, float size, float viewSize, float boundsMargin, int xRes, int yRes, int xConvRes, int yConvRes, btDynamicsWorld* ownerWorld) :
    SimNodeBase(CanvasTag, ownerWorld), _canvasSize(size), _patchSize(viewSize), _margin(boundsMargin), _areaSize(size + boundsMargin)
{
    _color = ofColor::white;
    _brushColor = ofColor::black;

    _canvasRes = glm::ivec2(xRes, yRes);
    _canvasConvRes = glm::vec2(xConvRes, yConvRes);

    _drawQuad = MeshUtils::rectMesh(0, 0, _canvasRes.x, _canvasRes.y, true);
    _drawQuadConv = MeshUtils::rectMesh(0, 0, _canvasConvRes.x, _canvasConvRes.y, true);

    initPlane(position, _canvasSize);
    
    // Single-channel canvas 'height map'
    for (int i = 0; i < 2; i++) {
        _fbo[i].allocate(_canvasRes.x, _canvasRes.y, GL_R8);
        _fbo[i].getTexture().setTextureWrap(GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER);

        _fbo[i].getTexture().bind();
        ofFloatColor color(0.0f, 0.0f, 0.0f, 0.0f);
        glTexParameterfv(GL_TEXTURE_RECTANGLE, GL_TEXTURE_BORDER_COLOR, &color.r);
        _fbo[i].getTexture().unbind();
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
        _brushCoordQueue[i].reset();
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

    getRigidBody()->setCollisionFlags(btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
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
        iFbo = OFUtils::swap(iFbo);

        glEnable(GL_BLEND);
        glBlendEquation(GL_MAX);
        glBlendFunc(GL_ONE, GL_ONE);
        _fbo[iFbo].begin();

        _fbo[OFUtils::swap(iFbo)].draw(0, 0);

        _updateShader->begin();
        _updateShader->setUniform1f("use_brush_pressure", _bVariableBrushPressure);
        _updateShader->setUniform1i("brush_coords_bufsize", _brushQueueSize);

        _drawQuad.draw();
        _updateShader->end();
        _fbo[iFbo].end();

        glBlendEquation(GL_FUNC_ADD);
        glDisable(GL_BLEND);

        _brushCoordBuffer.unbindBase(GL_SHADER_STORAGE_BUFFER, 0);

        // copy the last brush coord to local vision cache
        if (_cachedBrushCoord.coord != _brushCoordQueue[_brushQueueSize - 1].coord) {
            _cachedBrushCoord = _brushCoordQueue[_brushQueueSize - 1];
        }

        btScalar z, y, x;
        _cachedLocalVisionRotation.getEulerZYX(z, y, x);

        float theta = -y;
        _localVisionRotationMatrix = glm::vec4(
            cos(theta), -sin(theta),
            sin(theta), cos(theta)
        );
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
        _brushCoordQueue[i].reset();
    }
    _brushQueueSize = 0;
}

void SimCanvasNode::updateConvPixelBuffer()
{
    // Sample local patch from full-size canvas fbo
    _convFbo.begin();
    _subTextureShader->begin();
    _subTextureShader->setUniformTexture("tex", _fbo[iFbo].getTexture(), 0);
    _subTextureShader->setUniform2f("patchLocation", _cachedBrushCoord.coord);
    _subTextureShader->setUniform1f("patchSize", _patchSize);
    _subTextureShader->setUniform4f("patchRotationMatrix", _localVisionRotationMatrix);
    _drawQuadConv.draw();
    _subTextureShader->end();
    _convFbo.end();

    _convFbo.copyTo(*_pboPtr);
    _pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);

    ofBufferObject* backBufPtr = &_pixelWriteBuffers[(iPbo + 1) % 2];
    unsigned char* bytesPtr = backBufPtr->map<unsigned char>(GL_READ_ONLY);

    _convPixelBuffer.setFromExternalPixels(bytesPtr, _canvasConvRes.x, _canvasConvRes.y, 1);

    backBufPtr->unmap();
    backBufPtr->unbind(GL_PIXEL_UNPACK_BUFFER);

    swapPbo();
}

void SimCanvasNode::clearConvPixelBuffer()
{
    _convFbo.clearColorBuffer(ofFloatColor(0,0,0,0));
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

void SimCanvasNode::addBrushStroke(btVector3 location, float pressure, bool active)
{
    if (_brushQueueSize < BRUSH_COORD_BUF_MAXSIZE)
    {
        glm::vec3 loc = SimUtils::bulletToGlm(location);

        // convert to normalized texture coordinates
        glm::vec2 px = (glm::vec2(loc.x, loc.z) + glm::vec2(_canvasSize)) / glm::vec2(_canvasSize * 2);
        
        // add brushstroke
        _brushCoordQueue[_brushQueueSize].coord = px;
        _brushCoordQueue[_brushQueueSize].pressure = glm::clamp(pressure, 0.0f, 1.0f);
        _brushCoordQueue[_brushQueueSize].active = active;
        _brushQueueSize++;
    }
}

void SimCanvasNode::setLocalVisionRotation(btQuaternion rotationMatrix)
{
    _cachedLocalVisionRotation = rotationMatrix;
}

void SimCanvasNode::swapPbo()
{
    iPbo = (iPbo + 1) % 2;
    _pboPtr = &_pixelWriteBuffers[iPbo];
}

glm::ivec2 SimCanvasNode::getCanvasResolution()
{
    return _canvasRes;
}

const ofFbo* SimCanvasNode::getViewMap() const
{
    return &_convFbo;
}

const ofFbo* SimCanvasNode::getPaintMap() const
{
    return &_fbo[iFbo];
}

const ofFbo* SimCanvasNode::getPaintMapRGBA() const
{
    return &_colorFbo;
}

const ofPixels& SimCanvasNode::getConvPixelBuffer()
{
    return _convPixelBuffer;
}

void SimCanvasNode::setCanvasUpdateShader(std::shared_ptr<ofShader> shader) {
    _updateShader = shader; 
}

void SimCanvasNode::setCanvasColorizeShader(std::shared_ptr<ofShader> shader) {
    _colorizeShader = shader;
}

void SimCanvasNode::setSubTextureShader(std::shared_ptr<ofShader> shader) {
    _subTextureShader = shader;
}

void SimCanvasNode::spawnBounds(bool bDebugRender)
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

            if (!bDebugRender) {
                _bounds[i]->getRigidBody()->setCollisionFlags(btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
            }
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
