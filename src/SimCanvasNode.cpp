#include "SimCanvasNode.h"
#include "SimUtils.h"
#include "toolbox.h"

// Maximum number of contactpoints registered and applied to canvas per frame.
// This number should be synced with BRUSH_COORD_BUF_MAXSIZE in canvas.frag.
#define BRUSH_COORD_BUF_MAXSIZE 8

SimCanvasNode::SimCanvasNode(int tag, float size, int x_res, int y_res, btDynamicsWorld* owner) : 
    SimNodeBase(tag, owner), _canvasSize(size)
{
    _color = ofColor::white;
    _brushColor = ofColor::black;
    _canvasClearColor = ofColor::white;

    _canvasRes = glm::ivec2(x_res, y_res);
    _canvasDrawQuad = tb::rectMesh(0, 0, _canvasRes.x, _canvasRes.y, true);

    initPlane(glm::vec3(0), _canvasSize);

    for (int i = 0; i < 2; i++) {
        _canvasFbo[i].allocate(_canvasRes.x, _canvasRes.y, GL_RGBA32F);
    }
    _canvasFinalFbo.allocate(_canvasRes.x, _canvasRes.y, GL_RGBA);

    _brushCoordQueue.resize(BRUSH_COORD_BUF_MAXSIZE);
    for (int i = 0; i < BRUSH_COORD_BUF_MAXSIZE; i++) {
        _brushCoordQueue[i].coord = glm::vec2(0);
        _brushCoordQueue[i].impulse = -1.0f;
        _brushCoordQueue[i].active = 0.0f;
    }
    _brushCoordBuffer.allocate();
    _brushCoordBuffer.setData(BrushCoord::size()*BRUSH_COORD_BUF_MAXSIZE, NULL, GL_DYNAMIC_DRAW);
}

void SimCanvasNode::initPlane(glm::vec3 position, float size)
{
    _shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    _mesh = std::make_shared<ofMesh>(tb::gridMesh(2, 2, size * 2, true));
    createBody(position, 0);
}

void SimCanvasNode::update()
{
    if (_brushQueueSize > 0)
    {
        // bind coord buffers
        _brushCoordBuffer.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
        _brushCoordBuffer.updateData(0, _brushCoordQueue);

        // update shader
        iFbo = SWAP(iFbo);

        glEnable(GL_BLEND);
        glBlendEquationSeparate(GL_FUNC_ADD, GL_MAX);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
        _canvasFbo[iFbo].begin();

        _canvasFbo[SWAP(iFbo)].draw(0, 0);

        _canvasUpdateShader->begin();
        _canvasUpdateShader->setUniform1i("brush_coords_bufsize", _brushQueueSize);
        _canvasUpdateShader->setUniform4f("color", _brushColor);

        _canvasDrawQuad.draw();
        _canvasUpdateShader->end();
        _canvasFbo[iFbo].end();

        glBlendEquation(GL_FUNC_ADD);
        glDisable(GL_BLEND);

        _brushCoordBuffer.unbindBase(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // invalidate values
    for (int i = 0; i < BRUSH_COORD_BUF_MAXSIZE; i++) {
        _brushCoordQueue[i].impulse = -1.0f;
        _brushCoordQueue[i].active = 0.0f;
    }
    _brushQueueSize = 0;

    // draw to final canvas
    _canvasFinalFbo.begin();
    ofBackground(_canvasClearColor);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    _canvasFbo[iFbo].draw(0, 0);
    glDisable(GL_BLEND);

    _canvasFinalFbo.end();
}

void SimCanvasNode::draw()
{
    if (_shader) {
        ofPushMatrix();
        ofMultMatrix(getTransform());

        _shader->begin();
        if (bUseTexture) {
            //_shader->setUniformTexture("tex", _canvasFbo[iFbo].getTexture(), 0);
            _shader->setUniformTexture("tex", _canvasFinalFbo.getTexture(), 0);
        }
        _shader->setUniform4f("color", _color);
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

void SimCanvasNode::addBrushStroke(btVector3 location, float impulse)
{
    if (_brushQueueSize < BRUSH_COORD_BUF_MAXSIZE)
    {
        // convert to normalized texture coordinates
        glm::vec3 loc = SimUtils::bulletToGlm(location);
        glm::vec2 px = (glm::vec2(loc.x, loc.z) + glm::vec2(_canvasSize)) / glm::vec2(_canvasSize * 2);

        if (px.x > 1.0f || px.x < 0 || px.y > 1.0f || px.y < 0) {
            return;
        }

        // add brushstroke
        _brushCoordQueue[_brushQueueSize].coord = px;
        _brushCoordQueue[_brushQueueSize].impulse = impulse;
        _brushQueueSize++;
    }
}

glm::ivec2 SimCanvasNode::getCanvasResolution()
{
    return _canvasRes;
}

ofFbo* SimCanvasNode::getCanvasFbo()
{
    return &_canvasFinalFbo;
}

void SimCanvasNode::setCanvasUpdateShader(std::shared_ptr<ofShader> shader) { 
    _canvasUpdateShader = shader; 
}

SimCanvasNode::~SimCanvasNode()
{
    removeFromWorld();
}
