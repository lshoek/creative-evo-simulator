#include "SimCanvasNode.h"
#include "SimUtils.h"
#include "toolbox.h"

// Maximum number of contactpoints registered and applied to canvas per frame.
// This number should be synced with BRUSH_COORD_BUF_MAXSIZE in canvas.frag.
#define BRUSH_COORD_BUF_MAXSIZE 8

// TODO: Refactoring/inheritance
SimCanvasNode::SimCanvasNode(int tag, float size, int x_res, int y_res, btDynamicsWorld* ownerWorld) :
    _tag(tag), _canvasSize(size), _ownerWorld(ownerWorld)
{
    _canvasRes = glm::ivec2(x_res, y_res);
    _canvasDrawQuad = tb::rectMesh(0, 0, _canvasRes.x, _canvasRes.y, true);

    _brushColor = ofColor::black;
    _canvasClearColor = ofColor::white;

    initPlane(glm::vec3(0), _canvasSize, 0);

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

void SimCanvasNode::initPlane(glm::vec3 position, float size, float mass)
{
    _shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    _mesh = std::make_shared<ofMesh>(tb::gridMesh(2, 2, size * 2, true));
    createBody(position, mass);
}

void SimCanvasNode::createBody(glm::vec3 position, float mass)
{
    btTransform trans = btTransform::getIdentity();
    trans.setOrigin(SimUtils::glmToBullet(position));

    btDefaultMotionState* motionState = new btDefaultMotionState(trans);

    btScalar bodyMass = mass;
    btVector3 bodyInertia;
    _shape->calculateLocalInertia(bodyMass, bodyInertia);

    btRigidBody::btRigidBodyConstructionInfo bodyConstrInfo =
        btRigidBody::btRigidBodyConstructionInfo(bodyMass, motionState, _shape, bodyInertia);
    bodyConstrInfo.m_restitution = 0.5f;
    bodyConstrInfo.m_friction = 0.5f;

    _body = new btRigidBody(bodyConstrInfo);
    _body->setUserPointer(this);
    _body->setUserIndex(_tag);
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
            _shader->setUniformTexture("tex", _canvasFbo[iFbo].getTexture(), 0);
        }
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

void SimCanvasNode::addToWorld()
{
    _ownerWorld->addRigidBody(_body);
}

void SimCanvasNode::removeFromWorld()
{
    _ownerWorld->removeRigidBody(_body);
}

void SimCanvasNode::setRigidBody(btRigidBody* body)
{
    _body = body;
    _body->setUserIndex(_tag);
    _shape = _body->getCollisionShape();
}

void SimCanvasNode::setTransform(glm::mat4 transform)
{
    if (_body) {
        _body->setWorldTransform(SimUtils::glmToBullet(transform));
    }
}

glm::mat4 SimCanvasNode::getTransform()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform());
    }
    else return glm::identity<glm::mat4>();
}

void SimCanvasNode::setPosition(glm::vec3 position)
{
    if (_body) {
        btTransform trans = _body->getWorldTransform();
        trans.setOrigin(SimUtils::glmToBullet(position));
        _body->setWorldTransform(trans);
    }
}

glm::vec3 SimCanvasNode::getPosition()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform().getOrigin());
    }
    else return glm::vec3();
}

void SimCanvasNode::setRotation(glm::quat rotation)
{
    if (_body) {
        _body->getWorldTransform().setRotation(SimUtils::glmToBullet(rotation));
    }
}

glm::quat SimCanvasNode::getRotation()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform().getRotation());
    }
    else return glm::identity<glm::quat>();
}

// rigidbody
btRigidBody* SimCanvasNode::getRigidBody() { return _body; }
bool SimCanvasNode::hasBody() { return _body != nullptr; }

// shape
btCollisionShape* SimCanvasNode::getShape() { return _shape; }

// meta
std::string SimCanvasNode::getName() { return _name; }
int SimCanvasNode::getTag() { return _tag; }

// shader
void SimCanvasNode::setCanvasUpdateShader(std::shared_ptr<ofShader> shader) { _canvasUpdateShader = shader; }
void SimCanvasNode::setShader(std::shared_ptr<ofShader> shader) { _shader = shader; }
std::shared_ptr<ofShader> SimCanvasNode::getShader() { return _shader; }

void SimCanvasNode::setTexture(std::shared_ptr<ofTexture> texture) {
    _texture = texture;
    bUseTexture = true;
}
void SimCanvasNode::setMaterial(std::shared_ptr<ofMaterial> mtl) { _material = mtl; }
void SimCanvasNode::setMesh(std::shared_ptr<ofMesh> mesh) { _mesh = mesh; }

SimCanvasNode::~SimCanvasNode()
{
    _ownerWorld->removeRigidBody(_body);

    delete _body->getMotionState();
    delete _body;
    delete _shape;
}
