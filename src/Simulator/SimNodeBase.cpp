#include "Simulator/SimNodeBase.h"
#include "Utils/SimUtils.h"
#include "Utils/OFUtils.h"

SimNodeBase::SimNodeBase(int tag, btDynamicsWorld* owner) : _tag(tag), _ownerWorld(owner) {}
SimNodeBase::SimNodeBase(int tag, ofColor color, btDynamicsWorld* owner) : _tag(tag), _color(color), _ownerWorld(owner) {}

void SimNodeBase::setRigidBody(btRigidBody* body)
{
    removeFromWorld();
    dealloc();

    _body = body;
    _body->setUserIndex(_tag);
    _shape = _body->getCollisionShape();
}

void SimNodeBase::createBody(btVector3 position, btCollisionShape* shape, float mass, void* userPointer)
{
    btTransform trans = btTransform::getIdentity();
    trans.setOrigin(position);

    btDefaultMotionState* motionState = new btDefaultMotionState(trans);

    btScalar bodyMass = mass;
    btVector3 bodyInertia;
    shape->calculateLocalInertia(bodyMass, bodyInertia);

    btRigidBody::btRigidBodyConstructionInfo bodyConstrInfo =
        btRigidBody::btRigidBodyConstructionInfo(bodyMass, motionState, shape, bodyInertia);
    bodyConstrInfo.m_restitution = 0.5f;
    bodyConstrInfo.m_friction = 0.5f;

    btRigidBody* body = new btRigidBody(bodyConstrInfo);
    body->setUserPointer(userPointer);

    setRigidBody(body);
}

void SimNodeBase::setTransform(btTransform transform)
{
    if (_body) {
        _body->setWorldTransform(transform);
    }
}

btTransform SimNodeBase::getTransform()
{
    if (_body) {
        return _body->getWorldTransform();
    }
    else return btTransform::getIdentity();
}

void SimNodeBase::setPosition(btVector3 position)
{
    if (_body) {
        btTransform trans = _body->getWorldTransform();
        trans.setOrigin(position);
        _body->setWorldTransform(trans);
    }
}

btVector3 SimNodeBase::getPosition()
{
    if (_body) {
        return _body->getWorldTransform().getOrigin();
    }
    else return btVector3(.0, .0, .0);
}

void SimNodeBase::setRotation(btQuaternion rotation)
{
    if (_body) {
        _body->getWorldTransform().setRotation(rotation);
    }
}

btQuaternion SimNodeBase::getRotation()
{
    if (_body) {
        return _body->getWorldTransform().getRotation();
    }
    else return btQuaternion::getIdentity();
}

// rigidbody
btRigidBody* SimNodeBase::getRigidBody() { return _body; }
bool SimNodeBase::hasBody() { return _body != 0; }

// shape
btCollisionShape* SimNodeBase::getShape() { return _shape; }

// meta
void SimNodeBase::setTag(uint32_t tag) { 
    _tag = tag;
    _body->setUserIndex(_tag);
}
uint32_t SimNodeBase::getTag() { return _tag; }

void SimNodeBase::addToWorld() { 
    if (_body && !_body->isInWorld()) {
        _ownerWorld->addRigidBody(_body);
    }
}

void SimNodeBase::removeFromWorld() { 
    if (_body && _body->isInWorld()) {
        _ownerWorld->removeRigidBody(_body);
    }
}

// shader
void SimNodeBase::setShader(std::shared_ptr<ofShader> shader) { _shader = shader; }
std::shared_ptr<ofShader> SimNodeBase::getShader() { return _shader; }

void SimNodeBase::setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<MaterialBase> mtl)
{
    _shader = shader;
    _material = mtl;
}
void SimNodeBase::setColor(ofColor c) {
    _color = c;
}
void SimNodeBase::setTexture(std::shared_ptr<ofTexture> texture) {
    _texture = texture;
    bUseTexture = true;
}
void SimNodeBase::setMaterial(std::shared_ptr<MaterialBase> mtl) { _material = mtl; }
void SimNodeBase::setLight(std::shared_ptr<ofLight> light) { _light = light; }
void SimNodeBase::setMesh(std::shared_ptr<ofMesh> mesh) { _mesh = mesh; }

void SimNodeBase::dealloc() {
    if (_body) {
        delete _body->getMotionState();
        delete _body;
    }
    if (_shape) {
        delete _shape;
    }
}

SimNodeBase::~SimNodeBase()
{
    removeFromWorld();
    dealloc();
}
