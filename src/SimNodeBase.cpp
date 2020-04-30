#include "SimNodeBase.h"
#include "SimUtils.h"
#include "toolbox.h"

SimNodeBase::SimNodeBase(int tag, btDynamicsWorld* owner) : _tag(tag), _ownerWorld(owner) {}
SimNodeBase::SimNodeBase(int tag, ofColor color, btDynamicsWorld* owner) : _tag(tag), _color(color), _ownerWorld(owner) {}

void SimNodeBase::setRigidBody(btRigidBody* body)
{
    _body = body;
    _body->setUserIndex(_tag);
    _shape = _body->getCollisionShape();
}

void SimNodeBase::createBody(btVector3 position, float mass, void* userPointer)
{
    btTransform trans = btTransform::getIdentity();
    trans.setOrigin(position);

    btDefaultMotionState* motionState = new btDefaultMotionState(trans);

    btScalar bodyMass = mass;
    btVector3 bodyInertia;
    _shape->calculateLocalInertia(bodyMass, bodyInertia);

    btRigidBody::btRigidBodyConstructionInfo bodyConstrInfo =
        btRigidBody::btRigidBodyConstructionInfo(bodyMass, motionState, _shape, bodyInertia);
    bodyConstrInfo.m_restitution = 0.5f;
    bodyConstrInfo.m_friction = 0.5f;

    _body = new btRigidBody(bodyConstrInfo);
    _body->setUserPointer(userPointer);
    _body->setUserIndex(_tag);
}

void SimNodeBase::setTransform(glm::mat4 transform)
{
    if (_body) {
        _body->setWorldTransform(SimUtils::glmToBullet(transform));
    }
}

glm::mat4 SimNodeBase::getTransform()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform());
    }
    else return glm::identity<glm::mat4>();
}

void SimNodeBase::setPosition(glm::vec3 position)
{
    if (_body) {
        btTransform trans = _body->getWorldTransform();
        trans.setOrigin(SimUtils::glmToBullet(position));
        _body->setWorldTransform(trans);
    }
}

glm::vec3 SimNodeBase::getPosition()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform().getOrigin());
    }
    else return glm::vec3();
}

void SimNodeBase::setRotation(glm::quat rotation)
{
    if (_body) {
        _body->getWorldTransform().setRotation(SimUtils::glmToBullet(rotation));
    }
}

glm::quat SimNodeBase::getRotation()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform().getRotation());
    }
    else return glm::identity<glm::quat>();
}

// rigidbody
btRigidBody* SimNodeBase::getRigidBody() { return _body; }
bool SimNodeBase::hasBody() { return _body != nullptr; }

// shape
btCollisionShape* SimNodeBase::getShape() { return _shape; }

// meta
std::string SimNodeBase::getName() { return _name; }
int SimNodeBase::getTag() { return _tag; }

// owner
void SimNodeBase::addToWorld() { _ownerWorld->addRigidBody(_body); }
void SimNodeBase::removeFromWorld() { _ownerWorld->removeRigidBody(_body); }

// shader
void SimNodeBase::setShader(std::shared_ptr<ofShader> shader) { _shader = shader; }
std::shared_ptr<ofShader> SimNodeBase::getShader() { return _shader; }

void SimNodeBase::setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<ofMaterial> mtl, std::shared_ptr<ofTexture> tex)
{
    _shader = shader;
    _material = mtl;
    _texture = tex;
    bUseTexture = true;
}
void SimNodeBase::setTexture(std::shared_ptr<ofTexture> texture) {
    _texture = texture;
    bUseTexture = true;
}
void SimNodeBase::setMaterial(std::shared_ptr<ofMaterial> mtl) { _material = mtl; }
void SimNodeBase::setLight(std::shared_ptr<ofLight> light) { _light = light; }
void SimNodeBase::setMesh(std::shared_ptr<ofMesh> mesh) { _mesh = mesh; }

SimNodeBase::~SimNodeBase()
{
    if (hasBody()) {
        delete _body->getMotionState();
        delete _body;
        delete _shape;
    }
}
