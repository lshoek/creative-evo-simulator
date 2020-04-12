#include "SimNode.h"
#include "SimUtils.h"
#include "toolbox.h"

SimNode::SimNode(int tag) : _tag(tag) 
{
    _color = ofColor::fromHsb(ofRandom(255), 0.75f * 255, 1.0f * 255, 1.0f * 255);
}

SimNode::SimNode(int tag, ofColor color) : _tag(tag)
{
    _color = color;
}

void SimNode::setRigidBody(btRigidBody* body)
{
    _body = body;
    _body->setUserIndex(_tag);

    _shape = _body->getCollisionShape();
}

void SimNode::initBox(glm::vec3 position, glm::vec3 size, float mass)
{
    _shape = new btBoxShape(btVector3(size.x, size.y, size.z));
    _mesh = new ofMesh(ofMesh::box(size.x * 2, size.y * 2, size.z * 2));
    createBody(position, mass);
}

void SimNode::initCapsule(glm::vec3 position, float radius, float height, float mass)
{
    _shape = new btCapsuleShape(radius, height);
    _mesh = new ofMesh(ofMesh::cylinder(radius, height));
    createBody(position, mass);
}

void SimNode::initPlane(glm::vec3 position, float size, float mass)
{
    _shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    _mesh = new ofMesh(tb::gridMesh(2, 2, size*2, true));
    createBody(position, mass);
}

void SimNode::createBody(glm::vec3 position, float mass)
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

void SimNode::draw()
{
    if (_shader) {
        ofPushMatrix();
        ofMultMatrix(getTransform());

        _shader->begin();
        if (bUseTexture) {
            _shader->setUniformTexture("tex", *_texture, 0);
        }
        _shader->setUniform4f("color", _color);
        _shader->setUniform4f("mtl_ambient", _material->getAmbientColor());
        _shader->setUniform4f("mtl_diffuse", _material->getDiffuseColor());
        _shader->setUniform4f("mtl_specular", _material->getSpecularColor());
        _shader->setUniform4f("mtl_emission", _material->getEmissiveColor());
        _shader->setUniform1f("mtl_shininess", _material->getShininess());

        _mesh->draw();
        _shader->end();

        ofPopMatrix();
    }
}

void SimNode::setTransform(glm::mat4 transform)
{
    if (_body) {
        _body->setWorldTransform(SimUtils::glmToBullet(transform));
    }
}

glm::mat4 SimNode::getTransform()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform());
    }
    else return glm::identity<glm::mat4>();
}

void SimNode::setPosition(glm::vec3 position)
{
    if (_body) {
        btTransform trans = _body->getWorldTransform();
        trans.setOrigin(SimUtils::glmToBullet(position));
        _body->setWorldTransform(trans);
    }
}

glm::vec3 SimNode::getPosition()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform().getOrigin());
    }
    else return glm::vec3();
}

void SimNode::setRotation(glm::quat rotation)
{
    if (_body) {
        _body->getWorldTransform().setRotation(SimUtils::glmToBullet(rotation));
    }
}

glm::quat SimNode::getRotation()
{
    if (_body) {
        return SimUtils::bulletToGlm(_body->getWorldTransform().getRotation());
    }
    else return glm::identity<glm::quat>();
}

// rigidbody
btRigidBody* SimNode::getRigidBody() { return _body; }
bool SimNode::hasBody() { return _body != nullptr; }

// shape
btCollisionShape* SimNode::getShape() { return _shape; }

// meta
std::string SimNode::getName() { return _name; }
int SimNode::getTag() { return _tag; }

// shader
void SimNode::setShader(ofShader* shader) { _shader = shader; }
ofShader* SimNode::getShader() { return _shader; }

void SimNode::setTexture(ofTexture* texture) { 
    _texture = texture; 
    bUseTexture = true;
}
void SimNode::setMaterial(ofMaterial* mtl) { _material = mtl; }
void SimNode::setMesh(ofMesh* mesh) { _mesh = mesh; }

SimNode::~SimNode()
{
    delete _body->getMotionState();
    delete _body;
    delete _shape;

    delete _shader;
    delete _texture;
    delete _material;
    delete _mesh;
}
