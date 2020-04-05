#include "SimNode.h"
#include "SimUtils.h"
#include "toolbox.h"

void SimNode::init(std::string name, glm::vec3 position, glm::vec3 size, float mass, bool box, int tag)
{
    _name = name;
    _tag = tag;
    _size = SimUtils::glmToBullet(size);
    _color = ofColor::fromHsb(ofRandom(255), 0.75f*255, 1.0f*255, 1.0f*255);

    // create shape and body
    if (box) {
        _shape = new btBoxShape(_size);
        _mesh = ofMesh::box(size.x * 2, size.y * 2, size.z * 2);
    }
    else {
        _shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
        _mesh = tb::gridMesh(4, 4, size.x * 2, true);
        //_mesh = ofMesh::box(size.x * 2, size.y * 2, size.z*2);
        //position += glm::vec3(0, size.y/2.0f, 0);
    }
    tb::flipNormals(_mesh);
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
}

void SimNode::draw()
{
    if (_shader) {
        ofPushMatrix();
        ofMultMatrix(getTransform());
        
        _shader->begin();
        if (_texture) {
            _shader->setUniformTexture("tex", *_texture, 0);
        }
        _shader->setUniform4f("color", _color);
        _shader->setUniform4f("mtl_ambient", _material->getAmbientColor());
        _shader->setUniform4f("mtl_diffuse", _material->getDiffuseColor());
        _shader->setUniform4f("mtl_specular", _material->getSpecularColor());
        _shader->setUniform4f("mtl_emission", _material->getEmissiveColor());
        _shader->setUniform1f("mtl_shininess", _material->getShininess());

        _mesh.draw();
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

// meta
std::string SimNode::getName() { return _name; }
int SimNode::getTag() { return _tag; }

// shader
void SimNode::setShader(ofShader* shader) { _shader = shader; }
ofShader* SimNode::getShader() { return _shader; }

// texture
void SimNode::setTexture(ofTexture* texture) { _texture = texture; }
void SimNode::setMaterial(ofMaterial* mtl) { _material = mtl; }

void SimNode::dealloc()
{
    delete _body->getMotionState();
    delete _body;
    delete _shape;
    delete _shader;
}
