#include "Simulator/SimNode.h"
#include "Simulator/SimCreature.h"
#include "Simulator/SimDefines.h"
#include "Utils/SimUtils.h"
#include "Utils/MeshUtils.h"
#include "Utils/toolbox.h"

SimNode::SimNode(int tag, btDynamicsWorld* owner) : SimNodeBase(tag, owner)
{
    setColor(ofColor::fromHsb(ofRandom(255), 0.7f * 255, 255, 255));
}
SimNode::SimNode(int tag, ofColor color, btDynamicsWorld* owner) : SimNodeBase(tag, color, owner)
{
    setColor(color);
}

void SimNode::initBox(btVector3 position, btVector3 size, float mass)
{
    _mesh = std::make_shared<ofMesh>(ofMesh::box(size.x() * 2, size.y() * 2, size.z() * 2));

    btCollisionShape* shape = new btBoxShape(size);
    createBody(position, shape, mass, this);
}

void SimNode::initCapsule(btVector3 position, float radius, float height, float mass)
{
    _mesh = std::make_shared<ofMesh>(ofMesh::cylinder(radius, height));

    btCollisionShape* shape = new btCapsuleShape(radius, height);
    createBody(position, shape, mass, this);
}

void SimNode::initPlane(btVector3 position, float size, float mass)
{
    _mesh = std::make_shared<ofMesh>(MeshUtils::gridMesh(2, 2, size*2, true));

    btCollisionShape* shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    createBody(position, shape, mass, this);
}

void SimNode::draw()
{
    if (bRender) {
        if (_shader) {
            ofPushMatrix();
            ofMultMatrix(SimUtils::bulletToGlm(getTransform()));

            _shader->begin();
            if (bUseTexture) {
                _shader->setUniformTexture("tex", *_texture, 0);
            }
            _shader->setUniform4f("color", _color);
            _shader->setUniform4f("brush_color", _inkColor);
            _shader->setUniform1f("brush", isBrush());
            _shader->setUniform1f("brush_active", isBrushActivated());
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
}

void SimNode::drawImmediate()
{
    ofPushMatrix();
    ofMultMatrix(SimUtils::bulletToGlm(getTransform()));
    _mesh->draw();
    ofPopMatrix();
}

SimCreature* SimNode::getCreaturePtr()
{
    return _ownerCreature;
}

void SimNode::setCreatureOwner(SimCreature* creaturePtr)
{
    _ownerCreature = creaturePtr;
}

bool SimNode::isBrush()
{
    return _body->getUserIndex() & BrushTag;
}

bool SimNode::isBrushActivated()
{
    return _brushPressure >= _brushMinThreshold;
}

float SimNode::getBrushPressure()
{
    return _brushPressure;
}

void SimNode::setBrushPressure(float pressure)
{
    if (isBrush()) {
        _brushPressure = pressure;
        _color = isBrushActivated() ? _inkColor : _cachedColor;
    }
}

void SimNode::setColor(ofColor color)
{
    SimNodeBase::setColor(color);
    _cachedColor = color;
}

void SimNode::setInkColor(ofColor inkColor)
{
    _inkColor = inkColor;
}

SimNode::~SimNode()
{

}
