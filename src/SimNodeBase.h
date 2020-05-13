#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "ofGraphics.h"
#include "ofShader.h"
#include "ofMesh.h"
#include "ofTypes.h"

class SimNodeBase
{
public:
	SimNodeBase(int tag, btDynamicsWorld* owner);
	SimNodeBase(int tag, ofColor color, btDynamicsWorld* owner);
	virtual ~SimNodeBase();

	virtual void draw() = 0;
	virtual void drawImmediate() = 0;

	btDynamicsWorld* _ownerWorld;

	void setRigidBody(btRigidBody* body);
	btRigidBody* getRigidBody();
	btCollisionShape* getShape();

	void setTag(uint32_t tag);
	uint32_t getTag();

	bool hasBody();

	void addToWorld();
	void removeFromWorld();

	void setTransform(btTransform transform);
	btTransform getTransform();

	void setPosition(btVector3 position);
	btVector3 getPosition();

	void setRotation(btQuaternion rotation);
	btQuaternion getRotation();

	void setShader(std::shared_ptr<ofShader> shader);
	std::shared_ptr<ofShader> getShader();

	void setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<ofMaterial> mtl, std::shared_ptr<ofTexture> tex);

	virtual void setColor(ofColor c);
	void setTexture(std::shared_ptr<ofTexture> texture);
	void setMaterial(std::shared_ptr<ofMaterial> mtl);
	void setLight(std::shared_ptr<ofLight> light);
	void setMesh(std::shared_ptr<ofMesh> mesh);

	bool bUseTexture = false;
	bool bRender = true;

protected:
	virtual void createBody(btVector3 position, float mass, void* userPointer);

	btCollisionShape* _shape;
	btRigidBody* _body;
	uint32_t _tag;

	ofColor _color;
	std::shared_ptr<ofShader> _shader;
	std::shared_ptr<ofTexture> _texture;
	std::shared_ptr<ofMaterial> _material;
	std::shared_ptr<ofLight> _light;
	std::shared_ptr<ofMesh> _mesh;
};
