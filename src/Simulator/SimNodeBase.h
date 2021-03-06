#pragma once
#include "btBulletDynamicsCommon.h"
#include "ofGraphics.h"
#include "ofShader.h"
#include "ofMesh.h"
#include "ofTypes.h"
#include "Graphics/MaterialBase.h"

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

	virtual void addToWorld();
	virtual void removeFromWorld();

	void setTransform(btTransform transform);
	btTransform getTransform();

	void setPosition(btVector3 position);
	btVector3 getPosition();

	void setRotation(btQuaternion rotation);
	btQuaternion getRotation();

	void setShader(std::shared_ptr<ofShader> shader);
	std::shared_ptr<ofShader> getShader();

	void setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<MaterialBase> mtl);

	virtual void setColor(ofColor c);
	void setTexture(std::shared_ptr<ofTexture> texture);
	void setMaterial(std::shared_ptr<MaterialBase> mtl);
	void setLight(std::shared_ptr<ofLight> light);
	void setMesh(std::shared_ptr<ofMesh> mesh);

	bool bUseTexture = false;
	bool bRender = true;

protected:
	virtual void createBody(btVector3 position, btCollisionShape* shape, float mass, void* userPointer);
	void dealloc();

	btCollisionShape* _shape = NULL;
	btRigidBody* _body = NULL;
	uint32_t _tag;

	ofColor _color;

	std::shared_ptr<ofShader> _shader;
	std::shared_ptr<ofTexture> _texture;
	std::shared_ptr<MaterialBase> _material;
	std::shared_ptr<ofLight> _light;
	std::shared_ptr<ofMesh> _mesh;
};
