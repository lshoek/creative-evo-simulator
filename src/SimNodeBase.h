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
	~SimNodeBase();

	virtual void draw() = 0;

	btDynamicsWorld* _ownerWorld;

	void setRigidBody(btRigidBody* body);
	btRigidBody* getRigidBody();
	btCollisionShape* getShape();

	std::string getName();
	int getTag();

	bool hasBody();

	void addToWorld();
	void removeFromWorld();

	void setTransform(glm::mat4 transform);
	glm::mat4 getTransform();

	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();

	void setRotation(glm::quat rotation);
	glm::quat getRotation();

	void setShader(std::shared_ptr<ofShader> shader);
	std::shared_ptr<ofShader> getShader();

	void setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<ofMaterial> mtl, std::shared_ptr<ofTexture> tex);

	void setTexture(std::shared_ptr<ofTexture> texture);
	void setMaterial(std::shared_ptr<ofMaterial> mtl);
	void setLight(std::shared_ptr<ofLight> light);
	void setMesh(std::shared_ptr<ofMesh> mesh);

	bool bUseTexture = false;

protected:
	virtual void createBody(glm::vec3 position, float mass);

	btCollisionShape* _shape;
	btRigidBody* _body;

	std::string _name;
	int _tag;

	ofColor _color;
	std::shared_ptr<ofShader> _shader;
	std::shared_ptr<ofTexture> _texture;
	std::shared_ptr<ofMaterial> _material;
	std::shared_ptr<ofLight> _light;
	std::shared_ptr<ofMesh> _mesh;
};
