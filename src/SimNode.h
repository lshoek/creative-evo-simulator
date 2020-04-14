#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "ofGraphics.h"
#include "ofShader.h"
#include "ofMesh.h"
#include "ofTypes.h"

class SimNode
{
public:
	SimNode(int tag);
	SimNode(int tag, ofColor color);
	~SimNode();

	// conveniece init functions
	void initBox(glm::vec3 position, glm::vec3 size, float mass);
	void initCapsule(glm::vec3 position, float radius, float height, float mass);
	void initPlane(glm::vec3 position, float size, float mass);

	void draw();

	void setRigidBody(btRigidBody* body);

	btRigidBody* getRigidBody();
	btCollisionShape* getShape();

	std::string getName();
	int getTag();

	bool hasBody();

	void setTransform(glm::mat4 transform);
	glm::mat4 getTransform();

	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();

	void setRotation(glm::quat rotation);
	glm::quat getRotation();

	void setShader(std::shared_ptr<ofShader> shader);
	std::shared_ptr<ofShader> getShader();

	void setTexture(std::shared_ptr<ofTexture> texture);
	void setMaterial(std::shared_ptr<ofMaterial> mtl);
	void setMesh(std::shared_ptr<ofMesh> mesh);

	bool bUseTexture = false;

private:
	void createBody(glm::vec3 position, float mass);

	btCollisionShape* _shape;
	btRigidBody* _body;

	std::string _name;
	int _tag;

	ofColor _color;
	std::shared_ptr<ofShader> _shader;
	std::shared_ptr<ofTexture> _texture;
	std::shared_ptr<ofMaterial> _material;
	std::shared_ptr<ofMesh> _mesh;
};
