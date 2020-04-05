#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "ofGraphics.h"
#include "ofShader.h"
#include "ofMesh.h"
#include "ofTypes.h"

class SimNode
{
public:
	void init(std::string name, glm::vec3 position, glm::vec3 size, float mass, bool box, int tag);
	void createBody(glm::vec3 position, float mass);
	void draw();
	void dealloc();

	btRigidBody* getRigidBody();
	std::string getName();
	int getTag();

	bool hasBody();

	void setTransform(glm::mat4 transform);
	glm::mat4 getTransform();

	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();

	void setRotation(glm::quat rotation);
	glm::quat getRotation();

	void setShader(ofShader* shader);
	ofShader* getShader();

	void setTexture(ofTexture* texture);
	void setMaterial(ofMaterial* mtl);

private:
	btCollisionShape* _shape;
	btRigidBody* _body;
	btVector3 _size;

	std::string _name;
	int _tag;

	ofShader* _shader;
	ofTexture* _texture;
	ofMaterial* _material;
	ofColor _color;
	ofMesh _mesh;
};
