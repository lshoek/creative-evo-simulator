#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "ofGraphics.h"
#include "ofShader.h"
#include "ofMesh.h"
#include "ofTypes.h"
#include "ofBufferObject.h"
#include "ofFbo.h"

class SimCanvasNode
{
public:
	SimCanvasNode(int tag, float size, int x_res, int y_res);
	~SimCanvasNode();

	void update();
	void draw();

	void addBrushStroke(btVector3 location, float impulse);

	ofFbo* getCanvasFbo();

	void setRigidBody(btRigidBody* body);
	btRigidBody* getRigidBody();

	btCollisionShape* getShape();

	std::string getName();
	int getTag();

	bool hasBody();

	glm::ivec2 getCanvasResolution();

	void setTransform(glm::mat4 transform);
	glm::mat4 getTransform();

	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();

	void setRotation(glm::quat rotation);
	glm::quat getRotation();

	void setCanvasUpdateShader(ofShader* shader);
	void setShader(ofShader* shader);
	ofShader* getShader();

	void setTexture(ofTexture* texture);
	void setMaterial(ofMaterial* mtl);
	void setMesh(ofMesh* mesh);

	bool bUseTexture = false;

private:
	struct BrushCoord {
		glm::vec2 coord;
		float impulse;
		float active;

		static int size() {
			return sizeof(glm::vec2) + 2*sizeof(float);
		}
	};

	void initPlane(glm::vec3 position, float size, float mass);
	void createBody(glm::vec3 position, float mass);

	glm::ivec2 _canvasRes;
	float _canvasSize;

	ofMesh _canvasDrawQuad;

	btCollisionShape* _shape;
	btRigidBody* _body;

	std::string _name;
	int _tag;
	
	ofBufferObject _brushCoordBuffer;
	std::vector<BrushCoord> _brushCoordQueue;
	unsigned int _brushQueueSize = 0;

	// public canvas
	ofFbo _canvasFinalFbo;

	// render buffers
	ofFbo* _canvasFboPtr;
	ofFbo _canvasFbo[2];
	int iFbo = 0;

	ofColor _brushColor;
	ofColor _canvasClearColor;

	ofShader* _canvasUpdateShader;
	ofShader* _shader;
	ofTexture* _texture;
	ofMaterial* _material;
	ofMesh* _mesh;
};
