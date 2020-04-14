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
	SimCanvasNode(int tag, float size, int x_res, int y_res, btDynamicsWorld* ownerWorld);
	~SimCanvasNode();

	void update();
	void draw();

	void addBrushStroke(btVector3 location, float impulse);

	void setRigidBody(btRigidBody* body);
	btRigidBody* getRigidBody();

	btCollisionShape* getShape();

	std::string getName();
	int getTag();

	bool hasBody();

	glm::ivec2 getCanvasResolution();
	ofFbo* getCanvasFbo();

	// This should obviously be made consistent across node types
	void addToWorld();
	void removeFromWorld();

	void setTransform(glm::mat4 transform);
	glm::mat4 getTransform();

	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();

	void setRotation(glm::quat rotation);
	glm::quat getRotation();

	void setCanvasUpdateShader(std::shared_ptr<ofShader> shader);
	void setShader(std::shared_ptr<ofShader> shader);
	std::shared_ptr<ofShader> getShader();

	void setTexture(std::shared_ptr<ofTexture> texture);
	void setMaterial(std::shared_ptr<ofMaterial> mtl);
	void setMesh(std::shared_ptr<ofMesh> mesh);

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

	btDynamicsWorld* _ownerWorld;

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

	// render buffers
	ofFbo _canvasFinalFbo;
	ofFbo _canvasFbo[2];
	int iFbo = 0;

	ofColor _brushColor;
	ofColor _canvasClearColor;

	std::shared_ptr<ofShader> _canvasUpdateShader;
	std::shared_ptr<ofShader> _shader;
	std::shared_ptr<ofTexture> _texture;
	std::shared_ptr<ofMaterial> _material;
	std::shared_ptr<ofMesh> _mesh;
};
