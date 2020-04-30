#pragma once
#include "SimNode.h"
#include "ofBufferObject.h"
#include "ofFbo.h"

class SimCanvasNode : public SimNodeBase
{
public:
	SimCanvasNode(btVector3 position, int tag, float size, float extraBounds, int x_res, int y_res, btDynamicsWorld* ownerWorld);
	~SimCanvasNode();

	void update();
	virtual void draw() override;

	void addBrushStroke(btVector3 location, float impulse);

	glm::ivec2 getCanvasResolution();
	ofFbo* getCanvasFbo();

	void setCanvasUpdateShader(std::shared_ptr<ofShader> shader);
	void enableBounds();

private:
	struct BrushCoord {
		glm::vec2 coord;
		float impulse;
		float active;

		static int size() {
			return sizeof(glm::vec2) + 2*sizeof(float);
		}
	};

	void initPlane(btVector3 position, float size);

	glm::ivec2 _canvasRes;
	float _canvasSize;
	float _margin;
	
	ofBufferObject _brushCoordBuffer;
	std::vector<BrushCoord> _brushCoordQueue;
	unsigned int _brushQueueSize = 0;

	ofMesh _canvasDrawQuad;

	// render buffers
	ofFbo _canvasFinalFbo;
	ofFbo _canvasFbo[2];
	int iFbo = 0;

	ofColor _brushColor;
	ofColor _canvasClearColor;

	std::unique_ptr<SimNode> _bounds[4];
	bool _bBounds = false;

	std::shared_ptr<ofShader> _canvasUpdateShader;
};
