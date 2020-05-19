#pragma once
#include "SimNode.h"
#include "ofBufferObject.h"
#include "ofFbo.h"
#include "ofxOpenCv.h"

class SimCanvasNode : public SimNodeBase
{
public:
	SimCanvasNode(btVector3 position, float size, float extraBounds, int xRes, int yRes, int xNeuralInput, int yNeuralInput, btDynamicsWorld* ownerWorld);
	~SimCanvasNode();

	void update();
	void updateNeuralInputBuffer();

	virtual void draw() override;
	virtual void drawImmediate() override;

	virtual void addToWorld() override;
	virtual void removeFromWorld() override;

	void addBrushStroke(btVector3 location, float pressure);

	glm::ivec2 getCanvasResolution();

	ofFbo* getCanvasRawFbo();
	ofFbo* getCanvasFbo();

	ofFbo* getCanvasNeuralInputRawFbo();

	const unsigned char* getNeuralInputsBufferChar();
	const double* getNeuralInputsBufferDouble();

	void setCanvasUpdateShader(std::shared_ptr<ofShader> shader);
	void setCanvasColorizeShader(std::shared_ptr<ofShader> shader);
	void enableBounds();

private:
	struct BrushCoord {
		glm::vec2 coord;
		float pressure;
		float active;

		static int size() {
			return sizeof(glm::vec2) + 2*sizeof(float);
		}
	};

	void initPlane(btVector3 position, float size);

	// canvas
	glm::ivec2 _canvasRes;
	glm::ivec2 _canvasNeuralInputRes;

	float _canvasSize;
	float _margin;

	ofMesh _canvasDrawQuad;
	ofMesh _canvasLowResDrawQuad;

	// brush coords
	ofBufferObject _brushCoordBuffer;
	std::vector<BrushCoord> _brushCoordQueue;
	unsigned int _brushQueueSize = 0;

	// render buffers
	ofFbo _canvasNeuralInputFbo;
	ofFbo _canvasColorFbo;
	ofFbo _canvasFbo[2];
	int iFbo = 0;

	// neural input
	cv::Mat _neuralInputMat;
	cv::Mat _neuralInputMatDouble;

	ofBufferObject _pixelWriteBuffers[2];
	ofBufferObject* _pboPtr;
	uint32_t iPbo;

	// colors
	ofColor _brushColor;

	// bounds
	std::unique_ptr<SimNode> _bounds[4];
	bool _bBounds = false;

	// special shaders
	std::shared_ptr<ofShader> _canvasUpdateShader;
	std::shared_ptr<ofShader> _canvasColorizeShader;
};
