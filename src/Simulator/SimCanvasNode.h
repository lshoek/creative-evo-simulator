#pragma once
#include "SimNode.h"
#include "ofBufferObject.h"
#include "ofFbo.h"
#include "ofPixels.h"

class SimCanvasNode : public SimNodeBase
{
public:
	SimCanvasNode(btVector3 position, float size, float viewSize, float boundsMargin, int xRes, int yRes, int xNeuralInput, int yNeuralInput, btDynamicsWorld* ownerWorld);
	~SimCanvasNode();

	void update();
	void updateConvPixelBuffer();
	void clearConvPixelBuffer();

	virtual void draw() override;
	virtual void drawImmediate() override;

	virtual void addToWorld() override;
	virtual void removeFromWorld() override;

	void addBrushStroke(btVector3 location, float pressure, bool active);
	void setLocalVisionRotation(btQuaternion rotation);

	void setCanvasUpdateShader(std::shared_ptr<ofShader> shader);
	void setCanvasColorizeShader(std::shared_ptr<ofShader> shader);
	void setSubTextureShader(std::shared_ptr<ofShader> shader);
	void spawnBounds(bool bDebugRender = true);

	glm::ivec2 getCanvasResolution();

	// Full-resolution single-channel paint map
	const ofFbo* getPaintMap() const;

	// Full-resolution RGBA paint map
	const ofFbo* getPaintMapRGBA() const;

	// Downsampled single-channel paint map
	const ofFbo* getViewMap() const;

	// Downsampled single-channel paint map buffer
	const ofPixels& getConvPixelBuffer();

private:
	struct BrushCoord {
		glm::vec2 coord;
		float pressure;
		float active;

		void reset() {
			coord = glm::vec2(0);
			pressure = -1.0f;
			active = 0.0f;
		}
		static int size() {
			return sizeof(glm::vec2) + 2*sizeof(float);
		}
	};

	void initPlane(btVector3 position, float size);
	void swapPbo();

	// canvas
	glm::ivec2 _canvasRes;
	glm::ivec2 _canvasConvRes;

	float _areaSize; // canvas + margin
	float _canvasSize;
	float _patchSize;
	float _margin;

	ofMesh _drawQuad;
	ofMesh _drawQuadConv;

	// brush coords
	ofBufferObject _brushCoordBuffer;
	std::vector<BrushCoord> _brushCoordQueue;
	unsigned int _brushQueueSize = 0;

	BrushCoord _cachedBrushCoord;
	btQuaternion _cachedLocalVisionRotation = btQuaternion::getIdentity();
	glm::vec4 _localVisionRotationMatrix = glm::vec4();

	// render buffers
	ofFbo _convFbo;
	ofFbo _convLocalFbo;
	ofFbo _colorFbo;
	ofFbo _fbo[2];
	int iFbo = 0;

	bool _bVariableBrushPressure = true;

	ofPixels _convPixelBuffer;
	ofBufferObject _pixelWriteBuffers[2];
	ofBufferObject* _pboPtr;
	uint32_t iPbo;

	// colors
	ofColor _brushColor;

	// bounds
	std::unique_ptr<SimNode> _bounds[4];
	bool _bBounds = false;

	// special shaders
	std::shared_ptr<ofShader> _updateShader;
	std::shared_ptr<ofShader> _colorizeShader;
	std::shared_ptr<ofShader> _subTextureShader;
};
