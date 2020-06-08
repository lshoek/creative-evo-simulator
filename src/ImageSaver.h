#pragma once
#include "ImageSaverThread.h"
#include "ofTexture.h"
#include "ofImage.h"

class ImageSaver
{
public:
	void setup(int width, int height);

	void save(const ofTexture& texture, std::string path);
	void save(const ofPixels& pixels, std::string path);

	void copyToBuffer(const ofTexture& texture, std::function<void(uint8_t* ptr)> copyFunc);

private:
	void writeToPixels(const ofTexture& tex);
	void swapBuffers();

	ImageSaverThread _imageSaverThread;

    ofBufferObject* _pboPtr;
	ofBufferObject _pixelWriteBuffers[2];

	uint32_t _width = 0;
	uint32_t _height = 0;

    ofPixels _writePixels;
    ofBuffer _writeBuffer;
    uint32_t _iPBO;
};
