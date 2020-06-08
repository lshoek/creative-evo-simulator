#pragma once
#include "BufferSenderThread.h"
#include "ofTexture.h"
#include "ofImage.h"
#include "lz4frame.h"
#include "lz4.h"

class BufferSender
{
public:
	void setup(std::string host, int outport);
	void allocate(uint32_t w, uint32_t h, ofPixelFormat type);
	void setSave(bool b);

	void send(const uint8_t* bytes);
	void send(const ofPixels& pixels);
	void send(const ofTexture& texture);

private:
	void writeToPixels(const ofTexture& tex);
	void swapBuffers();

	BufferSenderThread _senderThread;
	LZ4F_preferences_t _prefs;

	ofPixels* _pixBufPtr;
	ofBufferObject* _pixBufObjectPtr;
	ofBuffer* _writeBufPtr;

	ofPixels _pixelBuffer[2];
	ofBufferObject _pixelBufferObject[2];
	ofBuffer _writeBuffer[2];

	uint32_t _iPBO;

	ofPixelFormat _pixelFormat = OF_PIXELS_GRAY;
	uint32_t _width = 0;
	uint32_t _height = 0;
	size_t _bufSize = 0;
	size_t _compressBound = 0;

	bool _bSaveToDisk = false;
};
