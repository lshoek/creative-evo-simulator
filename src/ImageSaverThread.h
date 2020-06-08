#pragma once
#include "ofMain.h"

class ImageSaverThread : public ofThread{
public:
	ImageSaverThread();
	~ImageSaverThread();

	void send(ofBuffer* img, std::string info);
	void waitReady();
	void threadedFunction();

private:
	ofPixels _pixels;

	ofThreadChannel<ofBuffer*> _bufferChannel;
	ofThreadChannel<std::string> _infoChannel;
	ofThreadChannel<bool> _channelReady;

	uint32_t _width = 0;
	uint32_t _height = 0;

	std::string _path;
};
