#pragma once
#include "ofMain.h"

class ImageSaverThread : public ofThread{
public:
	ImageSaverThread();
	~ImageSaverThread();

	void setup(std::string path);

	void send(ofBuffer* img, std::string info);
	void waitReady();
	void threadedFunction();

private:
	ofPixels _pixels;

	ofThreadChannel<ofBuffer*> _bufferChannel;
	ofThreadChannel<std::string> _infoChannel;
	ofThreadChannel<bool> _channelReady;

	std::string _path;
	int _width, _height;
};
