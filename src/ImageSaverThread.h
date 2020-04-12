#pragma once
#include "ofMain.h"

class ImageSaverThread : public ofThread{
public:
	ImageSaverThread();
	~ImageSaverThread();

	void setup(std::string path);

	void send(ofBuffer* img);
	void waitReady();
	void threadedFunction();

private:
	ofPixels _pixels;
	ofThreadChannel<ofBuffer*> _channel;
	ofThreadChannel<bool> _channelReady;

	std::string _path;
	int _width, _height;
};
