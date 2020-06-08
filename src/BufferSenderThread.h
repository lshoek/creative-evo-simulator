#pragma once
#include "ofBufferObject.h"
#include "ofThread.h"
#include "ofThreadChannel.h"
#include "ofxOsc.h"

class BufferSenderThread : public ofThread 
{
public:
	BufferSenderThread();
	~BufferSenderThread();

	void setup(std::string host, int outport);
	void setSaveSize(int w, int h);

	void setProcSize(size_t procSize);
	void setInfoFlag(uint32_t infoFlag);

	void send(ofBuffer* img);
	void waitReady();
	void threadedFunction();

	bool bFirstSave = true;

private:
	ofThreadChannel<ofBuffer*> channel;
	ofThreadChannel<bool> channelReady;

	ofxOscSender sender;
	int width, height;

	uint8_t _infoFlag = 0;
	size_t _procSize = 0;
};
