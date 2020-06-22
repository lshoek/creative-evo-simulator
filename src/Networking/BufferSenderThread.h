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
	void setId(uint32_t id);

	void send(ofBuffer* img);
	void waitReady();
	void threadedFunction();

	bool bFirstSave = true;

private:
	ofThreadChannel<ofBuffer*> channel;
	ofThreadChannel<bool> channelReady;

	ofxOscSender sender;
	int width, height;

	uint32_t _id = 0;
	size_t _procSize = 0;
};
