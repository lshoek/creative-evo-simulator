#pragma once
#include "ofxOscReceiver.h"
//#include "lz4frame.h"
//#include "lz4.h"

class BufferReceiver
{
public:
	void setup(int inport);
	void allocate(size_t numOutputs);

	const std::vector<float>& receive();

private:
	ofxOscReceiver receiver;
	std::vector<float> _floatBuffer;

	size_t _bufSize = 0;
};
