#include "BufferReceiver.h"
#include "SimDefines.h"

void BufferReceiver::setup(int inport)
{
	receiver.setup(inport);
}

void BufferReceiver::allocate(size_t numOutputs) 
{
	_floatBuffer.resize(numOutputs);
	_bufSize = numOutputs * sizeof(float);
}

const std::vector<float>& BufferReceiver::receive()
{
	if (receiver.isListening())
	{
		ofBuffer buf;
		bool bMsgStart = false;
		int numFrameParts = 0;

		while (receiver.hasWaitingMessages())
		{
			ofxOscMessage m;
			receiver.getNextMessage(&m);
			std::vector<std::string> tokens = ofSplitString(m.getAddress(), "/");

			if (tokens[1] == OSC_FRAME) {
				if (tokens[2] == OSC_START) {
					bMsgStart = true;
				}
				else if (tokens[2] == OSC_PART) {
					if (!bMsgStart) continue;
					buf.append(m.getArgAsBlob(0));
					numFrameParts++;
				}
				else if (tokens[2] == OSC_END) {
					if (!bMsgStart) continue;
					if (buf.size() > 0) {
						// check whether buffer is large enough as well !!
						memcpy(&_floatBuffer[0], buf.getData(), _bufSize);
						break;
					}
				}
			}
		}
		std::ostringstream ss;
		ss << "received " << buf.size() << " bytes over " << numFrameParts << " messages.";
		ofLog() << ss.str();

		return _floatBuffer;
	}
}
