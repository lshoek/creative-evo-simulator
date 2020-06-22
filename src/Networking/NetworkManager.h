#pragma once
#include "BufferSender.h"
#include "ofEvents.h"
#include "ofEvent.h"
#include "Utils/Scheduler.h"
#include "Simulator/SimInstance.h"
#include "Simulator/SimInfo.h"

class NetworkManager
{
public:
	ofEvent<void> onConnectionEstablished;
	ofEvent<void> onConnectionClosed;
	ofEvent<SimInfo> onInfoReceived;

	void setup(std::string host, int inPort, int outPort);
	void allocate(size_t numJoints, size_t numOutputs, uint32_t w, uint32_t h, ofPixelFormat type);
	void search();

	void send(std::string address);
	void send(std::string address, std::string arg);
	void send(std::string address, int arg);
	void send(std::string address, double arg);
	void sendState(SimInstance* instance);
	
	void receive();

	void close();

	const std::vector<float>& popOutputBuffer();
	bool isAgentOutputQueued();
	uint32_t getQueuedAgentId();

	BufferSender& getBufferSender();

private:
	enum State { IDLE, HANDSHAKE, ACTIVE };
	State _state = IDLE;

	ofxOscSender _sender;
	ofxOscReceiver _receiver;

	// For sending large messages in compressed chunks like image data
	BufferSender _bufferSender;

	ofEventListener _handshakeListener;
	Scheduler _repeatMessageScheduler;

	std::vector<float> _jointsInputBuffer;
	size_t _jointsInputBufferSize = 16;

	std::vector<float> _outputBuffer;
	size_t _outputBufferSize = 16;

	uint32_t _queuedAgentId = 0;
	bool _bOutputQueued = false;
};
