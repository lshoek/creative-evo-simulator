#include "BufferSenderThread.h"
#include "SimDefines.h"

#define OSC_BUFFER_SIZE 2048

BufferSenderThread::BufferSenderThread()
{
	startThread();
}

BufferSenderThread::~BufferSenderThread()
{
	channel.close();
	channelReady.close();
	waitForThread(true);
}

void BufferSenderThread::setup(std::string host, int outport)
{
	sender.setup(host, outport);
}

void BufferSenderThread::setSaveSize(int w, int h)
{
	width = w;
	height = h;
}

void BufferSenderThread::setProcSize(size_t procSize)
{
	_procSize = procSize;
}

void BufferSenderThread::setInfoFlag(uint32_t infoFlag)
{
	_infoFlag = infoFlag;
}

void BufferSenderThread::send(ofBuffer* pixels)
{
	channel.send(pixels);
}

void BufferSenderThread::waitReady()
{
	bool ready;
	channelReady.receive(ready);
	bFirstSave = false;
}

void BufferSenderThread::threadedFunction()
{
	ofBuffer* buf;
	while(channel.receive(buf))
	{
		ofxOscMessage msg_start;

		int id = 0;
		char cbuf[OSC_BUFFER_SIZE];
		long max = OSC_BUFFER_SIZE;
		long processed = 0;
		long procsize = (_procSize > 0) ? _procSize : buf->size();

		msg_start.setAddress(OSC_FRAME_START);
		msg_start.addInt32Arg(ceil(procsize / max));
		sender.sendMessage(msg_start);

		while (processed < procsize)
		{
			long available = std::min(max, procsize - processed);
			memcpy(cbuf, buf->getData() + processed, available);
			processed += available;

			std::ostringstream ss;
			ss << OSC_FRAME_PART << id;

			ofBuffer tmpbuf;
			tmpbuf.set(cbuf, available);

			ofxOscMessage m;
			m.setAddress(ss.str());
			m.addBlobArg(tmpbuf);
			sender.sendMessage(m);
			id++;
		}
		ofxOscMessage msg_end;
		msg_end.setAddress(OSC_FRAME_END);
		msg_end.addInt32Arg(_infoFlag);
		sender.sendMessage(msg_end);

		std::ostringstream ss;
		ss << "sending " << procsize << " bytes over " << id << " messages.";
		ofLog() << ss.str();
		channelReady.send(true);
	}
}
