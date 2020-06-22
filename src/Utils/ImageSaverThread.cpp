#include "ImageSaverThread.h"

ImageSaverThread::ImageSaverThread()
{
	startThread();
}

ImageSaverThread::~ImageSaverThread()
{
	_bufferChannel.close();
	_channelReady.close();
	waitForThread(true);
}

void ImageSaverThread::send(ofBuffer* pixels, std::string info)
{
	_infoChannel.send(info);
	_bufferChannel.send(pixels);
}

void ImageSaverThread::waitReady()
{
	bool ready;
	_channelReady.receive(ready);
}

void ImageSaverThread::threadedFunction()
{
	ofBuffer* buf;
	while (_bufferChannel.receive(buf))
	{
		std::string info;
		_infoChannel.receive(info);

		ofFile file;
		std::string fname = info + ".jpg";

		file.open(fname, ofFile::WriteOnly, true);
		file.writeFromBuffer(*buf);
		file.close();
		_channelReady.send(true);
	}
}
