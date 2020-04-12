#include "ImageSaverThread.h"

ImageSaverThread::ImageSaverThread()
{
	startThread();
}

ImageSaverThread::~ImageSaverThread()
{
	_channel.close();
	_channelReady.close();
	waitForThread(true);
}

void ImageSaverThread::setup(std::string path)
{
	_path = path;
}

void ImageSaverThread::send(ofBuffer* pixels)
{
	_channel.send(pixels);
}

void ImageSaverThread::waitReady()
{
	bool ready;
	_channelReady.receive(ready);
}

void ImageSaverThread::threadedFunction()
{
	ofBuffer* buf;
	while (_channel.receive(buf))
	{
		ofFile file;
		std::string fname = _path + ofGetTimestampString() + ".jpg";

		file.open(fname, ofFile::WriteOnly, true);
		file.writeFromBuffer(*buf);
		file.close();
		_channelReady.send(true);
	}
}
