#include "BufferSender.h"
#include "OscProtocol.h"

void BufferSender::setup(std::string host, int outport)
{
	_senderThread.setup(host, outport);
    _iPBO = 0;
}

void BufferSender::allocate(uint32_t w, uint32_t h, ofPixelFormat type)
{
    _width = w;
    _height = h;
    _pixelFormat = type;
    _bufSize = _width * _height * sizeof(uint8_t);

    LZ4F_frameInfo_t info;
    memset(&info, 0, sizeof(info));
    info.frameType = LZ4F_frame;
    info.contentSize = _bufSize;

    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.frameInfo = info;

    _compressBound = LZ4F_compressFrameBound(_bufSize, &_prefs);

    for (int i = 0; i < 2; i++) {
        _pixelBuffer[i].allocate(_width, _height, _pixelFormat);
        _pixelBufferObject[i].allocate(_bufSize, GL_DYNAMIC_DRAW);
        _writeBuffer[i].allocate(_compressBound);
    }
    swapBuffers();
}

void BufferSender::writeToPixels(const ofTexture& tex)
{
    tex.copyTo(*_pixBufObjectPtr);

    _pixBufObjectPtr->bind(GL_PIXEL_UNPACK_BUFFER);
    uint8_t* p = _pixBufObjectPtr->map<uint8_t>(GL_READ_ONLY);
    _pixBufPtr->setFromExternalPixels(p, _width, _height, _pixelFormat);

    _pixBufObjectPtr->unmap();
    _pixBufObjectPtr->unbind(GL_PIXEL_UNPACK_BUFFER);
}

void BufferSender::send(const uint8_t* bytes, uint32_t id)
{
    uint64_t start = ofGetElapsedTimeMicros();
    size_t compSize = LZ4F_compressFrame(_writeBufPtr->getData(), _compressBound, bytes, _bufSize, &_prefs);
    uint64_t total = ofGetElapsedTimeMicros() - start;

    bool bError = LZ4F_isError(compSize);
    //ofLog() << ((bError) ? LZ4F_getErrorName(compSize) : ofToString(ofToString(total) + "us"));

    // compression successful
    if (!bError || compSize < _bufSize) {
        _senderThread.setProcSize(compSize);
    }
    //else {
    //    // compression failed. send full size buffer instead (not recommended)
    //    memcpy(_writeBufPtr->getData(), bytes, _bufSize);
    //    _senderThread.setProcSize(_bufSize);
    //}
    _senderThread.setId(id);
    _senderThread.send(_writeBufPtr);

    swapBuffers();
}

void BufferSender::send(const ofPixels& pixels, uint32_t id)
{
    send(pixels.getData(), id);
}

void BufferSender::send(const ofTexture& texture, uint32_t id)
{
    writeToPixels(texture);
    send(*_pixBufPtr, id);
}

void BufferSender::swapBuffers()
{
    _iPBO = (_iPBO + 1) % 2;

    _pixBufPtr = &_pixelBuffer[_iPBO];
    _writeBufPtr = &_writeBuffer[_iPBO];
    _pixBufObjectPtr = &_pixelBufferObject[_iPBO];
}
