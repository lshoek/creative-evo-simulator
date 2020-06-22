#include "ImageSaver.h"

void ImageSaver::setup(int width, int height)
{
    _writePixels.allocate(width, height, GL_RGBA);
    _width = width;
    _height = height;

    for (int i = 0; i < 2; i++) {
        _pixelWriteBuffers[i].allocate(_width*_height*4, GL_DYNAMIC_READ);
    }
    int iPBO = 0;
    _pboPtr = &_pixelWriteBuffers[iPBO];
}

void ImageSaver::save(const ofPixels& pix, std::string path)
{
    ofSaveImage(pix, _writeBuffer, OF_IMAGE_FORMAT_JPEG, OF_IMAGE_QUALITY_BEST);
    _imageSaverThread.send(&_writeBuffer, path);
}

void ImageSaver::save(const ofTexture& texture, std::string path)
{
    writeToPixels(texture);
    save(_writePixels, path);
}

void ImageSaver::copyToBuffer(const ofTexture& texture, std::function<void(uint8_t* ptr)> copyFunc)
{
    texture.copyTo(*_pboPtr);
    _pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);

    uint8_t* p = _pboPtr->map<uint8_t>(GL_READ_ONLY);
    copyFunc(p);

    _pboPtr->unmap();
    _pboPtr->unbind(GL_PIXEL_UNPACK_BUFFER);
    swapBuffers();
}

void ImageSaver::writeToPixels(const ofTexture& texture)
{
    texture.copyTo(*_pboPtr);

    _pboPtr->bind(GL_PIXEL_UNPACK_BUFFER);
    uint8_t* p = _pboPtr->map<uint8_t>(GL_READ_ONLY);
    _writePixels.setFromExternalPixels(p, _width, _height, OF_PIXELS_RGBA);

    _pboPtr->unmap();
    _pboPtr->unbind(GL_PIXEL_UNPACK_BUFFER);

    swapBuffers();
}

void ImageSaver::swapBuffers()
{
    _iPBO = (_iPBO + 1) % 2;
    _pboPtr = &_pixelWriteBuffers[_iPBO];
}
