#pragma once
#include <uEye.h>

class uEyeCamera
{
public:
    uEyeCamera();
    ~uEyeCamera();

    void Init(int cameraId);
    int GetWidth() const;
    int GetHeight() const;
    char* GetImagePtr();
    void SetFocus(int value);
    int GetFocus();
    int GetId() const;

private:
    HIDS _camId;
    IS_SIZE_2D _size;
    int _bpp = 32;
    int _step;
    char* _image;
    int _memId;

    int focus;
    UINT focMax, focMin;

    INT lastError;
    char* lastErrorString;
    const unsigned short bufferLen = 128;
    char myErrorBuffer[128];
};