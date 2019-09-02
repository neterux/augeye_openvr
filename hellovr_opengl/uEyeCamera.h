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
    void SetFocus(int value, bool bAddition);
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
};