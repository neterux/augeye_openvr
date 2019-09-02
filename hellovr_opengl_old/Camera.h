#pragma once
#include <uEye.h>

class Camera
{
public:
    Camera();
    ~Camera();

    void Init(int cameraId);
    void Update();
    int GetWidth() const;
    int GetHeight() const;
    char* GetImagePtr();
    void SetFocus(int value);
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