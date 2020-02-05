#include <uEyeCamera.h>
#include <iostream>

uEyeCamera::uEyeCamera()
    : _camId(0)
    , _image(0)
    , _memId(0)
    , _size({ 0, 0 })
    , _step(0)
    , focMax(0)
    , focMin(0)
    , focus(0)
{
}

uEyeCamera::~uEyeCamera()
{
    is_FreeImageMem(_camId, _image, _memId);
    is_ExitCamera(_camId);
    std::cout << "Camera" << _camId << "  closed" << std::endl;
}

void uEyeCamera::Init(int CameraId)
{
    _camId = CameraId;
    if (is_InitCamera(&_camId, nullptr) != IS_SUCCESS)
        std::cout << "Camera" << CameraId << " init failed" << std::endl;
    
    if (is_ParameterSet(_camId, IS_PARAMETERSET_CMD_LOAD_EEPROM, nullptr, NULL) != IS_SUCCESS)
        std::cout << "Camera" << CameraId << " ParameterSet failed" << std::endl;

    if (is_SetColorMode(_camId, IS_CM_RGBA8_PACKED) != IS_SUCCESS)
        std::cout << "Camera" << CameraId << " ColorMode set failed" << std::endl;

    is_Focus(_camId, FOC_CMD_GET_MANUAL_FOCUS, &focus, 4);
    is_Focus(_camId, FOC_CMD_GET_MANUAL_FOCUS_MIN, &focMin, 4);
    is_Focus(_camId, FOC_CMD_GET_MANUAL_FOCUS_MAX, &focMax, 4);
    std::cout << "Camera" << CameraId << " focus range: " << focMin << "<" << focus << "<" << focMax << std::endl;
    
    is_AOI(_camId, IS_AOI_IMAGE_GET_SIZE, (void*)& _size, sizeof(_size));
    std::cout << "Camera" << CameraId << " image WxH: " << _size.s32Width << "x" << _size.s32Height << std::endl;
    
    if (is_AllocImageMem(_camId, _size.s32Width, _size.s32Height, _bpp, &_image, &_memId) != IS_SUCCESS)
        std::cout << "Camera" << CameraId << " allocate memory failed" << std::endl;
    
    if (is_SetImageMem(_camId, _image, _memId) != IS_SUCCESS)
        std::cout << "Camera" << CameraId << " set image failed" << std::endl;
    
    if (is_CaptureVideo(_camId, IS_DONT_WAIT) != IS_SUCCESS)
    {
        std::cout << "Camera" << CameraId << " start capture failed" << std::endl;
    }
    else
    {
        std::cout << "Camera" << CameraId << " start capture" << std::endl;
    }
}

int uEyeCamera::GetWidth() const
{
    return _size.s32Width;
}

int uEyeCamera::GetHeight() const
{
    return _size.s32Height;
}

char* uEyeCamera::GetImagePtr()
{
    return _image;
}

void uEyeCamera::SetFocus(int value)
{
    focus = value;
    if (focus > (int)focMax) focus = focMax;
    if (focus < (int)focMin) focus = focMin;

    is_Focus(_camId, FOC_CMD_SET_MANUAL_FOCUS, &focus, 4);
    // std::cout << "Camera" << _camId << ": " << focus << std::endl;
    
    // Error check
    INT nValue = 0;
    INT ret = is_Focus(_camId, FOC_CMD_GET_AUTOFOCUS_STATUS, (void*)&nValue, sizeof(nValue));
    if (ret != IS_SUCCESS)
    {
        INT nRet = is_GetError(_camId, &lastError, &lastErrorString);
        if (nRet == IS_SUCCESS)
        {
            memset(myErrorBuffer, 0, bufferLen);
            strncpy_s(myErrorBuffer, lastErrorString, bufferLen);
            lastErrorString = 0;
            std::cout << "uEye Error - code " << ret << ": " << myErrorBuffer << std::endl;
        }
    }
}

int uEyeCamera::GetFocus()
{
    is_Focus(_camId, FOC_CMD_GET_MANUAL_FOCUS, &focus, 4);
    return focus;
}

int uEyeCamera::GetId() const
{
    return (int)_camId;
}
