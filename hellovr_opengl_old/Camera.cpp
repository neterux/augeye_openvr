#include <Camera.h>
#include <iostream>

Camera::Camera()
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

Camera::~Camera()
{
    is_FreeImageMem(_camId, _image, _memId);
    is_ExitCamera(_camId);
}

void Camera::Init(int CameraId)
{
    _camId = CameraId;
    if (is_InitCamera(&_camId, NULL) != IS_SUCCESS)
        std::cout << "Camera" << CameraId << " init failed" << std::endl;
    
    if (is_ParameterSet(_camId, IS_PARAMETERSET_CMD_LOAD_EEPROM, NULL, NULL) != IS_SUCCESS)
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

void Camera::Update()
{
    //if (_unity == nullptr || _texture == nullptr || _image == nullptr) return;

    //std::lock_guard<std::mutex> lock(_mutex);
    //auto device = _unity->Get<IUnityGraphicsD3D11>()->GetDevice();
    //ID3D11DeviceContext* context = nullptr;
    //device->GetImmediateContext(&context);
    //context->UpdateSubresource(_texture, 0, nullptr, _image, _step, 0);
    //context->Release();
}

int Camera::GetWidth() const
{
    return _size.s32Width;
}

int Camera::GetHeight() const
{
    return _size.s32Height;
}

char* Camera::GetImagePtr()
{
    return _image;
}

void Camera::SetFocus(int value)
{
    focus += value;
    if (focus > (int)focMax) focus = focMax;
    if (focus < (int)focMin) focus = focMin;
    is_Focus(_camId, FOC_CMD_SET_MANUAL_FOCUS, &focus, 4);
}

int Camera::GetId() const
{
    return (int)_camId;
}
