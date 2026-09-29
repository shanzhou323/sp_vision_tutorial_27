#include "camera.hpp"

Camera::Camera() {
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        printf("[Camera] No camera found\n");
        return;
    }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) { return; }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) { return; }

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 60);

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) { return; }

    opened_ = true;
    printf("[Camera] Opened\n");
}

Camera::~Camera() {
    if (!opened_) return;
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    printf("[Camera] Closed\n");
}

bool Camera::read(cv::Mat& frame) {
    if (!opened_) return false;

    MV_FRAME_OUT raw;
    int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
    if (ret != MV_OK) return false;

    frame = transfer(raw);

    MV_CC_FreeImageBuffer(handle_, &raw);
    return true;
}

cv::Mat Camera::transfer(MV_FRAME_OUT& raw) {
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight),
                CV_8U, raw.pBufAddr);

    MvGvspPixelType pixel_type = raw.stFrameInfo.enPixelType;
    static const std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
    };

    if (type_map.count(pixel_type)) {
        cv::cvtColor(img, img, type_map.at(pixel_type));
    }
    return img.clone();
}
