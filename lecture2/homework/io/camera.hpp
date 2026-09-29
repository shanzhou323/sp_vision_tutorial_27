#pragma once
#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class Camera {
public:
    Camera();
    ~Camera();
    bool read(cv::Mat& frame);

private:
    void*      handle_   = nullptr;
    bool       opened_    = false;
    cv::Mat    transfer(MV_FRAME_OUT& raw);
};
