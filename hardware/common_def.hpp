#ifndef _COMMON_DEF_H_
#define _COMMON_DEF_H_

#include <opencv2/core/mat.hpp>
#include <chrono>

struct CameraFrame
{
    cv::Mat                               frame;
    std::chrono::steady_clock::time_point timestamp;
};


#endif // _COMMON_DEF_H_