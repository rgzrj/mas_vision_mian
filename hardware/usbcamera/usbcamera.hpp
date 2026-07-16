#ifndef _USBCAMERA_H_
#define _USBCAMERA_H_

#include "common_def.hpp"
#include <opencv2/opencv.hpp>
#include <string>

namespace usbcamera
{

/**
 * @brief USB相机类
 */
class UsbCamera
{
  public:
    /**
     * @brief 构造函数
     * @param device_path 设备路径，默认为"/dev/video0"
     */
    explicit UsbCamera(const std::string &device_path);

    ~UsbCamera();

    bool open();

    void close();

    bool isOpened() const
    {
        return opened_ && cap_.isOpened();
    };

    CameraFrame captureImage();

    /**
     * @brief 获取相机设备路径
     * @return 设备路径
     */
    std::string getDevicePath() const;

  private:
    bool applyConfig(int width, int height, int fps, int auto_exposure, int exposure, int gain,
                     int auto_wb);

    std::string      device_path_;
    cv::VideoCapture cap_;
    bool             opened_;
};

} // namespace usbcamera

#endif // _USBCAMERA_H_