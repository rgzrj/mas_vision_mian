#include "usbcamera.hpp"
#include "mas_log.hpp"
#include "usbcamera_config.hpp"
#include <opencv2/videoio.hpp>

namespace usbcamera
{

UsbCamera::UsbCamera(const std::string &device_path) : device_path_(device_path), opened_(false)
{
}

UsbCamera::~UsbCamera()
{
    close();
}

bool UsbCamera::open()
{
    if (opened_)
    {
        return true;
    }

    try
    {
        // 使用设备路径打开相机
        cap_.open(device_path_, cv::CAP_V4L2);

        if (!cap_.isOpened())
        {
            return false;
        }

        if (cap_.isOpened())
        {
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

            opened_ = true;

            applyConfig(UsbCameraConfigManager::getWidth(), UsbCameraConfigManager::getHeight(),
                        UsbCameraConfigManager::getFps(), UsbCameraConfigManager::getAutoExposure(),
                        UsbCameraConfigManager::getExposure(), UsbCameraConfigManager::getGain(),
                        UsbCameraConfigManager::getAutoWb());

            MAS_LOG_INFO("Successfully opened USB camera. Path: {}", device_path_.c_str());

            return true;
        }
        else
        {
            MAS_LOG_ERROR("Could not open USB camera. Path: {}", device_path_.c_str());
            return false;
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception opening USB camera: {}", e.what());
        return false;
    }
}

void UsbCamera::close()
{
    if (cap_.isOpened())
    {
        cap_.release();
        opened_ = false;
        MAS_LOG_INFO("USB camera closed");
    }
}

bool UsbCamera::applyConfig(int width, int height, int fps, int auto_exposure, int exposure,
                            int gain, int auto_wb)
{
    if (!isOpened())
    {
        MAS_LOG_WARN("Cannot apply config when camera is not opened");
        return false;
    }

    bool success = true;

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, auto_exposure);
    cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
    cap_.set(cv::CAP_PROP_GAIN, gain);
    cap_.set(cv::CAP_PROP_AUTO_WB, auto_wb);

    return success;
}

CameraFrame UsbCamera::captureImage()
{
    if (!isOpened())
    {
        MAS_LOG_WARN("Cannot capture image when camera is not opened");
        return CameraFrame{};
    }

    try
    {
        cv::Mat frame;
        bool    ret = cap_.read(frame);
        if (!ret)
        {
            return CameraFrame{};
        }
        return CameraFrame{frame, std::chrono::steady_clock::now()};
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception capturing image: {}", e.what());
        return CameraFrame{};
    }
}

std::string UsbCamera::getDevicePath() const
{
    return device_path_;
}

} // namespace usbcamera
