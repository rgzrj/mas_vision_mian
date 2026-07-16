#include "usbcamera_thread.hpp"
#include "mas_log.hpp"
#include "usbcamera.hpp"
#include "usbcamera_config.hpp"

#include <memory>

namespace threads
{

void usb_camera_thread_func(size_t                                           buffer_size,
                            std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> buffer)
{
    std::string device_path = UsbCameraConfigManager::getDevicePath();
    if (device_path.empty())
    {
        MAS_LOG_WARN("No USB camera device path configured");
        return;
    }

    MAS_LOG_INFO("USB camera thread initializing with device: {}", device_path.c_str());

    auto usb_camera = std::make_unique<usbcamera::UsbCamera>(device_path);
    Base_Camera &cam = *usb_camera;
    if (!cam.openCamera())
    {
        MAS_LOG_ERROR("Could not open USB camera: {}", device_path.c_str());
        return;
    }

    MAS_LOG_INFO("USB camera processing thread started");

    while (!g_shutdown)
    {
        CameraFrame data = cam.getImage();
        if (!data.frame.empty())
        {
            if (buffer)
            {
                (void)buffer->try_push(std::move(data));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cam.closeCamera();
    MAS_LOG_INFO("USB camera processing thread exited");
}

} // namespace threads
