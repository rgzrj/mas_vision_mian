#include "camera_thread.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

#include "hikcamera.hpp"
#include "mas_log.hpp"

namespace threads
{
void camera_thread_func(const std::string &config_path, size_t buffer_size, std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> buffer)
{
    // 相机初始化
    hikcamera::HikCamera cam(config_path);

    // 尝试打开相机
    if (!cam.openCamera())
    {
        MAS_LOG_WARN("Initial camera open failed; waiting for automatic reconnect");
    }

    if (g_shutdown)
    {
        return;
    }
    MAS_LOG_INFO("Camera processing thread started");

    while (!g_shutdown)
    {
        // 阻塞式获取图像
        CameraFrame data = cam.getImage();

        if (!data.frame.empty())
        {
            // 移入SPSCQueue（使用 try_push，防止在算法线程处理慢时导致采集线程空转）
            if (buffer)
            {
                (void)buffer->try_push(std::move(data));
            }
        }
        else
        {
            // 仅在获取图像失败时短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    MAS_LOG_INFO("Camera processing thread exited");
}
} // namespace threads