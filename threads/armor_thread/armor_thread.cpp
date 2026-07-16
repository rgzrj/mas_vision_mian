#include "armor_thread.hpp"

#include <opencv2/opencv.hpp>

#include "armor_detector.hpp"
#include "armor_shoot.hpp"
#include "armor_track.hpp"
#include "mas_log.hpp"
#include "serial_types.hpp"
#include "user_serial.hpp"

namespace threads
{
void armor_thread_func(std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> hikcamerabuffer,
                       std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> usbcamera_buffer)
{
    MAS_LOG_INFO("Armor processing thread started");

    auto_aim::ArmorDetector detector("config/auto_aim.yaml");
    auto_aim::ArmorTrack    tracker("config/auto_aim.yaml", "config/hikcamera.yaml");
    auto_aim::ArmorShoot    shooter("config/auto_aim.yaml");

    // 获取串口
    UserSerial &serial    = UserSerial::getInstance();
    VisionMode  last_mode = AUTO_AIM_RED;

    while (!g_shutdown)
    {
        bool processed = false;

        VisionMode current_mode = serial.getVisionMode();

        // 设置识别颜色（只在模式变化时设置）
        if (current_mode != last_mode)
        {
            if (current_mode == AUTO_AIM_RED)
            {
                detector.ArmorDetector_Set_Color(auto_aim::EnemyColor::RED);
                MAS_LOG_WARN("ArmorDetector_Set_Color RED");
            }
            else if (current_mode == AUTO_AIM_BLUE)
            {
                detector.ArmorDetector_Set_Color(auto_aim::EnemyColor::BLUE);
                MAS_LOG_WARN("ArmorDetector_Set_Color BLUE");
            }
            last_mode = current_mode;
        }

        // 非红蓝自瞄模式则跳过本次循环
        if (current_mode != AUTO_AIM_RED && current_mode != AUTO_AIM_BLUE)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 获取并处理海康相机数据
        if (hikcamerabuffer)
        {
            if (auto *f = hikcamerabuffer->front())
            {
                CameraFrame frame = std::move(*f);
                hikcamerabuffer->pop();
                if (!frame.frame.empty())
                {
                    // 检查四元数数据是否有效
                    if (!serial.isQuaternionValid())
                    {
                        processed = true;  // 标记为已处理，避免频繁休眠
                        serial.sendVision(0.0f, 0.0f, 0, 0);
                        continue;
                    }
                    
                    auto               armors = detector.ArmorDetect(frame.frame, "HikCamera");
                    Eigen::Quaterniond pos    = serial.q(frame.timestamp);
                    tracker.armor_pose().set_R_gimbal2world(pos);
                    auto target = tracker.track(armors, frame.timestamp, "HikCamera_track", frame.frame);
                    auto shoot_result = shooter.shoot(target, frame.timestamp, tracker.armor_pose().Get_R_gimbal2world(), frame.frame, "HikCamera_shoot");
                    serial.sendVision(shoot_result.target_yaw, shoot_result.target_pitch, shoot_result.found, shoot_result.fire_advice);
                    processed = true;
                }
            }
        }

        // 获取并处理 USB 相机数据
        if (usbcamera_buffer)
        {
            if (auto *f = usbcamera_buffer->front())
            {
                CameraFrame frame = std::move(*f);
                usbcamera_buffer->pop();
                if (!frame.frame.empty())
                {
                    auto armors = detector.ArmorDetect(frame.frame, "USB Camera");
                    processed   = true;
                }
            }
        }

        // 如果没有任何处理任务，休眠以降低 CPU 占用
        if (!processed)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    MAS_LOG_INFO("Armor processing thread exited");
}
} // namespace threads
