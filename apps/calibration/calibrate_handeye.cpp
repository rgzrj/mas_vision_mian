#include "calibrate_handeye.hpp"

#include <BS_thread_pool.hpp>
#include <Eigen/Dense>
#include <fmt/core.h>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "camera_thread.hpp"
#include "display.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "serial_thread.hpp"
#include "user_serial.hpp"

extern std::atomic<bool> g_shutdown;

namespace calibration
{

// YAML配置文件路径
constexpr const char *CONFIG_PATH        = "config/hikcamera.yaml";
constexpr const char *CAMERA_CONFIG_PATH = "config/hikcamera.yaml";

// 生成标定板3D点
std::vector<cv::Point3f> centers_3d(const cv::Size &pattern_size, float center_distance_mm)
{
    std::vector<cv::Point3f> centers_3d;

    for (int i = 0; i < pattern_size.height; i++)
    {
        for (int j = 0; j < pattern_size.width; j++)
        {
            float x = 0;
            float y = (-j + 0.5 * pattern_size.width) * center_distance_mm;
            float z = (-i + 0.5 * pattern_size.height) * center_distance_mm;
            centers_3d.push_back({x, y, z});
        }
    }

    return centers_3d;
}

// 全局状态用于回调
struct CalibrationState
{
    bool       save_requested  = false;
    bool       clear_requested = false;
    bool       quit_requested  = false;
    bool       board_detected  = false;
    std::mutex mutex;
};

// SDL事件回调函数
bool sdl_event_callback(const SDL_Event &event, void *user_data)
{
    auto *state = static_cast<CalibrationState *>(user_data);

    if (event.type == SDL_KEYDOWN)
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        switch (event.key.keysym.sym)
        {
        case SDLK_s:
            state->save_requested = true;
            return true;
        case SDLK_c:
            state->clear_requested = true;
            return true;
        case SDLK_q:
            state->quit_requested = true;
            return true;
        }
    }
    return false;
}

int calibrate_handeye_main()
{
    // 初始化日志（已在main中初始化，这里不需要重复）
    MAS_LOG_INFO("HandEye Calibration Start");

    // 读取配置
    YAML::Node config = YAML::LoadFile(CONFIG_PATH);

    auto     pattern_cols       = config["calibration"]["pattern_cols"].as<int>();
    auto     pattern_rows       = config["calibration"]["pattern_rows"].as<int>();
    auto     center_distance_mm = config["calibration"]["center_distance_mm"].as<float>();
    cv::Size pattern_size(pattern_cols, pattern_rows);

    auto    camera_matrix_data  = config["calibration"]["camera_matrix"].as<std::vector<double>>();
    auto    distort_coeffs_data = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
    cv::Mat camera_matrix       = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
    cv::Mat distort_coeffs(distort_coeffs_data);

    auto            R_gimbal2imubody_data = config["handeye_calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
    Eigen::Matrix3d R_gimbal2imubody      = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());

    // 创建SPSCQueue
    size_t camera_buffer_size = 2;
    auto   camera_buffer      = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(camera_buffer_size);

    // 创建线程池，使用硬件线程数
    BS::thread_pool pool(std::thread::hardware_concurrency());

    // 启动显示线程
    rm_utils::Display &display = rm_utils::Display::getInstance();
    display.start(pool);

    // 设置校准状态和回调
    CalibrationState calib_state;
    display.set_event_callback(sdl_event_callback, &calib_state);

    // 启动相机线程
    auto camera_future = pool.submit_task(
        [camera_buffer_size, camera_buffer]() { threads::camera_thread_func(CAMERA_CONFIG_PATH, camera_buffer_size, camera_buffer); });

    // 启动串口线程
    auto serial_future = pool.submit_task([]() { threads::serial_thread_func(); });

    // 获取UserSerial实例用于读取IMU数据
    UserSerial &user_serial = UserSerial::getInstance();

    // 标定数据存储
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;

    MAS_LOG_INFO("HandEye Calibration Started");

    while (!g_shutdown.load())
    {
        // 检查退出请求
        {
            std::lock_guard<std::mutex> lock(calib_state.mutex);
            if (calib_state.quit_requested)
            {
                break;
            }
        }

        // 从队列获取图像
        const CameraFrame *frame = camera_buffer->front();
        if (!frame)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        cv::Mat image      = frame->frame.clone();
        auto    frame_time = frame->timestamp;
        camera_buffer->pop();

        // 获取IMU四元数
        Eigen::Quaterniond q = user_serial.q(frame_time);

        // 计算云台到世界的旋转
        Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
        Eigen::Matrix3d R_gimbal2world   = R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
        Eigen::Vector3d ypr              = rm_utils::eulers(R_gimbal2world, 2, 1, 0) * 57.3;

        // 检测标定板
        std::vector<cv::Point2f> centers_2d;
        bool                     board_detected = cv::findCirclesGrid(image, pattern_size, centers_2d);

        // 更新检测状态
        {
            std::lock_guard<std::mutex> lock(calib_state.mutex);
            calib_state.board_detected = board_detected;
        }

        // 处理保存请求
        bool should_save = false;
        {
            std::lock_guard<std::mutex> lock(calib_state.mutex);
            if (calib_state.save_requested)
            {
                should_save                = true;
                calib_state.save_requested = false;
            }
        }

        if (should_save && board_detected)
        {
            // PnP求解标定板位姿
            cv::Mat rvec, tvec;
            auto    centers_3d_ = centers_3d(pattern_size, center_distance_mm);
            cv::solvePnP(centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

            // 计算R_world2gimbal
            Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
            cv::Mat         R_world2gimbal_cv;
            cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
            cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);

            // 记录数据
            rvecs.emplace_back(rvec);
            tvecs.emplace_back(tvec);
            R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
            t_world2gimbal_list.emplace_back(t_world2gimbal);
        }
        else if (should_save && !board_detected)
        {
            MAS_LOG_WARN("Cannot save: no board detected");
        }

        // 处理清空请求
        bool should_clear = false;
        {
            std::lock_guard<std::mutex> lock(calib_state.mutex);
            if (calib_state.clear_requested)
            {
                should_clear                = true;
                calib_state.clear_requested = false;
            }
        }

        if (should_clear)
        {
            rvecs.clear();
            tvecs.clear();
            R_world2gimbal_list.clear();
            t_world2gimbal_list.clear();
            MAS_LOG_INFO("Cleared all calibration data");
        }

        // 准备显示文本
        std::vector<rm_utils::DisplayText> texts;
        texts.push_back({fmt::format("Data count: {}", rvecs.size()), 20, 40, 28, {0, 255, 0, 255}, 1});
        texts.push_back({fmt::format("YPR: {:.1f}, {:.1f}, {:.1f}", ypr[0], ypr[1], ypr[2]), 20, 80, 24, {0, 0, 255, 255}, 1});

        if (board_detected)
        {
            texts.push_back({"Board detected", 20, 120, 24, {0, 255, 0, 255}, 1});
        }
        else
        {
            texts.push_back({"No board detected", 20, 120, 24, {0, 0, 255, 255}, 1});
        }

        texts.push_back({"'s':save 'c':clear 'q':quit&calibrate", 20, 160, 24, {0, 255, 0, 255}, 1});

        // 在图像上绘制标定板角点
        if (board_detected)
        {
            cv::drawChessboardCorners(image, pattern_size, centers_2d, board_detected);
        }

        // 发送到显示
        display.display_add("HandEye Calibration", image, texts, {}, {}, 720, 540);
    }

    display.shutdown();
    g_shutdown = true;

    // 等待所有线程退出
    if (camera_future.valid())
    {
        camera_future.get();
    }
    if (serial_future.valid())
    {
        serial_future.get();
    }

    // 执行手眼标定
    if (rvecs.size() < 3)
    {
        MAS_LOG_ERROR("Calibration failed, need at least 3 data, got {}", rvecs.size());
        return -1;
    }

    cv::Mat R_gimbal2camera, t_gimbal2camera;
    cv::Mat R_world2board, t_world2board;

    cv::calibrateRobotWorldHandEye(rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board, R_gimbal2camera,
                                   t_gimbal2camera);

    t_gimbal2camera /= 1e3; // mm to m
    t_world2board /= 1e3;   // mm to m

    // 计算R_camera2gimbal和t_camera2gimbal
    cv::Mat R_camera2gimbal, t_camera2gimbal;
    cv::Mat R_board2world, t_board2world;
    cv::transpose(R_gimbal2camera, R_camera2gimbal);
    cv::transpose(R_world2board, R_board2world);
    t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
    t_board2world   = -R_board2world * t_world2board;

    // 计算相机同理想情况的偏角
    Eigen::Matrix3d R_camera2gimbal_eigen;
    cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
    Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
    Eigen::Vector3d camera_ypr     = rm_utils::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

    // 计算标定板到世界坐标系原点的水平距离
    auto x        = t_board2world.at<double>(0);
    auto y        = t_board2world.at<double>(1);
    auto distance = std::sqrt(x * x + y * y);

    // 计算标定板同竖直摆放时的偏角
    Eigen::Matrix3d R_board2world_eigen;
    cv::cv2eigen(R_board2world, R_board2world_eigen);
    Eigen::Vector3d board_ypr = rm_utils::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;

    // 保存结果
    YAML::Node config_out = YAML::LoadFile(CONFIG_PATH);

    std::vector<double> R_camera2gimbal_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
    std::vector<double> t_camera2gimbal_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

    config_out["handeye_calibration"]["R_camera2gimbal"] = R_camera2gimbal_data;
    config_out["handeye_calibration"]["t_camera2gimbal"] = t_camera2gimbal_data;
    config_out["handeye_calibration"]["R_camera2gimbal"].SetStyle(YAML::EmitterStyle::Flow);
    config_out["handeye_calibration"]["t_camera2gimbal"].SetStyle(YAML::EmitterStyle::Flow);

    std::ofstream fout(CONFIG_PATH);
    fout << config_out;

    MAS_LOG_INFO("Saved calibration result to {}", CONFIG_PATH);
    MAS_LOG_INFO("相机同理想情况的偏角: yaw={:.2f} pitch={:.2f} roll={:.2f} degree", camera_ypr[0], camera_ypr[1], camera_ypr[2]);
    MAS_LOG_INFO("标定板到世界坐标系原点的水平距离: {:.2f} m", distance);
    MAS_LOG_INFO("标定板同竖直摆放时的偏角: yaw={:.2f} pitch={:.2f} roll={:.2f} degree", board_ypr[0], board_ypr[1], board_ypr[2]);

    return 0;
}

} // namespace calibration
