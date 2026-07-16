#include "calibrate_camera.hpp"

#include <BS_thread_pool.hpp>
#include <fmt/core.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "SPSCQueue.h"
#include "camera_thread.hpp"
#include "display.hpp"
#include "mas_log.hpp"

extern std::atomic<bool> g_shutdown;

// YAML配置文件路径
constexpr const char *CONFIG_PATH = "config/hikcamera.yaml";

// 生成标定板3D点
std::vector<cv::Point3f> centers_3d(const cv::Size &pattern_size, float center_distance)
{
    std::vector<cv::Point3f> centers_3d;
    for (int i = 0; i < pattern_size.height; i++)
    {
        for (int j = 0; j < pattern_size.width; j++)
        {
            centers_3d.push_back({j * center_distance, i * center_distance, 0});
        }
    }
    return centers_3d;
}

// SDL事件回调函数
struct CalibrationState
{
    bool       save_requested = false;
    bool       quit_requested = false;
    std::mutex mutex;
};

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
        case SDLK_ESCAPE:
            state->quit_requested = true;
            return true;
        }
    }
    return false;
}

int calibrate_camera_main()
{
    MAS_LOG_INFO("Camera Calibration Start");

    // 读取配置
    YAML::Node config             = YAML::LoadFile(CONFIG_PATH);
    int        pattern_cols       = config["calibration"]["pattern_cols"].as<int>();
    int        pattern_rows       = config["calibration"]["pattern_rows"].as<int>();
    double     center_distance_mm = config["calibration"]["center_distance_mm"].as<double>();
    cv::Size   pattern_size(pattern_cols, pattern_rows);

    // 创建相机队列和线程池
    size_t          camera_buffer_size = 2;
    auto            camera_buffer      = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(camera_buffer_size);
    BS::thread_pool pool(std::thread::hardware_concurrency());

    // 启动显示线程
    rm_utils::Display &display = rm_utils::Display::getInstance();
    display.start(pool);

    // 设置校准状态和回调
    CalibrationState calib_state;
    display.set_event_callback(sdl_event_callback, &calib_state);

    // 启动相机线程
    auto camera_future =
        pool.submit_task([camera_buffer_size, camera_buffer]() { threads::camera_thread_func(CONFIG_PATH, camera_buffer_size, camera_buffer); });

    // 标定数据存储
    cv::Size                              img_size;
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;

    int       saved_images = 0;
    const int min_images   = 10;

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
        cv::Mat img = frame->frame.clone();
        camera_buffer->pop();

        if (img.empty()) continue;

        img_size = img.size();

        // 检测标定板
        std::vector<cv::Point2f> centers_2d;
        bool                     success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);

        // 在图像上绘制角点
        if (success)
        {
            cv::drawChessboardCorners(img, pattern_size, centers_2d, success);
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

        if (should_save && success)
        {
            img_points.emplace_back(centers_2d);
            obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
            saved_images++;
            MAS_LOG_INFO("Saved image {}/{}", saved_images, min_images);
        }
        else if (should_save && !success)
        {
            MAS_LOG_WARN("Cannot save: no board detected");
        }

        // 准备显示文本
        std::vector<rm_utils::DisplayText> texts;
        texts.push_back({fmt::format("Images: {}/{}", saved_images, min_images), 20, 40, 28, {0, 255, 0, 255}, 1});

        if (success)
        {
            texts.push_back({"Board detected", 20, 80, 24, {0, 255, 0, 255}, 1});
        }
        else
        {
            texts.push_back({"No board detected", 20, 80, 24, {0, 0, 255, 255}, 1});
        }

        texts.push_back({"'s':save, ESC:exit", 20, 120, 24, {0, 255, 0, 255}, 1});

        // 发送到显示
        display.display_add("Camera Calibration", img, texts, {}, {}, 720, 540);
    }

    display.shutdown();
    g_shutdown = true;

    // 等待相机线程退出
    if (camera_future.valid())
    {
        camera_future.get();
    }

    // 检查数据量
    if (saved_images < min_images)
    {
        MAS_LOG_ERROR("Not enough images, need at least {}", min_images);
        return -1;
    }

    // 执行相机标定
    MAS_LOG_INFO("Starting camera calibration with {} images", saved_images);

    cv::Mat              camera_matrix, distort_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    auto                 criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
    cv::calibrateCamera(obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3, criteria);

    // 计算重投影误差
    double error_sum    = 0;
    size_t total_points = 0;
    for (size_t i = 0; i < obj_points.size(); i++)
    {
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

        total_points += reprojected_points.size();
        for (size_t j = 0; j < reprojected_points.size(); j++)
        {
            error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
        }
    }
    double error = error_sum / total_points;

    MAS_LOG_INFO("Calibration completed! Error: {:.4f} pixels", error);

    // 保存到配置文件
    YAML::Node config_out = YAML::LoadFile(CONFIG_PATH);

    std::vector<double> camera_matrix_data(camera_matrix.begin<double>(), camera_matrix.end<double>());
    std::vector<double> distort_coeffs_data(distort_coeffs.begin<double>(), distort_coeffs.end<double>());

    config_out["calibration"]["camera_matrix"]      = camera_matrix_data;
    config_out["calibration"]["distort_coeffs"]     = distort_coeffs_data;
    config_out["calibration"]["reprojection_error"] = error;

    config_out["calibration"]["camera_matrix"].SetStyle(YAML::EmitterStyle::Flow);
    config_out["calibration"]["distort_coeffs"].SetStyle(YAML::EmitterStyle::Flow);

    std::ofstream fout(CONFIG_PATH);
    fout << config_out;
    fout.close();

    MAS_LOG_INFO("Saved calibration result to {}", CONFIG_PATH);

    return 0;
}
