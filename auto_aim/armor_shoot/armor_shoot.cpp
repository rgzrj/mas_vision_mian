#include "armor_shoot.hpp"

#include "display.hpp"
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#include "mas_log.hpp"
#include "math_tools.hpp"
#include "trajectory.hpp"
#include "plotter.hpp"

namespace auto_aim
{

ArmorShoot::ArmorShoot(const std::string &config_path)
{
    try
    {
        YAML::Node config     = YAML::LoadFile(config_path);
        auto       node       = config["auto_aim"]["armor_shoot"];
        yaw_offset_           = node["yaw_offset"].as<double>(0.0) / 57.3;
        pitch_offset_         = node["pitch_offset"].as<double>(0.0) / 57.3;
        fire_delay_time_      = node["fire_delay_time"].as<double>(0.02);
        yaw_tolerance_near_   = node["yaw_tolerance_near"].as<double>(2.0) / 57.3;
        yaw_tolerance_far_    = node["yaw_tolerance_far"].as<double>(1.0) / 57.3;
        pitch_tolerance_near_ = node["pitch_tolerance_near"].as<double>(2.0) / 57.3;
        pitch_tolerance_far_  = node["pitch_tolerance_far"].as<double>(1.0) / 57.3;
        bullet_speed_         = node["bullet_speed"].as<double>(25.0);
        gravity_              = node["gravity"].as<double>(9.81);
        air_resistance_       = node["resistance"].as<double>(0.003);
        debug_                = node["debug"].as<bool>(false);
        // plotter 配置
        plotter_enable_ = node["plotter_enable"].as<bool>(false);
        if (plotter_enable_) plotter_ = std::make_unique<rm_utils::Plotter>();

        MAS_LOG_INFO("armor_shoot yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("armor_shoot: Failed to load config, using defaults: {}", e.what());
    }
}

ArmorShoot::~ArmorShoot() = default;

ShootResult ArmorShoot::shoot(const std::optional<Target> &target, std::chrono::steady_clock::time_point timestamp,
                             const Eigen::Matrix3d &R_gimbal2world, const cv::Mat &bgr_img, const std::string &window_name)
{
    // 从旋转矩阵提取云台欧拉角（yaw, pitch）
    Eigen::Vector3d gimbal_euler = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
    double          gimbal_yaw   = gimbal_euler[0];
    double          gimbal_pitch = gimbal_euler[1];

    // 无目标情况处理
    if (!target.has_value())
    {
        AimPoint aim_point = {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK};
        if (debug_) showDebug(target, aim_point, gimbal_yaw, gimbal_pitch, 0.0f, 0.0f, 0, bgr_img, window_name);
        return {};
    }

    // 总延迟 = 处理延迟 + 发弹延迟
    double process_delay = rm_utils::delta_time(std::chrono::steady_clock::now(), timestamp);
    double dt            = process_delay + fire_delay_time_;

    auto future = timestamp + std::chrono::microseconds(static_cast<int>(dt * 1e6));

    // 使用目标中心估算初始飞行时间，正式选板放在预计命中时刻执行。
    Target predicted_target = *target;
    predicted_target.predict(future);
    const auto x = predicted_target.ekf_x();

    // 选择初始瞄准点
    const double d0 = std::hypot(x[0], x[2]);
    rm_utils::Trajectory trajectory0(bullet_speed_, d0, x[4], gravity_, air_resistance_);

    if (trajectory0.unsolvable)
    {
        debug_aim_point.valid = false;
        AimPoint aim_point    = {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK};
        if (debug_) showDebug(target, aim_point, gimbal_yaw, gimbal_pitch, 0.0f, 0.0f, 0, bgr_img, window_name);
        return {};
    }

// 迭代求解飞行时间 (最多 10 次，收敛条件：相邻两次 fly_time 差 < 0.001)
    bool                converged     = false;
    double              prev_fly_time = trajectory0.fly_time;

    for (int iter = 0; iter < 10; ++iter)
    {
        Target iteration_target = *target;
        // 预测目标在 future + prev_fly_time 时刻的位置
        auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
        iteration_target.predict(predict_time);

        // 计算瞄准点
        auto aim_point_iter = chooseAimPoint(iteration_target);
        debug_aim_point     = aim_point_iter;

        if (!aim_point_iter.valid)
        {
            AimPoint aim_point = {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK};
            if (debug_) showDebug(target, aim_point, gimbal_yaw, gimbal_pitch, 0.0f, 0.0f, 0, bgr_img, window_name);
            return {};
        }

        // 计算新弹道
        Eigen::Vector3d      xyz = aim_point_iter.xyza.head(3);
        double               d   = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
        rm_utils::Trajectory current_traj(bullet_speed_, d, xyz.z(), gravity_, air_resistance_);

        // 检查弹道是否可解
        if (current_traj.unsolvable)
        {
            MAS_LOG_DEBUG("Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1, bullet_speed_, d, xyz.z());
            debug_aim_point.valid = false;
            AimPoint aim_point    = {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK};
            if (debug_) showDebug(target, aim_point, gimbal_yaw, gimbal_pitch, 0.0f, 0.0f, 0, bgr_img, window_name);
            return {};
        }

        // 检查收敛条件
        if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001)
        {
            converged = true;
            break;
        }
        prev_fly_time = current_traj.fly_time;
    }

    // 计算最终角度
    Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
    double          yaw_raw   = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
    // 初始化：如果是第一次运行，或者相差太离谱，用当前云台角度初始化基准，这里的 3.0 (弧度) 是一个阈值，约 171 度，防止刚启动时转一大圈
    if (std::abs(last_target_yaw_ - gimbal_yaw) > 3.0) {
        last_target_yaw_ = gimbal_yaw;
    }
    // 计算原始目标角度与上一次目标角度的差值
    double yaw_delta = yaw_raw - last_target_yaw_;
    // 将差值归一化到 [-π, π] 范围内
    yaw_delta = rm_utils::limit_rad(yaw_delta);
    // 基于上一次目标角度 + 归一化差值，得到连续的目标角度
    double yaw = last_target_yaw_ + yaw_delta;
    // 更新状态变量
    last_target_yaw_ = yaw;

    double pitch = -(rm_utils::Trajectory(bullet_speed_, final_xyz.head<2>().norm(), 
                                            final_xyz.z(), gravity_, air_resistance_).pitch + pitch_offset_);

    float dist = static_cast<float>(final_xyz.head<2>().norm());

    // 射击决策，只有解算成功且在阈值内才允许开火
    double yaw_tolerance   = dist < 3.0 ? yaw_tolerance_near_ : yaw_tolerance_far_;
    double pitch_tolerance = dist < 3.0 ? pitch_tolerance_near_ : pitch_tolerance_far_;

    uint8_t fire = 0;
    if (converged && debug_aim_point.valid && debug_aim_point.fire_allowed)
    {
        bool yaw_ok   = std::abs(rm_utils::limit_rad(gimbal_yaw - yaw)) < yaw_tolerance;
        bool pitch_ok = std::abs(gimbal_pitch - pitch) < pitch_tolerance;
        if (yaw_ok && pitch_ok) fire = 1;
    }

    // Plotter 输出
    if (plotter_enable_ && plotter_)
    {
        nlohmann::json data;

        // ShootResult 数据
        data["ShootResult"]["found"]        = 1;
        data["ShootResult"]["fire_advice"]  = fire;
        data["ShootResult"]["target_yaw"]   = yaw * 57.3f;   // 转换为度
        data["ShootResult"]["target_pitch"] = pitch * 57.3f; // 转换为度

        // 当前云台姿态（yaw, pitch, roll）
        data["gimbal"]["yaw"]   = gimbal_yaw * 57.3;      // 转换为度
        data["gimbal"]["pitch"] = gimbal_pitch * 57.3;    // 转换为度
        data["gimbal"]["roll"]  = gimbal_euler[2] * 57.3; // 转换为度

        // 距离信息
        data["distance"] = dist;
        // aim point
        data["aim_point"]["x"] = final_xyz.x();
        data["aim_point"]["y"] = final_xyz.y();
        data["aim_point"]["z"] = final_xyz.z();

        plotter_->plot(data);
    }

    if (debug_)
    {
        showDebug(target, debug_aim_point, gimbal_yaw, gimbal_pitch, yaw, pitch, fire, bgr_img, window_name);
    }

    // 返回结果
    ShootResult result;
    result.found        = true;
    result.fire_advice  = fire;
    result.target_yaw   = yaw;
    result.target_pitch = pitch;
    return result;
}

AimPoint ArmorShoot::chooseAimPoint(const Target &target)
{
    const auto x          = target.ekf_x();
    const auto armors     = target.armor_xyza_list();
    const auto omega      = std::abs(x[7]);
    const auto center_yaw = std::atan2(x[2], x[0]);

    // 异常情况：无装甲板数据
    if (armors.empty())
    {
        lock_id_ = -1; // 重置锁定的 ID
        return {false, Eigen::Vector4d::Zero(), false, -1, AimMode::TRACK};
    }

    if (lock_id_ < 0 || lock_id_ >= static_cast<int>(armors.size()))
    {
        MAS_LOG_DEBUG("Invalid lock_id_ {} reset to -1 (armor_num: {})", lock_id_, armors.size());
        lock_id_ = -1;
    }
    // 默认打击装甲板：列表中的第一个装甲板
    int             default_id    = 0;
    Eigen::Vector4d default_point = armors[0];

    // 初始状态或未跳变：直接打击默认装甲板
    if (!target.jumped)
    {
        return {true, default_point, true, default_id, AimMode::TRACK};
    }

    // 预计算所有装甲板相对中心的夹角
    std::vector<double> delta_angles;
    for (const auto &armor : armors)
    {
        delta_angles.emplace_back(rm_utils::limit_rad(armor[3] - center_yaw));
    }

    // 决策逻辑

    // 高速旋转模式下采用CENTER模式
    if (omega >= spinning_threshold_high_ )
    {
        // M0/M2 共用 x[4] 高度；CENTER 模式不依赖 lock_id_
        const double h = x[4];
        // yaw 固定瞄准旋转中心，pitch 固定瞄准 M0/M2 高度
        Eigen::Vector4d center_point(x[0], x[2], h, center_yaw);
        const double dist = std::hypot(x[0], x[2]);

        // 设定近距离阈值和远距离阈值，单位：度
        constexpr double angle_near = 60.0; // 近距离窗口
        constexpr double angle_far  = 20.0; // 远距离窗口
        constexpr double dist_set   = 3.0;  // 近距离定义
        constexpr double dist_far   = 5.0;  // 远距离定义

        double fire_window_angle_deg = 0.0;

        // 线性插值计算
        if (dist <= dist_set)
        {
            fire_window_angle_deg = angle_near;
        }
        else if(dist <= dist_far)
        {
            // 3～5 m 从 60°线性减小到20°
            const double ratio    = (dist - dist_set) / (dist_far - dist_set);
            fire_window_angle_deg = angle_near + ratio * (angle_far - angle_near);        
        }
        else
        {
            fire_window_angle_deg = angle_far;
        }

        // 转换为弧度
        double fire_window_angle = fire_window_angle_deg / 57.3;

        // 主动预判开火逻辑，检查预测时刻是否有装甲板位于计算出的窗口内
        bool fire_allowed = false;

        // 只检查偶数高度组：M0、M2
        for (int id = 0; id < static_cast<int>(armors.size()); id += 2)        
        {
            // delta_angles[i] 是预测时刻装甲板相对于中心的角度差
            if (std::abs(delta_angles[id]) < fire_window_angle)
            {
                fire_allowed = true;
                break; // 只要有一块板满足条件即可
            }
        }

        lock_id_ = -1; // 中心瞄准模式不锁定特定装甲板
        return {true, center_point, fire_allowed, -1, AimMode::CENTER};
    }
    // 跟踪模式,角速度低于低阈值（近似静止或低速旋转）锁定并跟踪可见装甲板
    else if (omega <= spinning_threshold_low_)
    {
        // 筛选视野内的装甲板（±60 度范围内）
        constexpr double VISIBLE_ANGLE_THRESHOLD = 60.0 / 57.3;
        std::vector<int> visible_ids;
        for (size_t i = 0; i < armors.size(); ++i)
        {
            if (std::abs(delta_angles[i]) < VISIBLE_ANGLE_THRESHOLD)
            {
                visible_ids.push_back(static_cast<int>(i));
            }
        }

        // 选择逻辑
        int selected_id = -1;
        if (!visible_ids.empty())
        {
            // 锁定逻辑：优先保持当前锁定的装甲板
            if (visible_ids.size() > 1)
            {
                int id0 = visible_ids[0];
                int id1 = visible_ids[1];
                if (lock_id_ != id0 && lock_id_ != id1)
                {
                    // 选择夹角更小的装甲板
                    lock_id_ = (std::abs(delta_angles[id0]) < std::abs(delta_angles[id1])) ? id0 : id1;
                }
            }
            else
            {
                // 只有当前锁定的不在可见列表中才更新
                if (lock_id_ == -1 || std::find(visible_ids.begin(), visible_ids.end(), lock_id_) == visible_ids.end())
                {
                    lock_id_ = visible_ids[0];
                }
            }
            selected_id = lock_id_;
        }

        // 默认打击装甲板：没找到视野内的，直接用默认的
        if (selected_id == -1) selected_id = default_id;

        return {true, armors[selected_id], true, selected_id, AimMode::TRACK};
    }

    // 中速模式，条件：角速度介于低阈值和高阈值之间，策略：选择正在靠近的装甲板打击
    else
    {
        double coming_ang  = (target.name == "outpost") ? (70.0 / 57.3) : comming_angle_;
        double leaving_ang = (target.name == "outpost") ? (30.0 / 57.3) : leaving_angle_;
        int    selected_id = -1;

        // 尝试寻找"正在靠近"的板
        for (size_t i = 0; i < armors.size(); ++i)
        {
            if (std::abs(delta_angles[i]) > coming_ang) continue;
            if (x[7] > 0 && delta_angles[i] < leaving_ang)
            {
                selected_id = i;
                break;
            }
            if (x[7] < 0 && delta_angles[i] > -leaving_ang)
            {
                selected_id = i;
                break;
            }
        }

        // 防抖动优化：如果找不到合适的板或者当前有锁定的板，优先保持连续性(针对outpost模式)
        if (selected_id == -1)
        {
            // 如果没有找到"正在靠近"的板，检查是否有视野内的板
            constexpr double VISIBLE_ANGLE_THRESHOLD = 60.0 / 57.3;
            std::vector<int> visible_ids;
            for (size_t i = 0; i < armors.size(); ++i)
            {
                if (std::abs(delta_angles[i]) < VISIBLE_ANGLE_THRESHOLD)
                {
                    visible_ids.push_back(static_cast<int>(i));
                }
            }

            // 优先选择之前锁定的板（如果有）
            if (lock_id_ != -1 && std::find(visible_ids.begin(), visible_ids.end(), lock_id_) != visible_ids.end())
            {
                selected_id = lock_id_;
            }
            else
            {
                // 选择夹角最小的
                int    best_id   = visible_ids[0];
                double min_angle = std::abs(delta_angles[best_id]);
                for (int id : visible_ids)
                {
                    double angle = std::abs(delta_angles[id]);
                    if (angle < min_angle)
                    {
                        min_angle = angle;
                        best_id   = id;
                    }
                }
                selected_id = best_id;
            }
        }
        else
        {
            // 找到了正在靠近的板，更新 lock_id_
            lock_id_ = selected_id;
        }

        return {true, armors[selected_id], true, selected_id, AimMode::COMING};
    }
}

void ArmorShoot::showDebug(const std::optional<Target> &target, const AimPoint &aim_point, double gimbal_yaw, double gimbal_pitch, float target_yaw,
                           float target_pitch, uint8_t fire, const cv::Mat &bgr_img, const std::string &window_name) const noexcept
{
    if (bgr_img.empty() || window_name.empty()) return;

    auto &display   = rm_utils::Display::getInstance();
    auto &fps       = fps_map_[window_name];
    auto &count     = count_map_[window_name];
    auto &last_time = last_time_map_[window_name];

    // FPS 统计
    count++;
    auto now      = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    if (duration >= 1000)
    {
        fps       = count * 1000.0 / duration;
        count     = 0;
        last_time = now;
    }

    std::vector<rm_utils::DisplayText>  texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine>  lines;

    // 1. FPS 信息
    rm_utils::DisplayText t_fps;
    t_fps.content = "FPS: " + std::to_string(static_cast<int>(fps));
    t_fps.x       = 10;
    t_fps.y       = 10;
    t_fps.size    = 22;
    t_fps.color   = {255, 255, 0, 255};
    texts.push_back(t_fps);

    // 云台当前角度
    rm_utils::DisplayText t_gimbal;
    t_gimbal.content =
        "Gimbal  yaw:" + std::to_string(gimbal_yaw * 57.3f).substr(0, 6) + "  pitch:" + std::to_string(gimbal_pitch * 57.3f).substr(0, 6) + " deg";
    t_gimbal.x     = 10;
    t_gimbal.y     = 36;
    t_gimbal.size  = 32;
    t_gimbal.color = {0, 255, 255, 255};
    texts.push_back(t_gimbal);

    // 无目标情况
    if (!target.has_value())
    {
        rm_utils::DisplayText t_no;
        t_no.content = "No Target";
        t_no.x       = 10;
        t_no.y       = 36;
        t_no.size    = 32;
        t_no.color   = {200, 200, 200, 255};
        texts.push_back(t_no);
        display.display_add(window_name, bgr_img, texts, points, lines);
        return;
    }

    const Eigen::VectorXd &x = target->ekf().x;

    // 目标角度（解算结果）
    rm_utils::DisplayText t_target;
    t_target.content =
        "Target  yaw:" + std::to_string(target_yaw * 57.3f).substr(0, 6) + "  pitch:" + std::to_string(target_pitch * 57.3f).substr(0, 6) + " deg";
    t_target.x     = 10;
    t_target.y     = 70;
    t_target.size  = 32;
    t_target.color = {0, 255, 0, 255};
    texts.push_back(t_target);

    // 角度偏差
    rm_utils::DisplayText t_diff;
    double                diff_yaw   = (target_yaw - gimbal_yaw) * 57.3;
    double                diff_pitch = (target_pitch - gimbal_pitch) * 57.3;
    t_diff.content                   = "Diff_yaw:" + std::to_string(diff_yaw).substr(0, std::to_string(diff_yaw).find('.') + 3) +
                     "Diff_pitch:" + std::to_string(diff_pitch).substr(0, std::to_string(diff_pitch).find('.') + 3) + " deg";
    t_diff.x     = 10;
    t_diff.y     = 104;
    t_diff.size  = 32;
    t_diff.color = {255, 200, 100, 255};
    texts.push_back(t_diff);

    // 瞄准点模式和信息
    std::string mode_str;
    switch (aim_point.mode)
    {
    case AimMode::TRACK:
        mode_str = "TRACK";
        break;
    case AimMode::CENTER:
        mode_str = "CENTER";
        break;
    case AimMode::COMING:
        mode_str = "COMING";
        break;
    }

    rm_utils::DisplayText t_info;
    t_info.content = "Name:" + target->name + "  armor_num:" + std::to_string(target->armor_num) + "  omega:" + std::to_string(x[7]).substr(0, 5) +
                     "  fire:" + std::to_string(fire) + "  " + mode_str;
    t_info.x     = 10;
    t_info.y     = 144;
    t_info.size  = 32;
    t_info.color = {255, 150, 0, 255};
    texts.push_back(t_info);

    // 距离和弹道信息
    double                dist = std::sqrt(aim_point.xyza[0] * aim_point.xyza[0] + aim_point.xyza[1] * aim_point.xyza[1]);
    rm_utils::DisplayText t_dist;
    t_dist.content = "Dist:" + std::to_string(dist).substr(0, 5) + "m  " + "Valid:" + std::string(aim_point.valid ? "YES" : "NO") + "  " +
                     "ArmorID:" + std::to_string(aim_point.armor_id);
    t_dist.x     = 10;
    t_dist.y     = 170;
    t_dist.size  = 28;
    t_dist.color = aim_point.valid ? SDL_Color{0, 255, 0, 255} : SDL_Color{0, 0, 255, 255};
    texts.push_back(t_dist);

    display.display_add(window_name, bgr_img, texts, points, lines);
}

} // namespace auto_aim
