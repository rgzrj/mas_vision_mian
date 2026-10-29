#include "armor_track.hpp"

#include "display.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <opencv2/core/mat.hpp>
#include <utility>

namespace auto_aim
{
ArmorTrack::ArmorTrack(const std::string &track_config_path, const std::string &pose_config_path)
    : armor_pose_(pose_config_path), last_timestamp_(std::chrono::steady_clock::now())
{
    try
    {
        // 加载YAML配置文件
        const YAML::Node config = YAML::LoadFile(track_config_path);
        const YAML::Node track  = config["auto_aim"]["armor_track"];

        debug_                       = track["armor_track_debug"].as<bool>(false);
        plotter_enable_              = track["plotter_enable"].as<bool>(false);
        min_detect_count_            = track["min_detect_count"].as<int>(5);
        max_temp_lost_count_         = track["max_temp_lost_count"].as<int>(25);
        outpost_max_temp_lost_count_ = track["outpost_max_temp_lost_count"].as<int>(75);

        MAS_LOG_INFO("armor_track yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }
}

ArmorTrack::~ArmorTrack() = default;

std::optional<Target> ArmorTrack::track(std::vector<Armor> &armors,
                                         std::chrono::steady_clock::time_point t,
                                        std::string window_name, cv::Mat bgr_img) noexcept
{
    const std::string state_before = state_;
    const auto dt = rm_utils::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    if (state_ != "lost" && dt > 0.1)
    {
        state_           = "lost";
        detect_count_    = 0;
        temp_lost_count_ = 0;
        reset_reason_    = "large_dt";
        target_.reset();
    }

    if (!armors.empty())
    {
        const cv::Point2f              img_center(bgr_img.cols / 2.0f, bgr_img.rows / 2.0f);
        const std::vector<std::string> priority{"1", "2", "3", "4", "5", "sentry", "outpost", "base"};

        // 获取装甲板优先级分数（越小优先级越高）
        const auto rank = [&](const Armor &armor) {
            const auto it = std::find(priority.begin(), priority.end(), armor.number);
            return it == priority.end() ? 100 : static_cast<int>(std::distance(priority.begin(), it));
        };
        std::sort(armors.begin(), armors.end(), [&](const Armor &lhs, const Armor &rhs) {
            const int lhs_rank = rank(lhs);
            const int rhs_rank = rank(rhs);
            if (lhs_rank != rhs_rank) return lhs_rank < rhs_rank;
            return cv::norm(lhs.center - img_center) < cv::norm(rhs.center - img_center);
        });
    }

    // 状态机处理
    bool found = false;
    if (state_ == "lost")
    {
        if (armors.empty()) reject_reason_ = "no_detection";
        else
        {
            pnp_attempt_count_ = 1;
            if (armor_pose_.GetArmorPose(armors.front()))
            {
                pnp_success_count_    = 1;
                raw_pnp_xyz_          = armors.front().xyz_in_world;
                raw_pnp_armor_yaw_    = armors.front().ypr_in_world[0];
                raw_pnp_valid_        = true;
                found                 = set_target(armors.front(), t);
                reject_reason_        = found ? "accepted_init" : "target_init_failed";
            }
            else reject_reason_ = "pnp_failed";
        }
    }
    else
    {
        // 非丢失状态下，更新已有目标
        found = update_target(armors, t);
    }

    state_machine(found);
    if (reset_reason_ == "none" && state_before == "detecting" && state_ == "lost" && !found)
        reset_reason_ = "detecting_miss";
    else if (reset_reason_ == "none" && state_before == "temp_lost" && state_ == "lost" && !found)
        reset_reason_ = "temp_lost_timeout";

    bool plot_emitted = false;

    // 发散检测：目标参数异常时重置
    if (state_ != "lost" && target_.has_value() && target_->diverged())
    {
        MAS_LOG_INFO("Target diverged!");
        state_        = "lost";
        reset_reason_ = "model_diverged";
        plotDiagnostics(armors, found, dt, state_before);
        plot_emitted = true;
        target_.reset();
    }

    if (debug_) showResult(armors, bgr_img, window_name);
    if (!plot_emitted) plotDiagnostics(armors, found, dt, state_before);
    if ((state_ == "tracking" || state_ == "temp_lost") && target_.has_value()) return target_;
    return std::nullopt;
}

void ArmorTrack::state_machine(bool found)
{
    if (state_ == "lost")
    {
        // 丢失状态：找到目标后进入检测状态
        if (!found) return;
        state_ = "detecting";
        detect_count_ = 1;
    }
    else if (state_ == "detecting")
    {
        // 检测状态：连续检测到足够次数后进入跟踪状态
        if (found)
        {
            detect_count_++;
            if (detect_count_ >= min_detect_count_) state_ = "tracking";
        }
        else
        {
            // 检测失败，回到丢失状态
            detect_count_ = 0;
            state_        = "lost";
            target_.reset();
        }
    }
    else if (state_ == "tracking")
    {
        // 跟踪状态：丢失目标后进入临时丢失状态
        if (found) return;
        temp_lost_count_ = 1;
        state_           = "temp_lost";
    }
    else if (state_ == "temp_lost")
    {
        // 临时丢失状态：重新找到目标则恢复跟踪，否则计数
        if (found)
        {
            state_ = "tracking";
            temp_lost_count_ = 0;
        }
        else
        {
            temp_lost_count_++;
            // 前哨站允许更长的丢失时间
            int max_lost = (target_.has_value() && target_->name == "outpost") ? outpost_max_temp_lost_count_ : max_temp_lost_count_;
            if (temp_lost_count_ > max_lost)
            {
                state_ = "lost";
                target_.reset();
            }
        }
    }
}

bool ArmorTrack::set_target(const Armor &armor, std::chrono::steady_clock::time_point t)
{
    double          radius;    // 旋转半径
    int             armor_num; // 装甲板数量
    Eigen::VectorXd P0_dig;    // 初始协方差矩阵对角线

    if (armor.number == "outpost")
    {
        // 前哨站：3块装甲板，半径0.2765m
        radius    = 0.2765;
        armor_num = 3;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    }
    else if (armor.number == "base")
    {
        // 基地：3块装甲板，半径0.3205m
        radius    = 0.3205;
        armor_num = 3;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    }
    else
    {
        // 普通车辆：4块装甲板，半径0.2m
        radius    = 0.2;
        armor_num = 4;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    }

    // 使用 Target 构造函数创建目标
    target_ = Target(armor, t, radius, radius, 0.0, armor_num, P0_dig);
    return true;
}

bool ArmorTrack::update_target(std::vector<Armor> &armors,
                               std::chrono::steady_clock::time_point t)
{
    if (!target_.has_value())
    {
        reject_reason_ = "no_target";
        return false;
    }
    // EKF预测
    target_->predict(t);

    matching_armor_count_ = static_cast<int>(std::count_if(
        armors.begin(), armors.end(), [&](const Armor &armor) {
            return armor.number == target_->name && armor.type == target_->armor_type;
        }));
    if (matching_armor_count_ == 0)
    {
        reject_reason_ = armors.empty() ? "no_detection" : "target_mismatch";
        return false;
    }

    Target trial = *target_;
    trial.is_switch = false;
    int successful_updates = 0;
    for (Armor &armor : armors)
    {
        if (armor.number != trial.name || armor.type != trial.armor_type) continue;
        ++pnp_attempt_count_;
        if (!armor_pose_.GetArmorPose(armor)) continue;
        ++pnp_success_count_;
        
        raw_pnp_xyz_       = armor.xyz_in_world;
        raw_pnp_armor_yaw_ = armor.ypr_in_world[0];
        raw_pnp_valid_     = true;

        if (trial.update(armor))
        {
            ++successful_updates;
            innovation_z_    = trial.last_innovation_z();
            innovation_norm_ = trial.last_innovation_norm();
        }
    }
    if (successful_updates == 0)
    {
        reject_reason_ = pnp_success_count_ == 0 ? "pnp_failed" : "pose_update_failed";
        return false;
    }
    target_               = std::move(trial);
    reject_reason_        = "accepted_pose";
    return true;
}

void ArmorTrack::plotDiagnostics(const std::vector<Armor> &armors, bool found, double dt,
                                 const std::string &state_before)
{
    if (!plotter_enable_) return;
    nlohmann::json data;
    data["implementation_stage"] = "tracker-stable-pnp-clean-v78";
    data["state"] = state_;
    data["state_before"] = state_before;
    data["reject_reason"] = reject_reason_;
    data["reset_reason"] = reset_reason_;
    data["dt_ms"] = dt * 1000.0;
    data["target_valid"] = target_.has_value();
    data["detected_armor_count"] = armors.size();
    data["matching_armor_count"] = matching_armor_count_;
    data["association_found"] = found;
    data["observation_mode"] = "pnp_pose";
    data["pnp_attempt_count"] = pnp_attempt_count_;
    data["pnp_success_count"] = pnp_success_count_;
    data["raw_pnp"]["valid"] = raw_pnp_valid_;
    data["raw_pnp"]["x"] = raw_pnp_xyz_.x();
    data["raw_pnp"]["y"] = raw_pnp_xyz_.y();
    data["raw_pnp"]["z"] = raw_pnp_xyz_.z();
    data["raw_pnp"]["armor_yaw"] = raw_pnp_armor_yaw_;
    data["innovation_z"] = innovation_z_;
    data["innovation_norm"] = innovation_norm_;
    data["target_name"] = target_ ? target_->name : "none";
    data["armor_type"] = target_ ? (target_->armor_type == ArmorType::BIG ? "BIG" : "SMALL")
                                  : "NONE";
    data["target_id"] = target_ ? target_->last_id : -1;
    data["update_count"] = target_ ? target_->update_count : 0;
    data["jumped"] = target_ && target_->jumped;
    data["is_switch"] = target_ && target_->is_switch;
    data["armor_num"] = target_ ? target_->armor_num : 0;
    data["predicted_armor_count"] = target_ ? target_->armor_xyza_list().size() : 0;

    Eigen::VectorXd x;
    if (target_) x = target_->ekf_x();
    data["x"] = x.size() >= 11 ? x[0] : 0.0;
    data["vx"] = x.size() >= 11 ? x[1] : 0.0;
    data["y"] = x.size() >= 11 ? x[2] : 0.0;
    data["vy"] = x.size() >= 11 ? x[3] : 0.0;
    data["z"] = x.size() >= 11 ? x[4] : 0.0;
    data["vz"] = x.size() >= 11 ? x[5] : 0.0;
    data["yaw"] = x.size() >= 11 ? x[6] : 0.0;
    data["w"] = x.size() >= 11 ? x[7] : 0.0;
    data["r"] = x.size() >= 11 ? x[8] : 0.0;
    data["l"] = x.size() >= 11 ? x[9] - x[8] : 0.0;
    data["radius_02"] = x.size() >= 11 ? x[8] : 0.0;
    data["radius_13"] = x.size() >= 11 ? x[9] : 0.0;
    data["h"] = x.size() >= 11 ? x[10] : 0.0;
    data["radius_group_02_updates"] = target_ ? target_->radius_group_visit_count(0) : 0;
    data["radius_group_13_updates"] = target_ ? target_->radius_group_visit_count(1) : 0;
    data["model_observations_sufficient"] = target_ && target_->model_observations_sufficient();
    data["geometry_committed"] = target_ && target_->geometry_committed();
    if (target_)
    {
        data["ekf_nis"] = target_->ekf().last_nis;
        data["ekf_update_applied"] = target_->ekf().last_update_applied;
    }
    plotter_.plot(data);
}

void ArmorTrack::showResult(const std::vector<Armor> &armors, const cv::Mat &bgr_img,
                            std::string window_name) const noexcept
{
    if (!debug_) return;
    if (bgr_img.empty())
    {
        MAS_LOG_WARN("Invalid input image for showResult");
        return;
    }

    // 调用位姿解算器的debug显示
    if (armor_pose_.isDebug())
    {
        armor_pose_.showPoseDebug(armors, bgr_img, window_name + "_pose");
    }

    auto &display = rm_utils::Display::getInstance();

    // 计算FPS
    auto &fps       = fps_map_[window_name];
    auto &count     = count_map_[window_name];
    auto &last_time = last_time_map_[window_name];
    ++count;

    const auto now     = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    if (elapsed >= 1000)
    {
        fps       = count * 1000.0 / elapsed;
        count     = 0;
        last_time = now;
    }

    std::vector<rm_utils::DisplayText> texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
    rm_utils::DisplayText status;
    status.content = "FPS: " + std::to_string(static_cast<int>(fps)) +
                     " Armors: " + std::to_string(armors.size()) + " State: " + state_;
    status.x = 20; status.y = 20; status.size = 24;
    status.color = {255, 255, 0, 255}; status.thickness = 2;
    texts.push_back(status);

    if (target_)
    {
        const Eigen::VectorXd x = target_->ekf_x();
        rm_utils::DisplayText model;
        model.content = "Target: " + target_->name + " ID: " + std::to_string(target_->last_id) +
                        " w: " + std::to_string(x[7]).substr(0, 6) +
                        " r: " + std::to_string(x[8]).substr(0, 5) + "/" +
                        std::to_string(x[9]).substr(0, 5) +
                        " h: " + std::to_string(x[10]).substr(0, 5);
        model.x = 20; model.y = 50; model.size = 20;
        model.color = {0, 255, 0, 255}; model.thickness = 1;
        texts.push_back(model);

        const auto predicted = target_->armor_xyza_list();
        for (int id = 0; id < static_cast<int>(predicted.size()); ++id)
        {
            const auto projected = armor_pose_.reproject_armor(
                predicted[id].head<3>(), predicted[id][3], target_->armor_type, target_->name);
            if (projected.size() != 4) continue;
            const bool tracked = id == target_->last_id;
            const SDL_Color color = tracked ? SDL_Color{0, 255, 0, 255} : SDL_Color{255, 0, 0, 255};
            for (int i = 0; i < 4; ++i)
            {
                rm_utils::DisplayLine line;
                line.x1 = static_cast<int>(projected[i].x);
                line.y1 = static_cast<int>(projected[i].y);
                line.x2 = static_cast<int>(projected[(i + 1) % 4].x);
                line.y2 = static_cast<int>(projected[(i + 1) % 4].y);
                line.thickness = tracked ? 3 : 2;
                line.color = color;
                lines.push_back(line);
            }
            if (tracked)
            {
                for (const auto diagonal : {std::pair<int, int>{0, 2}, {1, 3}})
                {
                    rm_utils::DisplayLine line;
                    line.x1 = static_cast<int>(projected[diagonal.first].x);
                    line.y1 = static_cast<int>(projected[diagonal.first].y);
                    line.x2 = static_cast<int>(projected[diagonal.second].x);
                    line.y2 = static_cast<int>(projected[diagonal.second].y);
                    line.thickness = 2; line.color = color;
                    lines.push_back(line);
                }
            }
            rm_utils::DisplayText label;
            label.content = "M" + std::to_string(id);
            label.x = static_cast<int>(projected[0].x);
            label.y = static_cast<int>(projected[0].y) - 5;
            label.size = 24; label.color = {255, 255, 0, 255}; label.thickness = 1;
            texts.push_back(label);
        }

        const std::vector<cv::Point3f> center{{static_cast<float>(x[0]),
                                                static_cast<float>(x[2]),
                                                static_cast<float>(x[4])}};
        const auto center_pixel = armor_pose_.world2pixel(center);
        if (!center_pixel.empty())
        {
            rm_utils::DisplayPoint point;
            point.x = static_cast<int>(center_pixel[0].x);
            point.y = static_cast<int>(center_pixel[0].y);
            point.size = 15; point.color = {255, 0, 255, 255};
            points.push_back(point);
        }
    }

    for (const Armor &armor : armors)
    {
        rm_utils::DisplayText label;
        label.content = "[" + armor.number + "]: [" + std::to_string(armor.confidence).substr(0, 5) + "]";
        label.x = static_cast<int>(armor.center.x);
        label.y = static_cast<int>(armor.center.y);
        label.size = 28; label.color = {255, 255, 255, 255}; label.thickness = 1;
        texts.push_back(label);
    }
    display.display_add(window_name, bgr_img, texts, points, lines);
}

} // namespace auto_aim
