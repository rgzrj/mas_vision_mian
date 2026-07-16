#ifndef _ARMOR_TRACK_H_
#define _ARMOR_TRACK_H_

#include "armor_track/armor_pose.hpp"
#include "armor_track/armor_target.hpp"
#include <Eigen/Dense>
#include <optional>
#include <opencv2/core/mat.hpp>
#include <string>

#include "plotter.hpp"

namespace auto_aim
{

class ArmorTrack
{
  public:
    /**
     * @brief 构造函数
     * @param track_config_path 跟踪配置文件路径
     * @param pose_config_path 位姿配置文件路径
     */
    ArmorTrack(const std::string &track_config_path = "config/auto_aim.yaml", const std::string &pose_config_path = "config/hikcamera.yaml");
    ~ArmorTrack();

    /**
     * @brief 获取位姿解算器
     * @return 位姿解算器引用
     */
    ArmorPose &armor_pose() noexcept { return armor_pose_; }

    /**
     * @brief 获取当前跟踪状态
     * @return 状态字符串 (lost/detecting/tracking/temp_lost)
     */
    std::string state() const { return state_; }

    /**
     * @brief 主跟踪函数
     * @param armors 检测到的装甲板列表
     * @param t 当前时间戳
     * @param window_name 窗口名称（用于debug显示）
     * @param bgr_img 图像（用于debug显示）
     * @return 当前锁定目标，tracking/temp_lost 状态下有值，否则 nullopt
     */
    std::optional<Target> track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t, std::string window_name = "",
                                cv::Mat bgr_img = cv::Mat()) noexcept;

  private:
    /**
     * @brief 显示跟踪结果（debug模式）
     */
    void showResult(const std::vector<Armor> &armors, const cv::Mat &bgr_img, std::string window_name) const noexcept;

    // 装甲板姿态解算器
    ArmorPose armor_pose_;
    // 当前跟踪目标
    std::optional<Target> target_;

    // YAML 配置参数
    bool                                  debug_;                       // debug开关
    int                                   min_detect_count_;            // 最小检测次数
    int                                   max_temp_lost_count_;         // 最大临时丢失次数
    int                                   detect_count_;                // 当前检测次数
    int                                   temp_lost_count_;             // 当前临时丢失次数
    int                                   outpost_max_temp_lost_count_; // 前哨站最大丢失次数
    int                                   normal_temp_lost_count_;      // 普通目标最大丢失次数
    std::string                           state_, pre_state_;           // 当前/上一状态
    std::chrono::steady_clock::time_point last_timestamp_;              // 上一帧时间戳

    /**
     * @brief 状态机处理
     * @param found 是否找到目标
     */
    void state_machine(bool found);

    /**
     * @brief 设置新目标
     * @param armor 初始装甲板
     * @param t 时间戳
     * @return 是否成功
     */
    bool set_target(const Armor &armor, std::chrono::steady_clock::time_point t);

    /**
     * @brief 更新目标
     * @param armors 检测到的装甲板列表
     * @param t 时间戳
     * @return 是否成功
     */
    bool update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

    // Debug相关
    mutable std::map<std::string, double>                                fps_map_;       // FPS计算
    mutable std::map<std::string, int>                                   count_map_;     // 帧计数
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_; // 上次时间

    // Plotter for debug
    rm_utils::Plotter plotter_;
};
} // namespace auto_aim
#endif // _ARMOR_TRACK_H_