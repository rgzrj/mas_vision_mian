#ifndef _ARMOR_SHOOT_H_
#define _ARMOR_SHOOT_H_

#include "armor_target.hpp"
#include "plotter.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <string>

namespace auto_aim
{
enum class AimMode
{
    TRACK,     ///< 跟踪当前装甲板
    CENTER,    ///< 高速时击打旋转中心
    COMING     ///< 来板模式：选择正在靠近的装甲板
};

/**
 * @brief 瞄准点结构
 */
struct AimPoint
{
    bool            valid;        ///< 是否有效
    Eigen::Vector4d xyza;         ///< 瞄准点坐标 [x, y, z, yaw]
    bool            fire_allowed; ///< 是否允许开火
    int             armor_id;     ///< 目标装甲板 ID
    AimMode         mode;         ///< 使用的瞄准模式
};

struct ShootResult
{
    bool    found        = false;
    uint8_t fire_advice  = 0;
    float   target_yaw   = 0.0F;
    float   target_pitch = 0.0F;
};

class ArmorShoot
{
  public:
    explicit ArmorShoot(const std::string &config_path = "config/auto_aim.yaml");
    ~ArmorShoot();

    /**
     * @brief 计算发射指令
     * @param target 当前目标
     * @param timestamp 当前时间戳
     * @param R_gimbal2world 云台到世界坐标系的旋转
     * @param bgr_img 当前帧 BGR 图像（可选）
     * @param window_name 图像调试窗口名称（可选）
     * @return 发射计算结果
     * @details 根据当前目标状态、时间戳和云台旋转，计算发射指令，
     *              包括 yaw、pitch 和发射建议。可选地使用图像调试窗口显示当前帧信息
     */
    ShootResult shoot(const std::optional<Target> &target, std::chrono::steady_clock::time_point timestamp,
                     const Eigen::Matrix3d &R_gimbal2world, const cv::Mat &bgr_img = cv::Mat(),
                     const std::string &window_name = "");

    AimPoint debug_aim_point; ///< 调试用的瞄准点信息

  private:
    double comming_angle_           = 60 / 57.3; ///< 来板模式角度阈值（弧度）
    double leaving_angle_           = 20 / 57.3; ///< 离开模式角度阈值（弧度）
    double spinning_threshold_high_ = 6.28 ;     ///< 高速旋转阈值（rad/s）
    double spinning_threshold_low_  = 2 ;        ///< 低速旋转阈值（rad/s）

    int    lock_id_ = -1;          ///< 当前锁定的模型装甲板 ID
    double last_target_yaw_ = 0.0; ///< 连续化后的目标 yaw（跨帧累加，可超出 ±π）

    /**
     * @brief 选择最佳瞄准点
     * @param target 当前目标
     * @return 选定的瞄准点
     * @details 根据当前目标状态和上一次锁定的装甲板 ID，选择最佳瞄准点
     */
    AimPoint chooseAimPoint(const Target &target);

    void showDebug(const std::optional<Target> &target, const AimPoint &aim_point, double gimbal_yaw,
                   double gimbal_pitch, float target_yaw, float target_pitch, uint8_t fire,
                   const cv::Mat &bgr_img, const std::string &window_name) const noexcept;

    // 配置参数

    double yaw_offset_           = 0.0;         ///< yaw 偏移量（弧度）
    double pitch_offset_         = 0.0;         ///< pitch 偏移量（弧度）
    double fire_delay_time_      = 0.02;        ///< 发弹延迟时间（秒）
    double yaw_tolerance_near_   = 2.0 / 57.3;  ///< 3 m 内 yaw 发弹容差（rad）
    double yaw_tolerance_far_    = 1.0 / 57.3;  ///< 3 m 外 yaw 发弹容差（rad）
    double pitch_tolerance_near_ = 2.0 / 57.3;  ///< 3 m 内 pitch 发弹容差（rad）
    double pitch_tolerance_far_  = 1.0 / 57.3;  ///< 3 m 外 pitch 发弹容差（rad）
    double bullet_speed_         = 25.0;        ///< 子弹初速度（m/s）
    double gravity_              = 9.81;        ///< 重力加速度（m/s²）
    double air_resistance_       = 0.003;       ///< 二次空气阻力系数（1/m）
    bool   debug_                = false;       ///< 图像叠加调试开关
    bool   plotter_enable_       = false;       ///< plotter 输出开关
    std::unique_ptr<rm_utils::Plotter> plotter_; ///< plotter 实例

    mutable std::map<std::string, double>                                fps_map_;       ///< FPS 统计
    mutable std::map<std::string, int>                                   count_map_;     ///< 帧计数
    mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_; ///< 时间统计
};

} // namespace auto_aim

#endif // _ARMOR_SHOOT_H_
