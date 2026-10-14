#ifndef _ARMOR_TARGET_H_
#define _ARMOR_TARGET_H_

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "armor_types.hpp"
#include "extended_kalman_filter.hpp"

namespace auto_aim
{

/**
 * @brief 跟踪目标类
 * @details 封装目标状态、EKF滤波器和预测/更新方法
 */
class Target
{
  public:
    static constexpr double MAX_HEIGHT_DIFF = 0.2;    // 最大高度差，单位：米

    std::string name;         // 目标编号
    ArmorType   armor_type;   // 装甲板类型
    bool        jumped;       // 是否发生跳变
    int         last_id;      // 上一次跟踪的装甲板ID
    int         armor_num;    // 装甲板数量
    int         update_count; // 更新次数
    bool        is_switch;    // 是否切换目标

    Target() = default;

    /**
     * @brief 构造函数
     * @param armor 初始装甲板
     * @param t 时间戳
     * @param radius_02 偶数板旋转半径
     * @param radius_13 奇数板旋转半径
     * @param height 奇数板相对偶数板高度
     * @param armor_num 装甲板数量
     * @param P0_diag 初始协方差对角线
     * @details 初始化目标状态，设置初始装甲板、时间戳、旋转半径、装甲板数量和初始协方差
     *                     初始化 EKF 滤波器，设置状态向量和协方差矩阵
     */
    Target(const Armor &armor, std::chrono::steady_clock::time_point t, double radius_02,
           double radius_13, double height, int armor_num, const Eigen::VectorXd &P0_diag);

    /**
     * @brief 预测目标在未来时刻的状态
     * @param t 未来时间戳
     */
    void predict(std::chrono::steady_clock::time_point t);

    /**
     * @brief 使用完整装甲板 PnP 位姿匹配模型槽位并更新 EKF
     * @return 本次更新是否成功应用
     */
    bool update(const Armor &armor);

    /**
     * @brief 获取上一帧更新的 z 方向新息
     */
    double last_innovation_z() const { return last_innovation_z_; }
    /**
     * @brief 获取上一帧更新的位移新息
     */
    double last_innovation_norm() const { return last_innovation_norm_; }

    /**
     * @brief 获取 EKF 状态向量
     */
    Eigen::VectorXd ekf_x() const { return ekf_.x; }

    /**
     * @brief 获取 EKF 滤波器
     */
    const rm_utils::ExtendedKalmanFilter &ekf() const { return ekf_; }

    /**
     * @brief 获取所有装甲板的位置和角度列表
     */
    std::vector<Eigen::Vector4d> armor_xyza_list() const;

    /**
     * @brief 检查目标是否发散
     */
    bool diverged() const;

    /**
     * @brief 检查目标是否收敛
     */
    bool converged() const;

    /**
     * @brief 四板模型是否已有足够的分组观测
     * @details 要求四个 ID 都出现过；只作为预测可信门，不停止后续几何更新
     */
    bool model_observations_sufficient() const;
    /**
     * @brief 四板模型是否已提交几何证据
     * @details 要求四个 ID 都出现过，并且至少有一次几何观测被计入 EKF 更新；
     *            一旦提交几何证据，后续几何观测将不再计入 EKF 更新，直到目标被重置或重新初始化
     */
    bool geometry_committed() const noexcept { return geometry_committed_; }
    /**
     * @brief 获取指定组的访问次数
     * @param group 组索引（0 或 1）
     * @return 访问次数
     */
    int radius_group_visit_count(int group) const
    {
        return group >= 0 && group < static_cast<int>(group_visits_.size())
                   ? group_visits_[group]
                   : 0;
    }

    /**
     * @brief 获取时间戳
     */
    std::chrono::steady_clock::time_point timestamp() const { return timestamp_; }

  private:
    rm_utils::ExtendedKalmanFilter        ekf_;            // EKF滤波器
    std::chrono::steady_clock::time_point init_timestamp_; // 初始化时间戳
    std::chrono::steady_clock::time_point timestamp_;      // 时间戳

    double                                last_innovation_z_         = 0.0;       // 上一帧更新的 z 方向新息
    double                                last_innovation_norm_      = 0.0;       // 上一帧更新的位移新息

    std::array<int, 2> group_visits_{{0, 0}}; // 两组独立装甲板访问次数

    unsigned int       observed_id_mask_         = 0;  // 已成功关联的四板 ID 位图
    int                last_observed_id_   = -1; // 上一次计入几何证据的模型 ID
    bool               geometry_committed_             = false; // 四个模型板均已提供 PnP 位姿观测

    /**
     * @brief 更新EKF（yaw, pitch, distance, angle）
     */
    void update_ypda(const Armor &armor, int id);
    /**
     * @brief 更新EKF（xyz, ypr）
     */
    void record_observation(int id);
    /**
     * @brief 观测函数：计算装甲板在世界坐标系下的位置
     */
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd &x, int id) const;

    /**
     * @brief 观测函数雅可比矩阵
     */
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd &x, int id) const;

    /**
     * @brief 约束状态向量
     */
    void constrainState();
};

} // namespace auto_aim

#endif // _ARMOR_TARGET_H_
