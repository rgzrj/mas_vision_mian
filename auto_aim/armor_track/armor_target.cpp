#include "armor_target.hpp"

#include <algorithm>
#include <cmath>

#include "math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr bool radiusModelValid(double radius_02, double radius_13)
{
    return radius_02 > 0.05 && radius_02 <= 0.5 && radius_13 > 0.05 && radius_13 <= 0.5;
}

constexpr bool convergence_ready(int updates, double cov_x, double cov_y, double age)
{
    return updates >= 10 && cov_x <= 1.0 && cov_y <= 1.0 && age >= 0.1;
}

constexpr bool observations_sufficient(unsigned int id_mask, int group_02_visits, int group_13_visits)
{
    return (id_mask & 0x0FU) == 0x0FU && group_02_visits >= 2 && group_13_visits >= 2;
}

} // namespace

/**
 * @brief 构造函数
 * @details 根据装甲板初始化EKF状态向量和协方差矩阵
 */
Target::Target(const Armor &armor, std::chrono::steady_clock::time_point t, double radius_02,
               double radius_13, double height, int armor_num, const Eigen::VectorXd &P0_diag)
    : jumped(false), last_id(0), armor_num(armor_num), update_count(0), is_switch(false),
      init_timestamp_(t), timestamp_(t)
{
    name       = armor.number;
    armor_type = armor.type;

    const Eigen::Vector3d &xyz = armor.xyz_in_world;
    const Eigen::Vector3d &ypr = armor.ypr_in_world;

    // 根据装甲板位置反推旋转中心
    auto center_x = xyz[0] + radius_02 * std::cos(ypr[0]);
    auto center_y = xyz[1] + radius_02 * std::sin(ypr[0]);
    auto center_z = xyz[2];

    // x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, radius_02, radius_13, h]
    Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0,
                        radius_02, radius_13, height}};
    Eigen::MatrixXd P0 = P0_diag.asDiagonal();

    // 状态向量加法（处理角度环绕）
    auto x_add = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a + b;
        c[6]              = rm_utils::limit_rad(c[6]);
        return c;
    };

    ekf_ = rm_utils::ExtendedKalmanFilter(x0, P0, x_add);
}

/**
 * @brief 预测目标在未来时刻的状态
 * @details 使用EKF进行状态预测，构建状态转移矩阵和过程噪声矩阵
 */
void Target::predict(std::chrono::steady_clock::time_point t)
{
    is_switch = false;

    if (ekf_.x.size() < 11 || ekf_.P.rows() < 11 || ekf_.P.cols() < 11 ||
        !ekf_.x.allFinite() || !ekf_.P.allFinite())
    {
        timestamp_ = t;
        return;
    }

    // 计算时间间隔
    auto dt    = rm_utils::delta_time(t, timestamp_);
    if (!std::isfinite(dt) || dt < 0.0) return;
    timestamp_ = t;

    // 状态转移矩阵 F (匀速模型)
    // x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, radius_02, radius_13, h]
    // clang-format off
    Eigen::MatrixXd F{
        {1, dt, 0,  0, 0,  0, 0,  0, 0, 0, 0},
        {0,  1, 0,  0, 0,  0, 0,  0, 0, 0, 0},
        {0,  0, 1, dt, 0,  0, 0,  0, 0, 0, 0},
        {0,  0, 0,  1, 0,  0, 0,  0, 0, 0, 0},
        {0,  0, 0,  0, 1, dt, 0,  0, 0, 0, 0},
        {0,  0, 0,  0, 0,  1, 0,  0, 0, 0, 0},
        {0,  0, 0,  0, 0,  0, 1, dt, 0, 0, 0},
        {0,  0, 0,  0, 0,  0, 0,  1, 0, 0, 0},
        {0,  0, 0,  0, 0,  0, 0,  0, 1, 0, 0},
        {0,  0, 0,  0, 0,  0, 0,  0, 0, 1, 0},
        {0,  0, 0,  0, 0,  0, 0,  0, 0, 0, 1}
    };
    // clang-format on

    // 过程噪声参数：前哨站运动较慢，使用较小的噪声
    double v1 = (name == "outpost") ? 10 : 100;
    double v2 = (name == "outpost") ? 0.1 : 400;

    // 计算过程噪声矩阵Q的系数
    auto   a   = dt * dt * dt * dt / 4;
    auto   b   = dt * dt * dt / 2;
    auto   c   = dt * dt;
    double q_h = (name == "outpost") ? 0.0 : 1e-6;

    // 过程噪声矩阵 Q
    // clang-format off
    Eigen::MatrixXd Q{
        {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0,   0},
        {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0,   0},
        {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0,   0},
        {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0,   0},
        {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0,   0},
        {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0,   0},
        {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0,   0},
        {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0,   0},
        {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0,   0},
        {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0,   0},
        {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, q_h}
    };
    // clang-format on

    // 非线性状态转移函数（处理角度环绕）
    auto f = [&](const Eigen::VectorXd &x) -> Eigen::VectorXd {
        Eigen::VectorXd x_prior = F * x;
        x_prior[6]              = rm_utils::limit_rad(x_prior[6]);
        return x_prior;
    };

    // 前哨站转速特判：收敛后限制角速度为理论值
    if (converged() && name == "outpost" && std::abs(ekf_.x[7]) > 2) ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;

    // 执行EKF预测
    ekf_.predict(F, Q, f);
}

bool Target::update(const Armor &armor)
{
    int    best_id         = -1;
    double min_angle_error = std::numeric_limits<double>::infinity();
    const auto predicted = armor_xyza_list();
    using Candidate = std::pair<Eigen::Vector4d, int>;
    std::vector<Candidate> candidates;
    candidates.reserve(predicted.size());
    for (int id = 0; id < armor_num && id < static_cast<int>(predicted.size()); ++id)
        candidates.push_back({predicted[id], id});

    // 按距离从小到大排序（距离最近的优先匹配）        
    std::sort(candidates.begin(), candidates.end(), [](const Candidate &lhs, const Candidate &rhs) {
        return rm_utils::xyz2ypd(lhs.first.head<3>())[2] <
               rm_utils::xyz2ypd(rhs.first.head<3>())[2];
    });

    const int match_count = std::min(3, static_cast<int>(candidates.size()));
    for (int index = 0; index < match_count; ++index)
    {
        const auto &[xyza, id] = candidates[index];
        const Eigen::Vector3d ypd = rm_utils::xyz2ypd(xyza.head<3>());
        const double angle_error =
            std::abs(rm_utils::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
            std::abs(rm_utils::limit_rad(armor.ypd_in_world[0] - ypd[0]));
        if (angle_error < min_angle_error)
        {
            min_angle_error = angle_error;
            best_id         = id;
        }
    }
    if (best_id < 0) return false;

    const Eigen::Vector3d innovation = armor.xyz_in_world - predicted[best_id].head<3>();
    last_innovation_z_    = innovation.z();
    last_innovation_norm_ = innovation.norm();
    
    if (best_id != 0) jumped = true;
    is_switch = best_id != last_id;
    last_id = best_id;
    update_count++;

    update_ypda(armor, best_id);
    if (!ekf_.last_update_applied) return false;
    constrainState();

    record_observation(best_id);
    if (observations_sufficient(observed_id_mask_, group_visits_[0],
                                       group_visits_[1]))
        geometry_committed_ = true;
    return true;
}

void Target::record_observation(int id)
{
    if (armor_num != 4 || id < 0 || id >= armor_num || id == last_observed_id_) return;
    observed_id_mask_ |= 1U << id;
    group_visits_[id & 1]++;
    last_observed_id_ = id;
}

/**
 * @brief 更新EKF（yaw, pitch, distance, angle）
 */
void Target::update_ypda(const Armor &armor, int id)
{
    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

    // 自适应观测噪声：根据装甲板角度和距离调整
    auto            center_yaw  = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
    auto            delta_angle = rm_utils::limit_rad(armor.ypr_in_world[0] - center_yaw);
    Eigen::VectorXd R_diag{{4e-3, 4e-3, std::log(std::abs(delta_angle) + 1) + 1,
                           std::log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

    Eigen::MatrixXd R = R_diag.asDiagonal();

    // 观测函数：从状态向量计算观测值
    auto h = [&](const Eigen::VectorXd &x) -> Eigen::Vector4d {
        Eigen::VectorXd xyz   = h_armor_xyz(x, id);
        Eigen::VectorXd ypd   = rm_utils::xyz2ypd(xyz);
        auto            angle = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / armor_num);
        return {ypd[0], ypd[1], ypd[2], angle};
    };

    // 观测值减法（处理角度环绕）
    auto z_subtract = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a - b;
        c[0]              = rm_utils::limit_rad(c[0]);
        c[1]              = rm_utils::limit_rad(c[1]);
        c[3]              = rm_utils::limit_rad(c[3]);
        return c;
    };

    // 实际观测值 z = [yaw, pitch, distance, armor_yaw]
    const Eigen::Vector3d &ypd = armor.ypd_in_world;
    const Eigen::Vector3d &ypr = armor.ypr_in_world;
    Eigen::VectorXd        z{{ypd[0], ypd[1], ypd[2], ypr[0]}};
    ekf_.update(z, H, R, h, z_subtract);
}

/**
 * @brief 观测函数：计算装甲板在世界坐标系下的位置
 */
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd &x, int id) const
{
    // 前哨站特殊处理：三层高度，每层一个装甲板
    if (name == "outpost")
    {
        auto angle = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / 3);
        auto r     = x[8];
        auto ax    = x[0] - r * std::cos(angle);
        auto ay    = x[2] - r * std::sin(angle);
        auto az    = x[4] + (id - 1) * 0.102;
        return {ax, ay, az};
    }

    // 普通目标：4个装甲板，id=0,2 使用 radius_02，id=1,3 使用独立的 radius_13 和高度差 h
    auto   angle   = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / armor_num);
    auto   odd_group  = (armor_num == 4) && (id == 1 || id == 3);
    auto   r       = odd_group ? x[9] : x[8];
    auto   armor_x = x[0] - r * std::cos(angle);
    auto   armor_y = x[2] - r * std::sin(angle);
    double dz      = odd_group ? x[10] : 0;
    auto   armor_z = x[4] + dz;

    return {armor_x, armor_y, armor_z};
}

/**
 * @brief 观测函数雅可比矩阵
 */
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd &x, int id) const
{
    // 前哨站特殊处理
    if (name == "outpost")
    {
        auto angle = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / 3);
        auto r     = x[8];

        auto dx_da = r * std::sin(angle);
        auto dy_da = -r * std::cos(angle);
        auto dx_dr = -std::cos(angle);
        auto dy_dr = -std::sin(angle);

        // clang-format off
        Eigen::MatrixXd H_armor_xyza{
            {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, 0, 0},
            {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, 0, 0},
            {0, 0, 0, 0, 1, 0,     0, 0,     0, 0, 0},
            {0, 0, 0, 0, 0, 0,     1, 0,     0, 0, 0}
        };
        // clang-format on

        Eigen::VectorXd armor_xyz   = h_armor_xyz(x, id);
        Eigen::MatrixXd H_armor_ypd = rm_utils::xyz2ypd_jacobian(armor_xyz);

        // clang-format off
        Eigen::MatrixXd H_armor_ypda{
            {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
            {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
            {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
            {                0,                 0,                 0, 1}
        };
        // clang-format on

        return H_armor_ypda * H_armor_xyza;
    }

    // 普通目标：4个装甲板，两组半径分别对应独立状态，不再用 r+l 表示第二组半径
    auto angle  = rm_utils::limit_rad(x[6] + id * 2 * CV_PI / armor_num);
    auto odd_group = (armor_num == 4) && (id == 1 || id == 3);
    auto r      = odd_group ? x[9] : x[8];

    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);
    auto dx_dr02 = odd_group ? 0.0 : -std::cos(angle);
    auto dy_dr02 = odd_group ? 0.0 : -std::sin(angle);
    auto dx_dr13 = odd_group ? -std::cos(angle) : 0.0;
    auto dy_dr13 = odd_group ? -std::sin(angle) : 0.0;
    auto dz_dh = odd_group ? 1.0 : 0.0;

    // clang-format off
    Eigen::MatrixXd H_armor_xyza{
        {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr02, dx_dr13,     0},
        {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr02, dy_dr13,     0},
        {0, 0, 0, 0, 1, 0,     0, 0,       0,       0, dz_dh},
        {0, 0, 0, 0, 0, 0,     1, 0,       0,       0,     0}
    };
    // clang-format on

    Eigen::VectorXd armor_xyz   = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = rm_utils::xyz2ypd_jacobian(armor_xyz);

    // clang-format off
    Eigen::MatrixXd H_armor_ypda{
        {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
        {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
        {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
        {                0,                 0,                 0, 1}
    };
    // clang-format on

    return H_armor_ypda * H_armor_xyza;
}

/**
 * @brief 获取所有装甲板的位置和角度列表
 */
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
    std::vector<Eigen::Vector4d> list;
    if (armor_num <= 0 || ekf_.x.size() < 11 || !ekf_.x.allFinite()) return list;
    list.reserve(static_cast<std::size_t>(armor_num));

    for (int i = 0; i < armor_num; i++)
    {
        auto            angle = rm_utils::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num);
        Eigen::Vector3d xyz   = h_armor_xyz(ekf_.x, i);
        list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return list;
}

/**
 * @brief 检查目标是否发散
 */
bool Target::diverged() const
{
    if (ekf_.x.size() < 11 || ekf_.P.rows() < 11 || ekf_.P.cols() < 11 ||
        !ekf_.x.allFinite() || !ekf_.P.allFinite())
        return true;
    if (!radiusModelValid(ekf_.x[8], ekf_.x[9])) return true;
    return ekf_.P(0, 0) > 150.0 || ekf_.P(2, 2) > 150.0 ||
           std::abs(ekf_.x[0]) > 15.0 || std::abs(ekf_.x[2]) > 15.0 ||
           std::abs(ekf_.x[4]) > 2.0 || std::abs(ekf_.x[7]) > 10.0 * CV_PI ||
           std::abs(ekf_.x[1]) > 5.0 || std::abs(ekf_.x[3]) > 5.0 ||
           std::abs(ekf_.x[5]) > 1.0;
}

/**
 * @brief 检查目标是否收敛
 */
bool Target::converged() const
{
    if (diverged()) return false;

    const double age = std::chrono::duration<double>(timestamp_ - init_timestamp_).count();
    return convergence_ready(update_count, ekf_.P(0, 0), ekf_.P(2, 2), age);
}

bool Target::model_observations_sufficient() const
{
    if (armor_num != 4) return true;
    return geometry_committed_ &&
           observations_sufficient(observed_id_mask_, group_visits_[0],
                                           group_visits_[1]);
}

void Target::constrainState()
{
    // 两组半径均为独立物理量，避免单板观测通过 r+l 同时拉动两个半径。
    ekf_.x[8] = std::clamp(ekf_.x[8], 0.08, 0.45);

    if (armor_num == 4)
    {
        ekf_.x[9] = std::clamp(ekf_.x[9], 0.08, 0.5);
    }

    if (armor_num == 4 && name != "outpost")
        ekf_.x[10] = std::clamp(ekf_.x[10], -MAX_HEIGHT_DIFF, MAX_HEIGHT_DIFF);
}

} // namespace auto_aim
