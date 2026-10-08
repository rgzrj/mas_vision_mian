#ifndef _ARMOR_TYPES_H_
#define _ARMOR_TYPES_H_

#include <algorithm>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <limits>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum EnemyColor
{
    RED,
    BLUE,
    PURPLE,
};

enum ArmorType
{
    BIG,
    SMALL,
    UNKNOWN
};

const char *const ARMOR_TYPES[] = {"BIG", "SMALL", "UNKNOWN"};

const char *const ARMOR_PRIORITIES[] = {"1", "2", "3", "4", "5", "sentry","outpost","base"};

struct LightBar
{
    int                      source_id              = -1;             // 本帧检测来源，不是车体模型槽位
    EnemyColor               color                  = PURPLE;          // 灯条颜色
    bool                     target_color_confirmed = false;           // 是否明确偏向当前敌方装甲板颜色
    cv::Point2f              center, top, bottom, top2bottom;          // 灯条中心、上顶点、下顶点、上顶点到下顶点的距离
    std::vector<cv::Point2f> points;                                   // 灯条顶点
    double                   angle, angle_error, length, width, ratio; // 灯条角度、角度误差、长度、宽度、宽高比
    cv::RotatedRect          rotated_rect;                             // 灯条旋转矩形
    cv::Point2f              detected_center;                          // 精修前中心，仅供诊断
    double                   detected_length = 0.0;                    // 精修前长度，仅供诊断

    LightBar() = default;

    explicit LightBar(const cv::RotatedRect &rotated_rect) : rotated_rect(rotated_rect)
    {
        std::vector<cv::Point2f> corners(4);
        rotated_rect.points(&corners[0]);
        std::sort(corners.begin(), corners.end(), [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });

        center     = rotated_rect.center;
        top        = (corners[0] + corners[1]) / 2;
        bottom     = (corners[2] + corners[3]) / 2;
        top2bottom = bottom - top;

        points.emplace_back(top);
        points.emplace_back(bottom);

        width       = cv::norm(corners[0] - corners[1]);
        angle       = std::atan2(top2bottom.y, top2bottom.x);
        angle_error = std::abs(angle - CV_PI / 2);
        length      = cv::norm(top2bottom);
        ratio       = length / width;
        detected_center = center;
        detected_length = length;
    }
};

struct Armor
{
    EnemyColor               color;
    LightBar                 left, right; // 左右灯条
    cv::Point2f              center;      // 灯条中心连线的中点
    std::vector<cv::Point2f> points;      // 4个角点，顺序：左上->右上->右下->左下

    double ratio;             // 宽高比
    double side_ratio;        // 长短灯条比
    double rectangular_error; // 垂直度误差 (0 表示垂直)

    ArmorType   type;
    std::string number;     // 识别出的类别名称
    float       confidence; // 置信度
    cv::Mat     number_img; // 用于存储提取出的数字图像

    Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero(); // 单位：m
    Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero(); // 单位：rad
    Eigen::Vector3d xyz_in_world  = Eigen::Vector3d::Zero(); // 单位：m
    Eigen::Vector3d ypr_in_world  = Eigen::Vector3d::Zero(); // 单位：rad
    Eigen::Vector3d ypd_in_world  = Eigen::Vector3d::Zero(); // 球坐标系，单位：rad, m

    double yaw_raw = 0.0; // 原始 IPPE yaw 角度

    // PnP / 模型约束位姿诊断。默认值保持有限，避免 JSON/CSV 序列化 NaN/inf。
    bool   pnp_diagnostics_valid        = false;
    int    pnp_solution_count           = 0;
    int    pnp_selected_branch          = -1;
    double pnp_reprojection_rms         = 0.0;
    double pnp_second_reprojection_rms  = 0.0;
    std::array<Eigen::Vector3d, 2> pnp_candidate_xyz_world{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    std::array<Eigen::Vector3d, 2> pnp_candidate_normal_world{
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    std::array<int, 2> pnp_candidate_branch{{-1, -1}};
    std::array<double, 2> pnp_candidate_yaw{{0.0, 0.0}};
    std::array<double, 2> pnp_candidate_yaw_raw{{0.0, 0.0}};
    std::array<double, 2> pnp_candidate_rms{{0.0, 0.0}};

    bool   pose_model_refined           = false;
    double pose_refinement_rms          = 0.0;
    double pose_refinement_normalized   = 0.0;
    double pose_refinement_pitch        = 0.0;
    int    pose_raw_support_branch      = -1;
    double pose_raw_support_rms         = 0.0;

    double pose_raw_support_normalized_rms = 0.0;

    double pose_raw_translation_delta     = 0.0;
    double pose_raw_normal_delta          = 0.0;
    double pose_raw_normalized_rms_delta  = 0.0;
    double pose_translation_condition     = 0.0;
    double pose_translation_algebraic_rms = 0.0;
    bool   pose_yaw_search_boundary       = false;
    bool   pose_pitch_search_boundary     = false;
    bool   pose_independent_valid         = false;
    double pose_independent_score         = 0.0;

    Armor() {}

    explicit Armor(const LightBar &l, const LightBar &r) : left(l), right(r)
    {
        if (l.center.x < r.center.x)
        {
            left = l, right = r;
        }
        else
        {
            left = r, right = l;
        }

        color  = left.color;
        center = (left.center + right.center) / 2;

        // 4个角点顺序 (对应 solvePnP 的 object_points 顺序)
        points.clear();
        points.reserve(4);
        points.emplace_back(left.top);
        points.emplace_back(right.top);
        points.emplace_back(right.bottom);
        points.emplace_back(left.bottom);

        // 计算几何特征
        auto left2right          = right.center - left.center;
        auto width               = cv::norm(left2right);
        auto max_lightbar_length = std::max(left.length, right.length);
        auto min_lightbar_length = std::min(left.length, right.length);

        // 除 0 保护
        if (max_lightbar_length < 1e-6) max_lightbar_length = 1e-6;
        if (min_lightbar_length < 1e-6) min_lightbar_length = 1e-6;

        ratio      = width / max_lightbar_length;
        side_ratio = max_lightbar_length / min_lightbar_length;

        auto roll = std::atan2(left2right.y, left2right.x);

        // 垂直度误差计算
        auto left_cos     = std::abs(std::cos(left.angle - roll));
        auto right_cos    = std::abs(std::cos(right.angle - roll));
        rectangular_error = std::max(left_cos, right_cos);
    }
};

constexpr double LIGHTBAR_LENGTH   = 48e-3;  // m
constexpr double BIG_ARMOR_WIDTH   = 230e-3; // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3; // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{{0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},       // 左上
                                                {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},      // 右上
                                                {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},     // 右下
                                                {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};     // 左下
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{{0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},   // 左上
                                                  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},  // 右上
                                                  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}, // 右下
                                                  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}}; // 左下

} // namespace auto_aim

#endif // _ARMOR_TYPES_H_
