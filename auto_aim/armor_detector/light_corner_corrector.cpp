#include "light_corner_corrector.hpp"
#include <numeric>

namespace auto_aim
{

void LightCornerCorrector::correctCorners(Armor &armor, const cv::Mat &gray_img) noexcept
{
    lightbar_points_corrector(armor.left, gray_img);
    lightbar_points_corrector(armor.right, gray_img);
    // 更新装甲板的四个角点 (Armor::points)
    armor.points.clear();
    armor.points.reserve(4);
    armor.points.emplace_back(armor.left.top);     // 0: 左上
    armor.points.emplace_back(armor.right.top);    // 1: 右上
    armor.points.emplace_back(armor.right.bottom); // 2: 右下
    armor.points.emplace_back(armor.left.bottom);  // 3: 左下

    // 更新装甲板中心 (基于优化后的灯条中心)
    armor.center = (armor.left.center + armor.right.center) / 2.0;
}

void LightCornerCorrector::lightbar_points_corrector(LightBar &lightbar, const cv::Mat &gray_img) const noexcept
{
    // 配置参数
    constexpr float MAX_BRIGHTNESS = 25;   // 归一化最大亮度值
    constexpr float ROI_SCALE      = 0.1;  // ROI扩展比例
    constexpr float SEARCH_START   = 0.4;  // 搜索起始位置比例
    constexpr float SEARCH_END     = 0.6;  // 搜索结束位置比例
    constexpr float SEARCH_STEP    = 0.5f; // 搜索步长

    // ROI 扩展与裁剪
    cv::Rect roi_box = lightbar.rotated_rect.boundingRect();
    roi_box.x -= static_cast<int>(roi_box.width * ROI_SCALE);
    roi_box.y -= static_cast<int>(roi_box.height * ROI_SCALE);
    roi_box.width += static_cast<int>(2 * roi_box.width * ROI_SCALE);
    roi_box.height += static_cast<int>(2 * roi_box.height * ROI_SCALE);

    // 边界约束
    roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);
    if (roi_box.area() <= 0) return;

    cv::Mat roi = gray_img(roi_box);

    // Otsu + MinAreaRect,使用大津法进行二值化，去除背景干扰
    cv::Mat binary;
    cv::threshold(roi, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return;

    // 找到面积最大的轮廓（即灯条主体）
    auto max_contour =
        *std::max_element(contours.begin(), contours.end(), [](const auto &c1, const auto &c2) { return cv::contourArea(c1) < cv::contourArea(c2); });

    cv::RotatedRect min_rect = cv::minAreaRect(max_contour);

    // 转换为全局坐标
    cv::Point2f center_global(min_rect.center.x + roi_box.x, min_rect.center.y + roi_box.y);

    // 提取主轴方向向量
    cv::Point2f axis;
    float       rect_angle = min_rect.angle;
    float       rect_w     = min_rect.size.width;
    float       rect_h     = min_rect.size.height;

    // OpenCV 的 RotatedRect 角度定义特殊：(-90, 0]，且总是对应较长的边
    // 需要根据长宽比调整角度和轴向量
    if (rect_w < rect_h)
    {
        // 高 > 宽，灯条接近竖直，angle 对应的是宽度（水平边）的角度
        // 实际主轴角度 = angle + 90
        float rad       = (rect_angle + 90) * CV_PI / 180.0;
        axis            = cv::Point2f(std::cos(rad), std::sin(rad));
        lightbar.length = rect_h;
        lightbar.width  = rect_w;
    }
    else
    {
        // 宽 > 高，灯条接近水平，angle 对应的是长边的角度
        float rad       = rect_angle * CV_PI / 180.0;
        axis            = cv::Point2f(std::cos(rad), std::sin(rad));
        lightbar.length = rect_w;
        lightbar.width  = rect_h;
    }

    // 统一轴向：强制让轴向下 (y > 0)，方便后续处理
    if (axis.y < 0) axis = -axis;

    // 更新灯条中心,使用 MinAreaRect 的中心
    lightbar.center = center_global;

    // 梯度搜索
    // 计算原始灰度图的均值，作为动态阈值参考
    const float mean_val = cv::mean(roi)[0];

    // 双线性插值辅助函数
    const uint8_t *gray_data        = gray_img.data;
    const size_t   gray_step        = gray_img.step[0];
    const int      gray_cols        = gray_img.cols;
    const int      gray_rows        = gray_img.rows;
    auto           get_pixel_interp = [gray_data, gray_step, gray_cols, gray_rows](const cv::Point2f &pt) -> float {
        int x = static_cast<int>(std::floor(pt.x));
        int y = static_cast<int>(std::floor(pt.y));
        if (x < 0 || x >= gray_cols - 1 || y < 0 || y >= gray_rows - 1) return 0.0f;
        float          u    = pt.x - x;
        float          v    = pt.y - y;
        const uint8_t *row0 = gray_data + y * gray_step;
        const uint8_t *row1 = row0 + gray_step;
        return (1.0f - u) * (1.0f - v) * row0[x] + u * (1.0f - v) * row0[x + 1] + (1.0f - u) * v * row1[x] + u * v * row1[x + 1];
    };

    const auto find_corner = [&](int direction) -> cv::Point2f {
        const float dx            = axis.x * direction;
        const float dy            = axis.y * direction;
        const float search_length = lightbar.length * (SEARCH_END - SEARCH_START);

        // 垂直于主轴的向量 (用于横向扫描)
        cv::Point2f perp_axis(-axis.y, axis.x);

        std::vector<cv::Point2f> candidates;

        // 横向采样多个候选线 (基于 MinAreaRect 的宽度)
        const int half_width = std::max(1, static_cast<int>(lightbar.width / 2.0));

        for (int i_offset = -half_width; i_offset <= half_width; ++i_offset)
        {
            // 计算搜索起点,这里使用 perp_axis 进行偏移，适应任意角度
            cv::Point2f start_point(center_global.x + lightbar.length * SEARCH_START * dx + perp_axis.x * i_offset,
                                    center_global.y + lightbar.length * SEARCH_START * dy + perp_axis.y * i_offset);

            cv::Point2f corner   = start_point;
            float       max_diff = 0;
            bool        found    = false;

            // 沿轴搜索亮度跳变点,步长取 SEARCH_STEP
            for (float step = 0; step < search_length; step += SEARCH_STEP)
            {
                const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

                // 边界检查 (全局坐标)
                if (cur_point.x < 0 || cur_point.x >= gray_img.cols || cur_point.y < 0 || cur_point.y >= gray_img.rows)
                {
                    break;
                }

                // 计算亮度差（使用双线性插值）,prev_point 是 step 上一个点
                cv::Point2f prev_point = cur_point - cv::Point2f(dx * SEARCH_STEP * 0.5f, dy * SEARCH_STEP * 0.5f);

                const auto  prev_val = get_pixel_interp(prev_point);
                const auto  cur_val  = get_pixel_interp(cur_point);
                const float diff     = prev_val - cur_val;

                if (diff > max_diff && prev_val > mean_val)
                {
                    max_diff = diff;
                    corner   = prev_point; // 跳变发生在上一位置
                    found    = true;
                }
            }

            if (found)
            {
                candidates.push_back(corner);
            }
        }

        // 返回候选点均值
        if (candidates.empty())
        {
            // 降级策略：如果找不到梯度点，返回几何计算的端点
            float dir_factor = (direction > 0) ? -0.5f : 0.5f;
            return cv::Point2f(center_global.x + axis.x * lightbar.length * dir_factor, center_global.y + axis.y * lightbar.length * dir_factor);
        }

        cv::Point2f sum = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
        return sum / static_cast<float>(candidates.size());
    };

    // 检测顶部和底部
    lightbar.top    = find_corner(-1); // 向上搜索 → top
    lightbar.bottom = find_corner(1);  // 向下搜索 → bottom
}

} // namespace auto_aim
