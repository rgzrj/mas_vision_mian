#ifndef _LIGHT_CORRECTOR_H_
#define _LIGHT_CORRECTOR_H_

#include <opencv2/opencv.hpp>

#include "armor_types.hpp"

namespace auto_aim
{

struct SymmetryAxis
{
    cv::Point2f centroid;
    cv::Point2f direction;
    float       mean_val;
};

class LightCornerCorrector
{
  public:
    LightCornerCorrector() noexcept = default;
    /**
     * @brief QD式亮度质心修正灯条，仅整体平移灯条的中心与端点，不改变灯条几何形状大小
     * @param lightbar 待修正的灯条
     * @param gray_img 灰度图像
     */
    bool correctCenter(LightBar &lightbar, const cv::Mat &gray_img) const;
    /**
     * @brief 对装甲板的角点进行修正
     * @param armor 待修正的装甲板
     * @param gray_img 灰度图像
     */
    void correctCorners(Armor &armor, const cv::Mat &gray_img) noexcept;
    /**
     * @brief 对灯条的角点进行修正
     * @param lightbar 待修正的灯条
     * @param gray_img 灰度图像
     */
    void correctLightbar(LightBar &lightbar, const cv::Mat &gray_img) const noexcept;
};

} // namespace auto_aim

#endif // _LIGHT_CORRECTOR_H_
