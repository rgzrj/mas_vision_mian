#ifndef _LIGHT_CORRECTOR_H_
#define _LIGHT_CORRECTOR_H_

#include <opencv2/opencv.hpp>
#include <vector>

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
     * @brief 配板前逐根精修灯条端点，原地更新灯条几何
     * @param lights 本帧来源灯条，每根只精修一次
     * @param gray_img 灰度图像
     */
    void correctCorners(std::vector<LightBar> &lights, const cv::Mat &gray_img) const noexcept;

  private:
    /**
     * @brief 对单根灯条的上下端点进行修正并同步几何
     * @param lightbar 待修正的灯条
     * @param gray_img 灰度图像
     */
    void correctLightbar(LightBar &lightbar, const cv::Mat &gray_img) const noexcept;
};

} // namespace auto_aim

#endif // _LIGHT_CORRECTOR_H_
