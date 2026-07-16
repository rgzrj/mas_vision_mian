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

    void correctCorners(Armor &armor, const cv::Mat &gray_img) noexcept;

  private:
    void lightbar_points_corrector(LightBar &lightbar, const cv::Mat &gray_img) const noexcept;
};

} // namespace auto_aim

#endif // _LIGHT_CORRECTOR_H_
