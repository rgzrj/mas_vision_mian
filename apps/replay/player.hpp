#pragma once

#include "common_def.hpp"
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <filesystem>
#include <opencv2/videoio.hpp>
#include <vector>

namespace replay_app
{
struct Metadata
{
    int64_t frame_index, timestamp_ns, t_recv_ns;
    std::array<double, 4> q;
    int mode;
};

std::vector<Metadata> read_metadata(const std::filesystem::path &path);

struct PairedFrame
{
    CameraFrame camera;
    Eigen::Quaterniond q;
    Metadata metadata;
};

class Player
{
  public:
    Player(const std::filesystem::path &video, const std::filesystem::path &csv);
    bool next(PairedFrame &frame);
    const std::vector<Metadata> &frames() const { return frames_; }

  private:
    std::vector<Metadata> frames_;
    cv::VideoCapture capture_;
    std::size_t next_ = 0;
    std::chrono::steady_clock::time_point origin_;
};
}
