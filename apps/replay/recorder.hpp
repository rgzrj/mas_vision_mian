#pragma once

#include "common_def.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace replay_app
{
inline bool recording_frame_due(int64_t timestamp_ns, int64_t period_ns, int64_t &next_ns)
{
    if (period_ns == 0) return true;
    if (next_ns < 0) next_ns = timestamp_ns;
    if (timestamp_ns < next_ns) return false;
    // Advance on the exposure timeline, avoiding drift and catch-up bursts after gaps.
    next_ns += ((timestamp_ns - next_ns) / period_ns + 1) * period_ns;
    return true;
}

class PairedRecorder
{
    struct Frame
    {
        cv::Mat image;
        int64_t timestamp_ns, recv_ns;
        Eigen::Quaterniond q;
        int mode;
    };
    std::filesystem::path output_;
    double fps_;
    double max_record_fps_;
    int64_t sample_period_ns_ = 0, next_sample_ns_ = -1;
    int64_t first_saved_ns_ = 0, last_saved_ns_ = 0;
    std::size_t skipped_by_rate_limit_ = 0, accepted_frames_ = 0;
    std::string codec_;
    std::size_t capacity_;
    std::thread writer_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Frame> queue_;
    bool stopping_ = false, failed_ = false;
    std::string error_;
    std::size_t received_ = 0, written_ = 0, peak_queue_ = 0;
    std::chrono::steady_clock::time_point first_input_, last_input_;
    double video_seconds_ = 0, csv_seconds_ = 0, max_video_seconds_ = 0, open_seconds_ = 0;

    // Caller holds mutex_. Input rate stops at the overflow, before draining the saved prefix.
    nlohmann::json statistics() const
    {
        const double span = received_ > 1 ? std::chrono::duration<double>(last_input_ - first_input_).count() : 0;
        return {{"received_frames", received_}, {"written_frames", written_}, {"peak_queue", peak_queue_},
                {"input_fps", span > 0 ? (received_ - 1) / span : 0},
                {"skipped_by_rate_limit", skipped_by_rate_limit_}, {"accepted_frames", accepted_frames_},
                {"recorded_input_fps", accepted_frames_ > 1 && last_saved_ns_ > first_saved_ns_ ?
                    (accepted_frames_ - 1) * 1.0e9 / (last_saved_ns_ - first_saved_ns_) : 0},
                {"mean_video_ms", written_ ? video_seconds_ * 1000 / written_ : 0},
                {"max_video_ms", max_video_seconds_ * 1000},
                {"mean_csv_ms", written_ ? csv_seconds_ * 1000 / written_ : 0},
                {"video_open_ms", open_seconds_ * 1000}};
    }

    void fail(const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ = true;
        error_ = reason;
        std::cerr << "RECORDING FAILED: " << reason << std::endl;
    }

    void write()
    {
        std::size_t count = 0;
        cv::VideoWriter video;
        try
        {
            std::ofstream csv(output_ / "frames.csv");
            csv.exceptions(std::ios::failbit | std::ios::badbit);
            csv << "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode\n" << std::setprecision(17);
            for (;;)
            {
                Frame frame;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    ready_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
                    if (queue_.empty()) break;
                    frame = std::move(queue_.front());
                    queue_.pop_front();
                }
                if (!video.isOpened())
                {
                    const auto start = std::chrono::steady_clock::now();
                    const int codec = cv::VideoWriter::fourcc(codec_[0], codec_[1], codec_[2], codec_[3]);
                    const auto path = (output_ / "video.avi").string();
                    const int backend = codec_ == "MJPG" ? cv::CAP_OPENCV_MJPEG : cv::CAP_ANY;
                    bool opened = video.open(path, backend, codec, fps_, frame.image.size(), true);
                    if (!opened && codec_ == "MJPG")
                    {
                        std::cerr << "OpenCV MJPEG writer unavailable; trying default backend" << std::endl;
                        opened = video.open(path, cv::CAP_ANY, codec, fps_, frame.image.size(), true);
                    }
                    if (!opened)
                        throw std::runtime_error("Cannot open video writer with configured codec");
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        open_seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                    }
                    std::cerr << "RECORDING VIDEO: " << frame.image.cols << 'x' << frame.image.rows
                              << " codec=" << codec_ << " backend=" << video.getBackendName() << std::endl;
                }
                const auto video_start = std::chrono::steady_clock::now();
                video.write(frame.image);
                const auto csv_start = std::chrono::steady_clock::now();
                csv << count << ',' << frame.timestamp_ns << ',' << frame.recv_ns << ','
                    << frame.q.w() << ',' << frame.q.x() << ',' << frame.q.y() << ',' << frame.q.z()
                    << ',' << frame.mode << '\n';
                csv.flush();
                ++count;
                const auto finish = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    written_ = count;
                    const double elapsed = std::chrono::duration<double>(csv_start - video_start).count();
                    video_seconds_ += elapsed;
                    max_video_seconds_ = std::max(max_video_seconds_, elapsed);
                    csv_seconds_ += std::chrono::duration<double>(finish - csv_start).count();
                    if (count % 300 == 0)
                        std::cerr << "RECORDING STATS: " << statistics().dump() << std::endl;
                }
            }
            video.release();
        }
        catch (const std::exception &e) { fail(e.what()); }
        video.release();
        std::lock_guard<std::mutex> lock(mutex_);
        if (count == 0 && !failed_) { failed_ = true; error_ = "No paired frames recorded"; }
        std::ofstream summary(output_ / "recording_summary.json");
        summary << nlohmann::json{{"frames", count}, {"failed", failed_}, {"error", error_},
                                  {"codec", codec_}, {"fps_metadata", fps_},
                                  {"max_record_fps", max_record_fps_},
                                  {"queue_size", capacity_}, {"performance", statistics()}}.dump(2);
    }

  public:
    explicit PairedRecorder(const std::string &path)
    {
        const auto config = YAML::LoadFile(path);
        output_ = config["output_dir"].as<std::string>();
        fps_ = config["fps"].as<double>();
        max_record_fps_ = config["max_record_fps"] ? config["max_record_fps"].as<double>() : 0;
        codec_ = config["codec"].as<std::string>();
        capacity_ = config["queue_size"].as<std::size_t>();
        if (codec_.size() != 4 || !std::isfinite(fps_) || fps_ <= 0 || capacity_ == 0 ||
            !std::isfinite(max_record_fps_) || max_record_fps_ < 0 || max_record_fps_ > 1.0e9 ||
            (max_record_fps_ > 0 && max_record_fps_ < 1.0e-9))
            throw std::runtime_error("Invalid recording settings");
        if (max_record_fps_ > 0) sample_period_ns_ = static_cast<int64_t>(std::ceil(1.0e9 / max_record_fps_));
        std::cerr << "RECORDING SETTINGS: codec=" << codec_ << " queue_size=" << capacity_
                  << " fps_metadata=" << fps_ << " max_record_fps=" << max_record_fps_ << std::endl;
        writer_ = std::thread([this] { write(); });
    }

    ~PairedRecorder()
    {
        { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
        ready_.notify_one();
        writer_.join();
    }

    void push(const CameraFrame &source, const Eigen::Quaterniond &q, int mode)
    {
        const auto ns = [](auto timestamp) {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();
        };
        std::lock_guard<std::mutex> lock(mutex_);
        if (failed_) return;
        last_input_ = std::chrono::steady_clock::now();
        if (received_++ == 0) first_input_ = last_input_;
        const int64_t timestamp_ns = ns(source.timestamp);
        if (!recording_frame_due(timestamp_ns, sample_period_ns_, next_sample_ns_))
        {
            ++skipped_by_rate_limit_;
            return;
        }
        if (queue_.size() >= capacity_)
        {
            failed_ = true;
            error_ = "Recording queue full; reduce image size/load or use a faster disk/codec";
            std::cerr << "RECORDING FAILED: " << error_ << '\n'
                      << "RECORDING STATS AT FAILURE: " << statistics().dump() << std::endl;
            return;
        }
        queue_.push_back({source.frame.clone(), timestamp_ns, ns(source.t_recv), q, mode});
        if (accepted_frames_++ == 0) first_saved_ns_ = timestamp_ns;
        last_saved_ns_ = timestamp_ns;
        peak_queue_ = std::max(peak_queue_, queue_.size());
        ready_.notify_one();
    }
};
} // namespace replay_app
