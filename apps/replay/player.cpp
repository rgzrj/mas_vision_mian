#include "player.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>

namespace replay_app
{
namespace
{
std::vector<std::string> csv_fields(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i)
    {
        const char c = line[i];
        if (c == '"')
        {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') { field += '"'; ++i; }
            else quoted = !quoted;
        }
        else if (c == ',' && !quoted) { fields.push_back(field); field.clear(); }
        else if (c != '\r') field += c;
    }
    if (quoted) throw std::runtime_error("Unclosed CSV quote");
    fields.push_back(field);
    return fields;
}

int64_t integer(const std::string &s)
{
    std::size_t end = 0;
    const auto value = std::stoll(s, &end);
    if (end != s.size()) throw std::runtime_error("Expected integer: " + s);
    return value;
}

double number(const std::string &s)
{
    std::size_t end = 0;
    const double value = std::stod(s, &end);
    if (end != s.size() || !std::isfinite(value)) throw std::runtime_error("Invalid number: " + s);
    return value;
}
}

std::vector<Metadata> read_metadata(const std::filesystem::path &path)
{
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open frame metadata: " + path.string());
    std::string line;
    if (!std::getline(stream, line)) throw std::runtime_error("Empty frames.csv");
    if (line.compare(0, 3, "\xef\xbb\xbf") == 0) line.erase(0, 3);
    const auto headers = csv_fields(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
        if (!columns.emplace(headers[i], i).second) throw std::runtime_error("Duplicate CSV column");
    for (const auto *key : {"frame_index", "timestamp_ns", "t_recv_ns", "q_w", "q_x", "q_y", "q_z", "mode"})
        if (!columns.count(key)) throw std::runtime_error(std::string("Missing CSV column: ") + key);
    std::vector<Metadata> frames;
    std::size_t line_number = 1;
    while (std::getline(stream, line))
    {
        ++line_number;
        try
        {
            const auto values = csv_fields(line);
            if (values.size() != headers.size()) throw std::runtime_error("CSV column count mismatch");
            const auto value = [&](const char *key) -> const std::string & { return values[columns.at(key)]; };
            Metadata m;
            m.frame_index = integer(value("frame_index"));
            m.timestamp_ns = integer(value("timestamp_ns"));
            m.t_recv_ns = integer(value("t_recv_ns"));
            const auto mode = integer(value("mode"));
            if (mode < 0 || mode > 5) throw std::runtime_error("Invalid mode");
            m.mode = static_cast<int>(mode);
            m.q = {number(value("q_w")), number(value("q_x")), number(value("q_y")), number(value("q_z"))};
            const double norm = std::hypot(std::hypot(m.q[0], m.q[1]), std::hypot(m.q[2], m.q[3]));
            if (!std::isfinite(norm) || norm <= 1e-6) throw std::runtime_error("Invalid quaternion");
            if (m.frame_index != static_cast<int64_t>(frames.size())) throw std::runtime_error("Noncontiguous frame_index");
            if (m.timestamp_ns < 0 || m.t_recv_ns < m.timestamp_ns ||
                (!frames.empty() && m.timestamp_ns <= frames.back().timestamp_ns))
                throw std::runtime_error("Invalid timestamp order or receipt time");
            frames.push_back(m);
        }
        catch (const std::exception &error)
        {
            throw std::runtime_error(path.string() + ":" + std::to_string(line_number) + ": " + error.what());
        }
    }
    if (stream.bad()) throw std::runtime_error("Failed reading metadata");
    if (frames.empty()) throw std::runtime_error("frames.csv has no frames");
    return frames;
}

Player::Player(const std::filesystem::path &video, const std::filesystem::path &csv)
    : frames_(read_metadata(csv)), capture_(video.string())
{
    if (!capture_.isOpened()) throw std::runtime_error("Cannot open video: " + video.string());
}

bool Player::next(PairedFrame &frame)
{
    if (next_ == frames_.size())
    {
        cv::Mat extra;
        if (capture_.read(extra)) throw std::runtime_error("Video has more frames than CSV");
        return false;
    }
    frame.metadata = frames_[next_];
    if (next_ == 0) origin_ = std::chrono::steady_clock::now();
    if (!capture_.read(frame.camera.frame) || frame.camera.frame.empty())
        throw std::runtime_error("Video ended before CSV frame " + std::to_string(next_));
    const auto &m = frame.metadata;
    const auto first = frames_.front().timestamp_ns;
    frame.camera.timestamp = origin_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::nanoseconds(m.timestamp_ns - first));
    frame.camera.t_recv = origin_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::nanoseconds(m.t_recv_ns - first));
    frame.q = Eigen::Quaterniond(m.q[0], m.q[1], m.q[2], m.q[3]);
    if (!std::isfinite(frame.q.norm()) || frame.q.norm() <= 1e-6)
        throw std::runtime_error("Quaternion cannot be normalized at frame " + std::to_string(next_));
    frame.q.normalize();
    ++next_;
    return true;
}
}
