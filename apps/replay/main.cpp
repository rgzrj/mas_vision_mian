// Diagnostic transport is captured in memory; the copied production algorithms are unchanged.
#include "armor_detector.hpp"
#include "armor_track.hpp"
#include "mas_log.hpp"
#include "player.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <thread>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <stdexcept>
#include <string>
#include <vector>

using Json = nlohmann::json;
namespace
{
std::vector<Json> packets;
auto_aim::ArmorPose *replay_pose = nullptr;
int update_order = 0;
Json runtime_noise = Json::object();
volatile std::sig_atomic_t stop_requested = 0;
}

// Replace only the UDP transport in this offline executable. No serial/camera threads run.
namespace rm_utils
{
Plotter::Plotter(std::string, uint16_t) : socket_(-1) {}
Plotter::~Plotter() = default;
void Plotter::plot(const Json &data) { packets.push_back(data); }
}

Json pixels(const std::vector<cv::Point2f> &points)
{
    Json result = Json::array();
    for (const auto &p : points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return Json::array();
        result.push_back({p.x, p.y});
    }
    return result;
}

Json fit_error(const std::vector<cv::Point2f> &observed, const Eigen::Vector4d &model,
               auto_aim::ArmorType type, const std::string &name)
{
    Json result{{"dx", nullptr}, {"dy", nullptr}, {"center_px", nullptr}, {"corner_rms_px", nullptr}};
    const auto projected = replay_pose->reproject_armor(model.head<3>(), model[3], type, name);
    if (observed.size() != 4 || projected.size() != 4) return result;
    cv::Point2d delta(0, 0);
    double squared_error = 0;
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2d d = cv::Point2d(projected[i]) - cv::Point2d(observed[i]);
        if (!std::isfinite(d.x) || !std::isfinite(d.y)) return result;
        delta += d;
        squared_error += d.dot(d);
    }
    delta *= 0.25;
    return Json{{"dx", delta.x}, {"dy", delta.y}, {"center_px", cv::norm(delta)},
                {"corner_rms_px", std::sqrt(squared_error / 4)}};
}

namespace auto_aim
{
double replay_noise(const char *key, double fallback) { return runtime_noise.value(key, fallback); }
double replay_r_scale(int i)
{
    return runtime_noise.contains("r_scale") ? runtime_noise["r_scale"][i].get<double>() : 1.0;
}

// Called only by the instrumented temporary source copy. The production file stays untouched.
void replay_observation_update(const Armor &armor, const Target &target, const Eigen::VectorXd &prior,
                               const Eigen::Vector4d &predicted, bool applied)
{
    if (target.armor_num != 4) return;
    const auto posterior = target.ekf_x();
    const auto models = target.armor_xyza_list();
    Json row{{"order", update_order++}, {"slot_id", target.last_id}, {"number", armor.number},
             {"confidence", armor.confidence}, {"trial_update_applied", applied},
             {"pnp", {{"x", armor.xyz_in_world[0]}, {"y", armor.xyz_in_world[1]},
                      {"z", armor.xyz_in_world[2]}, {"yaw", armor.ypr_in_world[0]}}},
             {"innovation_z", target.last_innovation_z()},
             {"innovation_norm", target.last_innovation_norm()},
             {"fit_before", fit_error(armor.points, predicted, armor.type, armor.number)},
             {"fit_after", fit_error(armor.points, models.at(target.last_id), armor.type, armor.number)}};
    const char *names[] = {"x", "vx", "y", "vy", "z", "vz", "yaw", "w", "r02", "r13", "h"};
    for (int i = 0; i < 11; ++i)
    {
        row["before"][names[i]] = prior[i];
        row["after"][names[i]] = posterior[i];
    }
    for (int i = 0; i < static_cast<int>(armor.points.size()); ++i)
        row["pixels"]["p" + std::to_string(i)] = {{"x", armor.points[i].x}, {"y", armor.points[i].y}};
    packets.push_back({{"replay_update", row}});
}
}

Json process_frame(auto_aim::ArmorDetector &detector, auto_aim::ArmorTrack &tracker, const replay_app::PairedFrame &frame)
{
    cv::Mat image = frame.camera.frame;
    const auto timestamp = frame.camera.timestamp;
    const auto &q = frame.q;
    const int mode = frame.metadata.mode;
    packets.clear();
    update_order = 0;
    Json response{{"frame_index", frame.metadata.frame_index}, {"models", Json::array()},
                  {"center", nullptr}, {"observations", Json::array()}};
    if (mode > 1)
    {
        response["packets"] = Json::array({Json{{"state", "mode_skipped"}}});
        return response;
    }
    detector.ArmorDetector_Set_Color(mode == 0 ? auto_aim::EnemyColor::RED : auto_aim::EnemyColor::BLUE);
    auto armors = detector.ArmorDetect(image);
    tracker.armor_pose().set_R_gimbal2world(q);
    const auto target = tracker.track(armors, timestamp, "Replay", image);
    for (const auto &armor : armors)
        response["observations"].push_back({{"number", armor.number}, {"points", pixels(armor.points)}});
    if (target)
    {
        const auto models = target->armor_xyza_list();
        for (int id = 0; id < static_cast<int>(models.size()); ++id)
            response["models"].push_back({{"id", id}, {"tracked", id == target->last_id},
                {"points", pixels(tracker.armor_pose().reproject_armor(
                    models[id].head<3>(), models[id][3], target->armor_type, target->name))}});
        const auto x = target->ekf_x();
        const std::vector<cv::Point3f> center{{static_cast<float>(x[0]), static_cast<float>(x[2]),
                                              static_cast<float>(x[4])}};
        const auto projected = pixels(tracker.armor_pose().world2pixel(center));
        if (!projected.empty()) response["center"] = projected[0];
    }
    bool frame_accepted = false;
    for (const auto &packet : packets)
        if (packet.contains("measurement_accepted"))
            frame_accepted = packet.at("measurement_accepted").get<bool>();
    for (auto &packet : packets)
    {
        if (!packet.contains("replay_update")) continue;
        auto &row = packet["replay_update"];
        row["frame_measurement_accepted"] = frame_accepted;
        row["final_target_present"] = target.has_value();
        row["fit_final"] = {{"dx", nullptr}, {"dy", nullptr},
                             {"center_px", nullptr}, {"corner_rms_px", nullptr}};
        if (!target || target->armor_num != 4 || row["pixels"].size() != 4) continue;
        std::vector<cv::Point2f> observed;
        for (int i = 0; i < 4; ++i)
        {
            const auto &p = row["pixels"]["p" + std::to_string(i)];
            observed.emplace_back(p["x"].get<float>(), p["y"].get<float>());
        }
        const int id = row["slot_id"].get<int>();
        row["fit_final"] = fit_error(observed, target->armor_xyza_list().at(id),
                                      target->armor_type, target->name);
    }
    response["packets"] = packets;
    return response;
}

namespace fs = std::filesystem;
namespace
{
struct Settings
{
    fs::path video, frames_csv, auto_aim_yaml, camera_yaml, output_dir;
    double speed = 1;
    bool display = true;
    Json noise = Json::object();
};

Settings load_settings(const fs::path &path)
{
    const auto node = YAML::LoadFile(path.string())["replay"];
    if (!node.IsMap()) throw std::runtime_error("Missing replay mapping");
    const auto resolve = [&](const char *key, bool input = true) {
        const auto value = node[key].as<std::string>();
        if (value.empty()) throw std::runtime_error(std::string("Empty replay.") + key);
        const auto result = fs::absolute(path.parent_path() / value).lexically_normal();
        if (input && !fs::is_regular_file(result)) throw std::runtime_error("Missing input: " + result.string());
        return result;
    };
    Settings s;
    s.video = resolve("video");
    s.frames_csv = resolve("frames_csv");
    s.auto_aim_yaml = resolve("auto_aim_yaml");
    s.camera_yaml = resolve("camera_yaml");
    s.output_dir = resolve("output_dir", false);
    s.speed = node["speed"].as<double>(1);
    s.display = node["display"].as<bool>(true);
    if (!std::isfinite(s.speed) || s.speed < 0) throw std::runtime_error("speed must be finite and nonnegative");
    if (const auto noise = node["ekf"])
    {
        if (!noise.IsMap()) throw std::runtime_error("ekf must be a mapping");
        for (const auto &entry : noise)
        {
            const auto key = entry.first.as<std::string>();
            if (key == "r_scale")
            {
                if (!entry.second.IsSequence() || entry.second.size() != 4)
                    throw std::runtime_error("r_scale needs four positive numbers");
                s.noise[key] = Json::array();
                for (const auto &item : entry.second)
                {
                    const double v = item.as<double>();
                    if (!std::isfinite(v) || v <= 0) throw std::runtime_error("r_scale needs four positive numbers");
                    s.noise[key].push_back(v);
                }
            }
            else if (key == "v1" || key == "v2" || key == "q_h")
            {
                const double v = entry.second.as<double>();
                if (!std::isfinite(v) || v < 0) throw std::runtime_error("EKF noise must be finite and nonnegative");
                s.noise[key] = v;
            }
            else throw std::runtime_error("Unknown EKF parameter: " + key);
        }
    }
    return s;
}

void write_text(const fs::path &path, const std::string &text)
{
    std::ofstream file(path);
    file << text;
    file.close();
    if (!file) throw std::runtime_error("Failed writing: " + path.string());
}

void prepare_configs(const Settings &s, const fs::path &output)
{
    auto config = YAML::LoadFile(s.auto_aim_yaml.string());
    for (const auto *name : {"armor_detector", "armor_track"})
    {
        auto node = config["auto_aim"][name];
        node["plotter_enable"] = true;
        for (const auto *key : {"debug", "armor_track_debug", "armor_pose_debug"})
            if (node[key]) node[key] = false;
    }
    auto numbers = config["auto_aim"]["armor_detector"]["number_params"];
    for (const auto *key : {"model_path", "label_path"})
    {
        const auto path = fs::absolute(s.auto_aim_yaml.parent_path() / numbers[key].as<std::string>()).lexically_normal();
        if (!fs::is_regular_file(path)) throw std::runtime_error("Missing classifier input: " + path.string());
        numbers[key] = path.string();
    }
    const auto camera = YAML::LoadFile(s.camera_yaml.string());
    const auto validate = [&](const char *group, const char *key, std::size_t size) {
        const auto values = camera[group][key];
        if (!values.IsSequence() || values.size() != size) throw std::runtime_error(std::string("Invalid camera ") + key);
        for (const auto &value : values)
            if (!std::isfinite(value.as<double>())) throw std::runtime_error(std::string("Nonfinite camera ") + key);
    };
    validate("calibration", "camera_matrix", 9);
    validate("calibration", "distort_coeffs", 5);
    validate("handeye_calibration", "R_gimbal2imubody", 9);
    validate("handeye_calibration", "R_camera2gimbal", 9);
    validate("handeye_calibration", "t_camera2gimbal", 3);
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter << config;
    if (!emitter.good()) throw std::runtime_error("Cannot serialize replay configuration");
    write_text(output / "auto_aim.yaml", emitter.c_str());
    fs::copy_file(s.camera_yaml, output / "hikcamera.yaml");
}

void flatten(const Json &input, Json &output, const std::string &prefix = "")
{
    for (auto item = input.begin(); item != input.end(); ++item)
    {
        if (item.value().is_object()) flatten(item.value(), output, prefix + item.key() + ".");
        else output[prefix + item.key()] = item.value();
    }
}

std::string csv_cell(const Json &value)
{
    if (value.is_null()) return "";
    if (value.is_boolean()) return value.get<bool>() ? "1.0" : "0.0";
    const std::string text = value.is_string() ? value.get<std::string>() : value.dump();
    if (text.find_first_of(",\"\r\n") == std::string::npos) return text;
    std::string result = "\"";
    for (char c : text) { if (c == '"') result += '"'; result += c; }
    return result + '"';
}

class CsvRecorder
{
  public:
    explicit CsvRecorder(fs::path directory) : directory_(std::move(directory)) { fs::create_directories(directory_); }
    void write(const Json &packet, const replay_app::Metadata &frame, int64_t first_ns)
    {
        Json row = Json::object();
        flatten(packet, row);
        row["frame_index"] = frame.frame_index;
        row["timestamp_ns"] = frame.timestamp_ns;
        row["t_recv_ns"] = frame.t_recv_ns;
        row["t_recorded"] = (frame.timestamp_ns - first_ns) / 1e9;
        std::vector<std::string> fields;
        for (auto item = row.begin(); item != row.end(); ++item) fields.push_back(item.key());
        auto found = streams_.find(fields);
        if (found == streams_.end())
        {
            const auto path = directory_ / ("plot_s" + std::to_string(streams_.size()) + ".csv");
            auto file = std::make_unique<std::ofstream>(path);
            if (!*file) throw std::runtime_error("Cannot create CSV: " + path.string());
            for (std::size_t i = 0; i < fields.size(); ++i) *file << (i ? "," : "") << csv_cell(fields[i]);
            *file << '\n';
            found = streams_.emplace(fields, std::move(file)).first;
            std::cout << "CSV: " << path << '\n';
        }
        auto &file = *found->second;
        for (std::size_t i = 0; i < fields.size(); ++i) file << (i ? "," : "") << csv_cell(row[fields[i]]);
        file << '\n';
        if (!file) throw std::runtime_error("CSV write failed");
    }
    void close()
    {
        for (auto &entry : streams_)
        {
            entry.second->close();
            if (!*entry.second) throw std::runtime_error("CSV flush failed");
        }
    }
  private:
    fs::path directory_;
    std::map<std::vector<std::string>, std::unique_ptr<std::ofstream>> streams_;
};

void draw_frame(cv::Mat image, const Json &response)
{
    for (const auto &model : response["models"])
    {
        const auto &points = model["points"];
        if (points.size() != 4) continue;
        const cv::Scalar color = model["tracked"].get<bool>() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        const auto point = [&](int i) { return cv::Point(static_cast<int>(points[i][0].get<double>()), static_cast<int>(points[i][1].get<double>())); };
        for (int i = 0; i < 4; ++i) cv::line(image, point(i), point((i + 1) % 4), color, 2);
        cv::putText(image, "M" + std::to_string(model["id"].get<int>()), point(0), cv::FONT_HERSHEY_SIMPLEX, .7, color, 2);
    }
    if (!response["center"].is_null())
        cv::circle(image, cv::Point(static_cast<int>(response["center"][0].get<double>()),
                                  static_cast<int>(response["center"][1].get<double>())), 5, cv::Scalar(255, 0, 255), -1);
    std::string state;
    for (const auto &packet : response["packets"])
        if (packet.contains("state")) state = packet["state"].get<std::string>();
    cv::putText(image, "frame " + std::to_string(response["frame_index"].get<int64_t>()) + "  " + state +
        "  [space]pause [n]step [q]quit", cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, .65, cv::Scalar(0, 255, 255), 2);
    if (image.cols > 1280) cv::resize(image, image, cv::Size(1280, image.rows * 1280 / image.cols));
    cv::imshow("Replay", image);
}

bool wait_frame(double delay, bool display, bool &paused)
{
    using Clock = std::chrono::steady_clock;
    auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(std::max(0.0, delay)));
    while (!stop_requested)
    {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        const int ms = paused ? 25 : static_cast<int>(std::max<int64_t>(1, std::min<int64_t>(25, remaining)));
        int key = -1;
        if (display) key = cv::waitKey(ms) & 255;
        else if (remaining > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        if (key == 27 || key == 'q') return true;
        if (key == ' ') { paused = !paused; deadline = Clock::now(); }
        if (paused && key == 'n') return false;
        if (!paused && Clock::now() >= deadline) return false;
    }
    return true;
}

fs::path new_output(const fs::path &base)
{
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream name;
    name << std::put_time(std::localtime(&t), "run_%Y%m%d_%H%M%S_")
         << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
    const auto output = base / name.str();
    fs::create_directories(base);
    if (!fs::create_directory(output)) throw std::runtime_error("Output directory already exists");
    return output;
}
}

int main(int argc, char **argv)
{
    fs::path config_path = "config/replay.yaml", output;
    std::size_t processed = 0, input_frames = 0;
    double processing_seconds = 0;
    bool headless = false, check = false, paused = false, stopped = false;
    const auto summary = [&](bool completed) {
        return Json{{"input_frames", input_frames}, {"processed_frames", processed}, {"completed", completed},
                    {"processing_fps", processing_seconds > 0 ? processed / processing_seconds : 0}};
    };
    try
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            if (argument == "--config" && i + 1 < argc) config_path = argv[++i];
            else if (argument == "--headless") headless = true;
            else if (argument == "--check") check = true;
            else if (argument == "--help" || argument == "-h")
            {
                std::cout << "Usage: replay --config replay.yaml [--headless] [--check]\n";
                return 0;
            }
            else throw std::runtime_error("Unknown or incomplete argument: " + argument);
        }
        config_path = fs::absolute(config_path);
        auto settings = load_settings(config_path);
        if (headless) settings.display = false;
        runtime_noise = settings.noise;
        replay_app::Player player(settings.video, settings.frames_csv);
        input_frames = player.frames().size();
        replay_app::PairedFrame frame;
        if (check)
        {
            while (player.next(frame)) ++processed;
            std::cout << "Pairing check passed: " << processed << " frames\n";
            return 0;
        }
        output = new_output(settings.output_dir);
        std::cout << "Output: " << output << std::endl;
        fs::copy_file(config_path, output / "replay.yaml");
        fs::copy_file(settings.frames_csv, output / "frames.csv");
        write_text(output / "parameter_overrides.json", settings.noise.dump(2));
        write_text(output / "inputs.json", Json{{"video", settings.video.string()}, {"frames_csv", settings.frames_csv.string()},
            {"auto_aim_yaml", settings.auto_aim_yaml.string()}, {"camera_yaml", settings.camera_yaml.string()},
            {"speed", settings.speed}, {"display", settings.display}, {"ekf", settings.noise}}.dump(2));
        for (const auto *file : {"source_manifest.json", "replay_instrumentation.patch"})
            fs::copy_file(fs::path(REPLAY_BUILD_INFO) / file, output / file);
        prepare_configs(settings, output);
        // Match the old backend's isolated working directory for relative debug/config lookups.
        fs::current_path(output);
        rm_utils::MasLog::get_logger() = spdlog::basic_logger_mt("replay", (output / "backend.log").string());
        rm_utils::MasLog::get_logger()->set_level(spdlog::level::warn);
        rm_utils::MasLog::get_logger()->flush_on(spdlog::level::warn);
        auto_aim::ArmorDetector detector((output / "auto_aim.yaml").string());
        auto_aim::ArmorTrack tracker((output / "auto_aim.yaml").string(), (output / "hikcamera.yaml").string());
        replay_pose = &tracker.armor_pose();
        CsvRecorder recorder(output / "csv");
        std::signal(SIGINT, [](int) { stop_requested = 1; });
        std::signal(SIGTERM, [](int) { stop_requested = 1; });
        std::cout << "OpenCV thread setting=" << cv::getNumThreads() << "; runtime EKF overrides=" << runtime_noise.dump() << std::endl;
        while (!stop_requested)
        {
            const auto begin = std::chrono::steady_clock::now();
            if (!player.next(frame)) break;
            const auto response = process_frame(detector, tracker, frame);
            for (const auto &packet : response["packets"])
                recorder.write(packet, frame.metadata, player.frames().front().timestamp_ns);
            ++processed;
            if (settings.display) draw_frame(frame.camera.frame, response);
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
            processing_seconds += elapsed;
            if (processed % 120 == 0)
                std::cout << "Replay: " << processed << '/' << input_frames << " frames; processing "
                          << processed / processing_seconds << " FPS" << std::endl;
            double delay = 0;
            if (settings.speed > 0 && processed < input_frames)
                delay = (player.frames()[processed].timestamp_ns - frame.metadata.timestamp_ns) / 1e9 / settings.speed - elapsed;
            if (wait_frame(delay, settings.display, paused)) { stopped = true; break; }
        }
        recorder.close();
        if (settings.display) cv::destroyAllWindows();
        const auto result = summary(processed == input_frames && !stopped && !stop_requested);
        write_text(output / "summary.json", result.dump(2));
        std::cout << result.dump(2) << "\nOutput: " << output << std::endl;
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Replay error: " << error.what() << '\n';
        if (!output.empty())
        {
            try
            {
                auto failed = summary(false);
                failed["error"] = error.what();
                write_text(output / "summary.json", failed.dump(2));
                std::ofstream log(output / "backend.log", std::ios::app);
                log << error.what() << '\n';
            }
            catch (...) { std::cerr << "Could not save failure summary\n"; }
        }
        return 1;
    }
}
