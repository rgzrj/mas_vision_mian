#include "mas_log.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

namespace rm_utils
{

std::shared_ptr<spdlog::logger> MasLog::logger_ = nullptr;

void MasLog::init(const std::string &log_path, spdlog::level::level_enum filelevel,
                  spdlog::level::level_enum consolelevel)
{
    try
    {
        // 生成带时间戳的文件名
        std::filesystem::path p(log_path);
        auto                  parent_path = p.parent_path();
        if (!parent_path.empty() && !std::filesystem::exists(parent_path))
        {
            std::filesystem::create_directories(parent_path);
        }

        auto              now       = std::chrono::system_clock::now();
        auto              in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H:%M");

        std::string filename       = p.stem().string() + "_" + ss.str() + p.extension().string();
        std::string final_log_path = (parent_path / filename).string();

        // 初始化线程池（用于异步日志）
        spdlog::init_thread_pool(65536, 1);

        // 创建控制台输出汇 (Color Console)，控制台级别
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
        console_sink->set_level(consolelevel);

        // 创建基础文件输出汇，文件级别
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>(final_log_path, true);
        file_sink->set_level(filelevel);

        // 汇集两个输出汇
        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};

        // 5. 创建异步 Logger
        logger_ = std::make_shared<spdlog::async_logger>(
            "mas_logger", sinks.begin(), sinks.end(), spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest);

        // 6. 设置 Logger 的总体级别：取文件和控制台中较低的级别（更详细的那个）
        auto min_level = (filelevel < consolelevel) ? filelevel : consolelevel;
        logger_->set_level(min_level);
        // 格式优化: [时间] [等级] [文件:行号] 内容
        logger_->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%s:%#] %v");
        // 刷新策略：每隔3秒自动刷新，或遇到 warn 级别立即刷新
        spdlog::flush_every(std::chrono::seconds(3));
        logger_->flush_on(spdlog::level::warn);
        // 注册为全局默认 logger
        spdlog::set_default_logger(logger_);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::fprintf(stderr, "Log initialization failed: %s\n", ex.what());
    }
}

} // namespace rm_utils
