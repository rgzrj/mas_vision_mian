#ifndef _MAS_LOG_H_
#define _MAS_LOG_H_

#include "spdlog/spdlog.h"
#include <memory>
#include <spdlog/common.h>
#include <string>

#define MAS_LOG_TRACE(...)                                                                         \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::trace, __VA_ARGS__)
#define MAS_LOG_DEBUG(...)                                                                         \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::debug, __VA_ARGS__)
#define MAS_LOG_INFO(...)                                                                          \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::info, __VA_ARGS__)
#define MAS_LOG_WARN(...)                                                                          \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::warn, __VA_ARGS__)
#define MAS_LOG_ERROR(...)                                                                         \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::err, __VA_ARGS__)
#define MAS_LOG_CRITICAL(...)                                                                      \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::critical, __VA_ARGS__)

namespace rm_utils
{
class MasLog
{
  public:
    /**
     * @brief 初始化日志系统
     * @param log_path 日志文件保存路径
     * @param filelevel 文件日志的输出级别
     * @param consolelevel 控制台日志的输出级别
     * @note 级别从低到高：trace < debug < info < warn < err < critical < off
     */
    static void init(const std::string        &log_path     = "logs/mas_vision.log",
                     spdlog::level::level_enum filelevel    = spdlog::level::debug,
                     spdlog::level::level_enum consolelevel = spdlog::level::warn);

    /**
     * @brief 获取全局 logger 实例
     * @return std::shared_ptr<spdlog::logger>
     */
    static std::shared_ptr<spdlog::logger> &get_logger()
    {
        return logger_;
    }

  private:
    static std::shared_ptr<spdlog::logger> logger_;
};

} // namespace rm_utils

#endif // _MAS_LOG_H_