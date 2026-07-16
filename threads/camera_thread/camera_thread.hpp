#ifndef _CAMERA_THREAD_H_
#define _CAMERA_THREAD_H_

#include "SPSCQueue.h"
#include "common_def.hpp"
#include <atomic>
#include <memory>

extern std::atomic<bool> g_shutdown;

namespace threads
{
/**
 * @brief 相机处理线程入口函数
 * @param config_path 相机配置文件路径
 * @param buffer_size SPSCQueue大小
 * @param buffer SPSCQueue指针
 */
void camera_thread_func(const std::string &config_path, size_t buffer_size,
                        std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> buffer);

} // namespace threads

#endif // _CAMERA_THREAD_H_