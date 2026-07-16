#ifndef _ROS2_THREAD_H_
#define _ROS2_THREAD_H_

#include <atomic>
#include "user_serial.hpp"

// 声明全局 shutdown 变量（在 main.cpp 中定义）
extern std::atomic<bool> g_shutdown;

namespace threads
{
/**
 * @brief ROS2 通信线程函数
 * 负责与 Docker 中的 ROS2 节点进行通信
 * 
 * @param user_serial UserSerial 单例对象
 */
void ros2_thread_func(UserSerial& user_serial);

} // namespace threads

#endif // _ROS2_THREAD_H_
