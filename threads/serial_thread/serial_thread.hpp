#ifndef _SERIAL_THREAD_H_
#define _SERIAL_THREAD_H_

#include <atomic>
extern std::atomic<bool> g_shutdown;

namespace threads
{
/**
 * @brief 串口处理线程入口函数
 */
void serial_thread_func();

} // namespace threads

#endif // _SERIAL_THREAD_H_