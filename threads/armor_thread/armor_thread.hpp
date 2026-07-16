#ifndef _ARMOR_THREAD_H_
#define _ARMOR_THREAD_H_

#include "SPSCQueue.h"
#include "common_def.hpp"
#include <atomic>
#include <memory>

extern std::atomic<bool> g_shutdown;

namespace threads
{
void armor_thread_func(std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> hikcamerabuffer,
                       std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> usbcamera_buffer);

} // namespace threads

#endif // _ARMOR_THREAD_H_