#ifndef _USBCAMERA_THREAD_H_
#define _USBCAMERA_THREAD_H_

#include "SPSCQueue.h"
#include "common_def.hpp"
#include <atomic>
#include <memory>

extern std::atomic<bool> g_shutdown;

namespace threads
{
/**
 * @brief USB相机处理线程入口函数
 * @param buffer_size SPSCQueue大小
 * @param buffers 相机对应的SPSCQueue大小
 */
void usb_camera_thread_func(size_t                                           buffer_size,
                            std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> buffer);

} // namespace threads

#endif // _USBCAMERA_THREAD_H_