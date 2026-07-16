#include "serial_thread.hpp"
#include "mas_log.hpp"
#include "user_serial.hpp"
#include <thread>

namespace threads
{
void serial_thread_func()
{
    MAS_LOG_INFO("Serial processing thread started");

    UserSerial &serial = UserSerial::getInstance();
    
    auto       last_send_time = std::chrono::steady_clock::now();
    const auto send_interval  = std::chrono::milliseconds(4);

    while (!g_shutdown)
    {
        auto current_time = std::chrono::steady_clock::now();
        // 发送数据,固定250hz
        if (current_time - last_send_time >= send_interval)
        {
            serial.sendPacketToSerial();
            last_send_time = current_time;
        }
        // 接收数据处理
        serial.receiveSerial();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    serial.closeSerial();
    MAS_LOG_INFO("Serial processing thread exited");
}
} // namespace threads