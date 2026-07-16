#include "ros2_thread.hpp"
#include "mas_log.hpp"
#include <chrono>
#include <thread>
#include "ros2_comm.hpp"
#include "user_serial.hpp"

namespace threads
{
void ros2_thread_func(UserSerial &user_serial)
{
    MAS_LOG_INFO("ROS2 UDP communication thread started");

    Ros2_comm ros2_comm;

    if (!Ros2_comm::isEnabled())
    {
        MAS_LOG_INFO("ROS2 communication is disabled");
        return;
    }

    bool is_initialized = false;
    bool was_connected = false; // 这里的 connected 代表“收到过数据”
    auto last_init_attempt = std::chrono::steady_clock::now();
    const int INIT_INTERVAL_MS = 2000;

    while (!g_shutdown.load())
    {
        // 初始化 UDP (如果没初始化)
        if (!is_initialized)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_init_attempt).count();

            if (elapsed >= INIT_INTERVAL_MS)
            {
                last_init_attempt = now;
                MAS_LOG_INFO("Initializing UDP socket...");
                
                if (ros2_comm.init() == 0)
                {
                    is_initialized = true;
                    MAS_LOG_INFO("UDP socket initialized");
                }
            }
        }

        // 收发数据 (如果初始化了)
        if (is_initialized)
        {
            // 发送数据 
            RefereePacket referee_data = user_serial.getRefereeData();
            ROS2_SEND_PACKET send_pkt;
            std::memcpy(&send_pkt, &referee_data, sizeof(send_pkt));
            ros2_comm.send(send_pkt); 
            
            // 接收数据
            ROS2_RECV_PACKET recv_pkt;
            int recv_ret = ros2_comm.recv(recv_pkt);

            if (recv_ret > 0)
            {
                user_serial.sendNav(recv_pkt.vx, recv_pkt.vy, recv_pkt.nav_state);

                if (!was_connected)
                {
                    MAS_LOG_INFO("Receiving data from ROS2 (Link is UP)");
                    was_connected = true;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ros2_comm.close();
    MAS_LOG_INFO("ROS2 communication thread stopped");
}

} // namespace threads