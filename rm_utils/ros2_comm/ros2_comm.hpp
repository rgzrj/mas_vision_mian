#ifndef _ROS2_COMM_H_
#define _ROS2_COMM_H_

#include <cstdint>
#include <string>
#include <vector>
#include <netinet/in.h> 

struct __attribute__((packed)) ROS2_SEND_PACKET
{
    uint16_t projectile_allowance_17mm;
    uint8_t  power_management_shooter_output;
    uint16_t current_hp;
    uint16_t outpost_HP; 
    uint16_t base_HP;
    uint8_t  game_progess;
};

struct __attribute__((packed)) ROS2_RECV_PACKET
{
    float vx;
    float vy;
    uint8_t nav_state;
};

// 帧格式定义 
constexpr uint8_t SEND_FRAME_HEADER = 0xAA;
constexpr uint8_t SEND_FRAME_TAIL   = 0x5A;
constexpr uint8_t RECV_FRAME_HEADER = 0xBB;
constexpr uint8_t RECV_FRAME_TAIL   = 0x5B;
constexpr size_t  MAX_DATA_LENGTH   = 255;
constexpr size_t  RECV_BUFFER_SIZE  = 1024;

class Ros2_comm
{
public:
    Ros2_comm();
    ~Ros2_comm();

    // 禁用拷贝
    Ros2_comm(const Ros2_comm&) = delete;
    Ros2_comm& operator=(const Ros2_comm&) = delete;

    static bool loadConfig(const std::string& config_path);
    static bool isEnabled();
    static std::string getServerIp() { return s_server_ip; }
    static int getServerPort() { return s_server_port; }
    static int getLocalPort() { return s_local_port; }

    /**
     * @brief 初始化 UDP Socket (绑定本地端口)
     * @return 0成功，-1失败
     */
    int init();

    /**
     * @brief 发送数据 (发送到 Docker)
     * @return 0成功，-1失败
     */
    int send(const ROS2_SEND_PACKET& pkt);

    /**
     * @brief 尝试接收数据 (非阻塞)
     * @return >0成功收到数据，0无数据，-1错误
     */
    int recv(ROS2_RECV_PACKET& out_pkt);

    /**
     * @brief 关闭
     */
    void close();

    /**
     * @brief 检查是否初始化
     */
    bool is_open() const { return m_sock_fd >= 0; }

private:
    // 静态配置
    static bool        s_enabled;
    static std::string s_server_ip;   // Docker 的 IP (host模式下是 127.0.0.1)
    static int         s_server_port; // Docker 监听的端口
    static int         s_local_port;  // 宿主机绑定的端口

    // 实例成员
    int         m_sock_fd;
    sockaddr_in m_server_addr; // 目标地址 (Docker)
    std::vector<uint8_t> m_recv_buffer;
};

#endif // _ROS2_COMM_H_