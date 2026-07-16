#include "ros2_comm.hpp"
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include "mas_log.hpp"
#include <yaml-cpp/yaml.h>

// 静态成员初始化
bool        Ros2_comm::s_enabled     = true;
std::string Ros2_comm::s_server_ip   = "127.0.0.1";
int         Ros2_comm::s_server_port = 8888; // Docker 端端口
int         Ros2_comm::s_local_port  = 8889; // 宿主机端口

Ros2_comm::Ros2_comm()
    : m_sock_fd(-1)
{
    m_recv_buffer.reserve(RECV_BUFFER_SIZE);
    memset(&m_server_addr, 0, sizeof(m_server_addr));
}

Ros2_comm::~Ros2_comm()
{
    close();
}

bool Ros2_comm::loadConfig(const std::string& config_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["ros2"])
        {
            YAML::Node ros2_node = config["ros2"];
            if (ros2_node["enabled"]) s_enabled = ros2_node["enabled"].as<bool>();
            if (ros2_node["server_ip"]) s_server_ip = ros2_node["server_ip"].as<std::string>();
            if (ros2_node["server_port"]) s_server_port = ros2_node["server_port"].as<int>();
            if (ros2_node["local_port"]) s_local_port = ros2_node["local_port"].as<int>();
            
            MAS_LOG_INFO("ROS2 UDP config loaded: enabled={}, server={}:{}, local_port={}", 
                         s_enabled ? "true" : "false", 
                         s_server_ip.c_str(), s_server_port, s_local_port);
            return true;
        }
        return false;
    }
    catch (...)
    {
        MAS_LOG_ERROR("Failed to load ros2 config");
        return false;
    }
}

bool Ros2_comm::isEnabled()
{
    return s_enabled;
}

void Ros2_comm::close()
{
    if (m_sock_fd >= 0)
    {
        ::close(m_sock_fd);
        m_sock_fd = -1;
    }
    m_recv_buffer.clear();
}

int Ros2_comm::init()
{
    close();

    // 创建 UDP Socket
    m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sock_fd < 0)
    {
        MAS_LOG_ERROR("Failed to create UDP socket: {}", strerror(errno));
        return -1;
    }

    // 设置为非阻塞
    int flags = fcntl(m_sock_fd, F_GETFL, 0);
    fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);

    // 绑定本地端口 (用于接收 Docker 发回来的数据)
    sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(s_local_port);

    if (bind(m_sock_fd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0)
    {
        MAS_LOG_ERROR("Failed to bind UDP port {}: {}", s_local_port, strerror(errno));
        close();
        return -1;
    }

    // 设置目标地址 (Docker)
    memset(&m_server_addr, 0, sizeof(m_server_addr));
    m_server_addr.sin_family = AF_INET;
    m_server_addr.sin_port = htons(s_server_port);
    if (inet_pton(AF_INET, s_server_ip.c_str(), &m_server_addr.sin_addr) <= 0)
    {
        MAS_LOG_ERROR("Invalid server IP: %s", s_server_ip.c_str());
        close();
        return -1;
    }

    MAS_LOG_INFO("UDP initialized successfully. Local port: {}, Target: {}:{}", 
                 s_local_port, s_server_ip.c_str(), s_server_port);
    return 0;
}

int Ros2_comm::send(const ROS2_SEND_PACKET& pkt)
{
    if (m_sock_fd < 0) return -1;

    // 组装帧 (保持协议一致)
    uint8_t frame[sizeof(ROS2_SEND_PACKET) + 3];
    frame[0] = SEND_FRAME_HEADER;
    frame[1] = static_cast<uint8_t>(sizeof(ROS2_SEND_PACKET));
    memcpy(frame + 2, &pkt, sizeof(ROS2_SEND_PACKET));
    frame[2 + sizeof(ROS2_SEND_PACKET)] = SEND_FRAME_TAIL;

    // UDP 发送
    int n = sendto(m_sock_fd, frame, sizeof(frame), 0, 
                   (sockaddr*)&m_server_addr, sizeof(m_server_addr));
    
    if (n < 0)
    {
        return -1;
    }
    return 0;
}

int Ros2_comm::recv(ROS2_RECV_PACKET& out_pkt)
{
    if (m_sock_fd < 0) return -1;

    uint8_t temp[RECV_BUFFER_SIZE];
    sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    // UDP 接收
    int n = recvfrom(m_sock_fd, temp, sizeof(temp), 0, 
                     (sockaddr*)&from_addr, &from_len);
    
    if (n > 0)
    {
        if (n < 3) return 0;
        
        // 检查帧头
        if (temp[0] != RECV_FRAME_HEADER) return 0;
        
        uint8_t data_len = temp[1];
        size_t total_len = 2 + data_len + 1;
        
        if (n < static_cast<int>(total_len)) return 0;
        if (temp[2 + data_len] != RECV_FRAME_TAIL) return 0;
        
        if (data_len == sizeof(ROS2_RECV_PACKET))
        {
            memcpy(&out_pkt, &temp[2], sizeof(ROS2_RECV_PACKET));
            return static_cast<int>(data_len);
        }
    }
    else if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0; // 无数据
        }
        return -1;
    }

    return 0;
}