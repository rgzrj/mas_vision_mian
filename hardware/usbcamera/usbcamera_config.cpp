#include "usbcamera_config.hpp"
#include "mas_log.hpp"
#include <sstream>
#include <vector>
#include <yaml-cpp/yaml.h>

// 初始化静态成员变量
bool        UsbCameraConfigManager::enabled_       = true;
std::string UsbCameraConfigManager::device_path_   = "/dev/video0";
int         UsbCameraConfigManager::width_         = 640;
int         UsbCameraConfigManager::height_        = 480;
int         UsbCameraConfigManager::fps_           = 60;
int         UsbCameraConfigManager::auto_exposure_ = 1;
int         UsbCameraConfigManager::exposure_      = 1;
int         UsbCameraConfigManager::gain_          = 100;
int         UsbCameraConfigManager::auto_wb_       = 0;

bool UsbCameraConfigManager::loadConfig(const std::string &config_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["usbcamera"])
        {
            YAML::Node camera_node = config["usbcamera"];

            if (camera_node["enable"])
            {
                enabled_ = camera_node["enable"].as<bool>();
            }

            if (camera_node["device_path"])
            {
                device_path_ = camera_node["device_path"].as<std::string>();
            }

            if (camera_node["width"])
            {
                width_ = camera_node["width"].as<int>();
            }

            if (camera_node["height"])
            {
                height_ = camera_node["height"].as<int>();
            }

            if (camera_node["fps"])
            {
                fps_ = camera_node["fps"].as<int>();
            }

            if (camera_node["auto_exposure"])
            {
                auto_exposure_ = camera_node["auto_exposure"].as<int>();
            }

            if (camera_node["exposure"])
            {
                exposure_ = camera_node["exposure"].as<int>();
            }

            if (camera_node["gain"])
            {
                gain_ = camera_node["gain"].as<int>();
            }

            if (camera_node["auto_wb"])
            {
                auto_wb_ = camera_node["auto_wb"].as<int>();
            }

            MAS_LOG_INFO("usbcamera config loaded successfully");

            return true;
        }
        else
        {
            MAS_LOG_WARN("UsbCameraConfig", "No 'usbcamera' section in config file: {}",
                         config_path.c_str());
            return false;
        }
    }
    catch (const YAML::BadFile &e)
    {
        MAS_LOG_ERROR("UsbCameraConfig", "Failed to load config file: {}, Error: {}",
                      config_path.c_str(), e.what());
        return false;
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("UsbCameraConfig", "Exception loading config file: {}, Error: {}",
                      config_path.c_str(), e.what());
        return false;
    }
}

bool UsbCameraConfigManager::isEnabled()
{
    return enabled_;
}

std::string UsbCameraConfigManager::getDevicePath()
{
    return device_path_;
}

int UsbCameraConfigManager::getWidth()
{
    return width_;
}

int UsbCameraConfigManager::getHeight()
{
    return height_;
}

int UsbCameraConfigManager::getFps()
{
    return fps_;
}

int UsbCameraConfigManager::getAutoExposure()
{
    return auto_exposure_;
}

int UsbCameraConfigManager::getExposure()
{
    return exposure_;
}

int UsbCameraConfigManager::getGain()
{
    return gain_;
}

int UsbCameraConfigManager::getAutoWb()
{
    return auto_wb_;
}