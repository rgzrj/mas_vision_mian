#ifndef _USBCAMERA_CONFIG_H_
#define _USBCAMERA_CONFIG_H_

#include <string>

/**
 * @brief USB相机配置管理器
 * 从YAML文件读取USB相机相关配置
 */
class UsbCameraConfigManager
{
  public:
    /**
     * @brief 加载配置文件
     * @param config_path 配置文件路径
     * @return 是否加载成功
     */
    static bool loadConfig(const std::string &config_path);
    /**
     * @brief 是否启用USB相机
     * @return 是否启用USB相机
     */
    static bool isEnabled();
    /**
     * @brief 获取设备路径
     * @return 设备路径
     */
    static std::string getDevicePath();
    /**
     * @brief 获取图像宽度
     * @return 图像宽度
     */
    static int getWidth();
    /**
     * @brief 获取图像高度
     * @return 图像高度
     */
    static int getHeight();
    /**
     * @brief 获取帧率
     * @return 帧率
     */
    static int getFps();
    /**
     * @brief 获取自动曝光
     * @return 自动曝光
     */
    static int getAutoExposure();
    /**
     * @brief 获取曝光时间
     * @return 曝光时间
     */
    static int getExposure();
    /**
     * @brief 获取增益
     * @return 增益
     */
    static int getGain();
    /**
     * @brief 获取自动白平衡
     * @return 自动白平衡
     */
    static int getAutoWb();

  private:
    static bool        enabled_;
    static std::string device_path_;
    static int         width_;
    static int         height_;
    static int         fps_;
    static int         auto_exposure_;
    static int         exposure_;
    static int         gain_;
    static int         auto_wb_;
};

#endif // _USBCAMERA_CONFIG_H_