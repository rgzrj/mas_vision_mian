#!/bin/bash

# 自定义串口设备udev规则设置脚本
# 该脚本会自动检测连接的串口设备并创建固定的设备名称

echo "视觉串口设备udev规则设置脚本（PCCOMM-AIM）"
echo "========================="

# 检查是否以root权限运行
if [[ $EUID -eq 0 ]]; then
   echo "警告: 此脚本不应以root身份运行，将以普通用户权限运行" 
fi

# 检测当前连接的串口设备
echo "检测当前串口设备..."
CURRENT_DEVICES=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)

if [ -z "$CURRENT_DEVICES" ]; then
    echo "未检测到串口设备，请插入设备后再运行此脚本"
    exit 1
else
    echo "检测到以下串口设备:"
    echo "$CURRENT_DEVICES"
fi

# 询问用户想要为哪个设备创建固定名称
echo ""
echo "请输入要创建固定名称的设备路径 (例如: /dev/ttyACM0):"
read DEVICE_PATH

if [ ! -e "$DEVICE_PATH" ]; then
    echo "错误: 设备 $DEVICE_PATH 不存在"
    exit 1
fi

# 只提醒端口占用，不关闭串口或结束进程。
if command -v fuser >/dev/null 2>&1; then
    if sudo fuser -- "$DEVICE_PATH" >/dev/null 2>&1; then
        echo "提醒：$DEVICE_PATH 已被进程占用，可能是视觉或导航程序："
        sudo fuser -v -- "$DEVICE_PATH"
        echo "本脚本不会释放占用；重新插拔或启动程序前，请先停止对应程序。"
    fi
else
    echo "提醒：未找到 fuser，无法检查端口占用；需要时安装 psmisc。"
fi

# 获取设备的属性信息
echo "获取设备属性信息..."

# 获取供应商ID和产品ID（使用 udev 导出的环境变量 ID_VENDOR_ID / ID_MODEL_ID）
VENDOR_ID=$(udevadm info -n "$DEVICE_PATH" -q property | sed -n 's/^ID_VENDOR_ID=//p')
PRODUCT_ID=$(udevadm info -n "$DEVICE_PATH" -q property | sed -n 's/^ID_MODEL_ID=//p')

if [ -z "$VENDOR_ID" ] || [ -z "$PRODUCT_ID" ]; then
    echo "无法获取设备的 ID_VENDOR_ID 和 ID_MODEL_ID"
    exit 1
fi

# 读取 USB 的 iInterface 字符串，不使用设备产品名称。
INTERFACE=$(udevadm info --attribute-walk --name="$DEVICE_PATH" | sed -n 's/^[[:space:]]*ATTRS{interface}=="\([^"]*\)".*/\1/p' | head -n 1)
if [ "$INTERFACE" != "PCCOMM-AIM" ]; then
    echo "错误：所选设备接口为 '$INTERFACE'，本脚本需要 PCCOMM-AIM。"
    exit 1
fi

# 获取序列号（优先 ID_SERIAL_SHORT，其次 ID_SERIAL）
SERIAL=$(udevadm info -n "$DEVICE_PATH" -q property | sed -n 's/^ID_SERIAL_SHORT=//p')
if [ -z "$SERIAL" ]; then
    SERIAL=$(udevadm info -n "$DEVICE_PATH" -q property | sed -n 's/^ID_SERIAL=//p')
fi

echo "设备信息:"
echo "  Vendor ID: $VENDOR_ID"
echo "  Product ID: $PRODUCT_ID"
echo "  Serial: $SERIAL"
echo "  Interface: $INTERFACE"

# 询问用户想要的固定设备名
echo ""
echo "请输入想要的固定设备名称 (视觉建议: mascdc, 将创建为 /dev/mascdc；不要与导航使用同名):"
read FIXED_NAME

if [ -z "$FIXED_NAME" ]; then
    echo "错误: 设备名称不能为空"
    exit 1
fi

# 创建udev规则
# VID/PID 用设备属性匹配，interface 用接口父节点匹配，避免 ATTRS 跨层混用。
RULE_CONTENT="SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"$VENDOR_ID\", ENV{ID_MODEL_ID}==\"$PRODUCT_ID\", ATTRS{interface}==\"$INTERFACE\""
RULE_CONTENT="$RULE_CONTENT, SYMLINK+=\"$FIXED_NAME\", MODE=\"0666\", GROUP=\"dialout\""

# 创建udev规则文件
sudo mkdir -p /etc/udev/rules.d

RULE_FILE="/etc/udev/rules.d/99-$FIXED_NAME-serial.rules"
echo "$RULE_CONTENT" | sudo tee $RULE_FILE

echo "udev规则已创建: $RULE_FILE"
echo "内容: $RULE_CONTENT"

# 重新加载udev规则
echo "重新加载udev规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 将当前用户添加到dialout组（如果还没有）
if ! groups $USER | grep -q dialout; then
    echo "将用户 $USER 添加到 dialout 组..."
    sudo usermod -a -G dialout $USER
    echo "请重新登录或运行 'newgrp dialout' 以使组更改生效"
else
    echo "用户 $USER 已在 dialout 组中"
fi

echo ""
echo "设置完成！"
echo "设备现在将始终映射到 /dev/$FIXED_NAME"
echo ""
echo "要验证设置是否生效，请："
echo "1. 拔掉设备"
echo "2. 重新插入设备"
echo "3. 检查设备是否存在: ls -la /dev/$FIXED_NAME"
echo "4. 检查权限: ls -la /dev/$FIXED_NAME"
echo ""
echo "如果需要修改MAS程序中的串口配置，请编辑 config/serial_config.yaml 文件："
echo "  port: \"/dev/$FIXED_NAME\""
