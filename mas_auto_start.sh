#!/bin/bash

sleep 5 

WORKSPACE_DIR="/home/mas/mas_vision/build"    # 工作目录
PROG="./base"                                 # 要执行的程序
PROG_NAME="base"                              # 进程名（用于pgrep精确匹配）
CHECK_INTERVAL=30                             # 检测间隔（秒）
# 进入工作目录（失败则退出）
cd "$WORKSPACE_DIR" || { echo "无法进入目录: $WORKSPACE_DIR"; exit 1; }

# 启动函数
start_program() {
    echo "启动程序: $PROG" 
    # 打开新终端执行程序，程序退出后保持终端开启
    gnome-terminal -- bash -c "cd $WORKSPACE_DIR; $PROG; exec bash"
    
}

# 首次启动
start_program

# 看门狗循环
#while true; do
#    sleep "$CHECK_INTERVAL"
    
    # 检测进程是否存在（-x：精确匹配进程名）
#    if ! pgrep -x "$PROG_NAME" > /dev/null; then
#        echo "程序已退出，正在重启..."
#        start_program
#    fi
#done
