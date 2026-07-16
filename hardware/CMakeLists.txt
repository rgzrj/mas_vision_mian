# hardware 模块通用定义
add_library(hardware_common INTERFACE)

# 将 hardware 目录添加到包含路径
target_include_directories(hardware_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
