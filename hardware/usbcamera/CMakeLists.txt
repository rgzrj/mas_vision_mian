# 创建静态库 
add_library(usbcamera STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/usbcamera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/usbcamera_config.cpp
)
find_package(OpenCV REQUIRED)

# 设置库的包含目录
target_include_directories(usbcamera PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# 链接所需库
target_link_libraries(usbcamera PUBLIC
    rm_utils
    ${OpenCV_LIBS}
    hardware_common
)