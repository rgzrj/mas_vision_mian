add_library(threads STATIC
    camera_thread/camera_thread.cpp
    serial_thread/serial_thread.cpp
    usbcamera_thread/usbcamera_thread.cpp
    armor_thread/armor_thread.cpp
    ros2_thread/ros2_thread.cpp
)

# 设置库的包含目录
target_include_directories(threads PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_thread
    ${CMAKE_CURRENT_SOURCE_DIR}/serial_thread
    ${CMAKE_CURRENT_SOURCE_DIR}/usbcamera_thread
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_thread
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_thread
)

target_link_libraries(threads PUBLIC
    hikcamera
    hardware_common
    rm_utils
    serial
    usbcamera
    auto_aim
)
