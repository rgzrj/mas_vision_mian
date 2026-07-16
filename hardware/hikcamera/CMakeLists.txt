# 创建静态库
add_library(hikcamera STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hikcamera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/hikcamera_reconnect_thread.cpp
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# 手动指定 MVS 路径（安装默认 /opt/MVS）
set(MVS_ROOT "/opt/MVS")
set(MVS_INCLUDE_DIR "${MVS_ROOT}/include")
set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")

# 设置库的包含目录
target_include_directories(hikcamera PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接库目录
target_link_directories(hikcamera PUBLIC 
    ${MVS_LIB_DIR}
)

# 链接所需库
target_link_libraries(hikcamera PUBLIC
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    MvCameraControl
    rm_utils
    hardware_common
)

# 运行时 rpath，防止找不到 .so
target_link_options(hikcamera PUBLIC
    "-Wl,-rpath,${MVS_LIB_DIR}"
)