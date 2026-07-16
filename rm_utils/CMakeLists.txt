# 创建静态库
add_library(rm_utils STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mas_log/mas_log.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plotter/plotter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/display/display.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/math_tools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/extended_kalman_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/ransac_sine_fitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_comm/ros2_comm.cpp
)

find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_ttf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 设置 rm_utils 库的包含目录
target_include_directories(rm_utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mas_log
    ${CMAKE_CURRENT_SOURCE_DIR}/thread_pool
    ${CMAKE_CURRENT_SOURCE_DIR}/SPSCQueue
    ${CMAKE_CURRENT_SOURCE_DIR}/plotter
    ${CMAKE_CURRENT_SOURCE_DIR}/display
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_comm
    ${OpenCV_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIR}
)

# 链接 rm_utils 库
target_link_libraries(rm_utils PUBLIC 
    spdlog::spdlog
    ${OpenCV_LIBS}
    SDL2::SDL2
    SDL2_image::SDL2_image
    SDL2_ttf::SDL2_ttf
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)