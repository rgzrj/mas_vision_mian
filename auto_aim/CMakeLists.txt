# 创建静态库
add_library(auto_aim STATIC
    armor_detector/armor_detector.cpp
    armor_detector/light_corner_corrector.cpp
    armor_detector/number_classifier.cpp
    armor_track/armor_track.cpp
    armor_track/armor_pose.cpp
    armor_track/armor_target.cpp
    armor_shoot/armor_shoot.cpp
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(Ceres REQUIRED)


# 设置库的包含目录
target_include_directories(auto_aim PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
    #${CERES_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_detector
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_track
    ${CMAKE_CURRENT_SOURCE_DIR}/armor_shoot
)

# 链接库
target_link_libraries(auto_aim PUBLIC 
    rm_utils
    hardware_common
    ${OpenCV_LIBS}
    serial
    hikcamera
    usbcamera
    #${CERES_LIBRARIES}
    Eigen3::Eigen
)
