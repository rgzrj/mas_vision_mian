cmake_minimum_required(VERSION 3.12)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(mas_replay LANGUAGES CXX)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(S "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_ttf REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Only this generated translation unit contains replay hooks. The real Target file is never written.
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/replay_target.cpp"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_target.py"
        "${S}/auto_aim/armor_track/armor_target.cpp" "${CMAKE_CURRENT_BINARY_DIR}/replay_target.cpp"
    DEPENDS "${S}/auto_aim/armor_track/armor_target.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_target.py"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/replay_instrumentation.patch"
    VERBATIM)
add_executable(replay EXCLUDE_FROM_ALL
    main.cpp player.cpp "${CMAKE_CURRENT_BINARY_DIR}/replay_target.cpp"
    ${S}/auto_aim/armor_detector/armor_detector.cpp
    ${S}/auto_aim/armor_detector/light_corner_corrector.cpp
    ${S}/auto_aim/armor_detector/number_classifier.cpp
    ${S}/auto_aim/armor_track/armor_pose.cpp
    ${S}/auto_aim/armor_track/armor_track.cpp
    ${S}/rm_utils/algorithm/math_tools.cpp
    ${S}/rm_utils/algorithm/extended_kalman_filter.cpp
    ${S}/rm_utils/display/display.cpp
    ${S}/rm_utils/mas_log/mas_log.cpp)
target_include_directories(replay PRIVATE
    ${S}/auto_aim ${S}/auto_aim/armor_detector ${S}/auto_aim/armor_track ${S}/hardware
    ${S}/rm_utils/algorithm ${S}/rm_utils/mas_log ${S}/rm_utils/plotter
    ${S}/rm_utils/display ${S}/rm_utils/thread_pool ${S}/rm_utils/SPSCQueue
    ${OpenCV_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS})
target_link_libraries(replay PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp::yaml-cpp
    spdlog::spdlog SDL2::SDL2 SDL2_image::SDL2_image SDL2_ttf::SDL2_ttf)
target_compile_definitions(replay PRIVATE REPLAY_BUILD_INFO="${CMAKE_CURRENT_BINARY_DIR}")
set_target_properties(replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# Preserve the actual build inputs for later comparisons, including header changes.
file(GLOB_RECURSE REPLAY_INPUTS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.py"
    "${S}/auto_aim/armor_detector/*.cpp" "${S}/auto_aim/armor_detector/*.hpp"
    "${S}/auto_aim/armor_track/*.cpp" "${S}/auto_aim/armor_track/*.hpp"
    "${S}/rm_utils/algorithm/*.cpp" "${S}/rm_utils/algorithm/*.hpp"
    "${S}/rm_utils/display/*.cpp" "${S}/rm_utils/display/*.hpp"
    "${S}/rm_utils/mas_log/*.cpp" "${S}/rm_utils/mas_log/*.hpp"
    "${S}/rm_utils/plotter/*.hpp" "${S}/rm_utils/thread_pool/*.hpp" "${S}/rm_utils/SPSCQueue/*.h")
list(APPEND REPLAY_INPUTS "${S}/auto_aim/armor_types.hpp" "${S}/hardware/common_def.hpp" "${CMAKE_CURRENT_LIST_FILE}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${REPLAY_INPUTS})
set(REPLAY_MANIFEST "{\n")
foreach(INPUT IN LISTS REPLAY_INPUTS)
    file(RELATIVE_PATH RELATIVE_INPUT "${S}" "${INPUT}")
    file(SHA256 "${INPUT}" HASH)
    string(APPEND REPLAY_MANIFEST "  \"${RELATIVE_INPUT}\": \"${HASH}\",\n")
endforeach()
string(REGEX REPLACE ",\n$" "\n}" REPLAY_MANIFEST "${REPLAY_MANIFEST}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/source_manifest.json" "${REPLAY_MANIFEST}\n")
