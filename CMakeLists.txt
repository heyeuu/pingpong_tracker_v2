cmake_minimum_required(VERSION 3.24)
project(pingpong_tracker)

# --- 基础配置 ---
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
add_compile_options(
  -Wall -Wextra -Wpedantic
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:Release>:-march=native>
)

# --- 依赖查找 ---
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- 源码搜集 ---
file(GLOB_RECURSE PINGPONG_TRACKER_KERNEL
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/kernel/*.cpp
    ${PROJECT_SOURCE_DIR}/src/kernel/*.cc
)
file(GLOB_RECURSE PINGPONG_TRACKER_MODULE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/module/*.cpp
    ${PROJECT_SOURCE_DIR}/src/module/*.cc
    ${PROJECT_SOURCE_DIR}/src/utility/*.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/*.cc
)
add_subdirectory(third_party/hikcamera)

# --- 核心目标: Module ---
add_library(
    ${PROJECT_NAME}_module SHARED
    ${PINGPONG_TRACKER_MODULE}
)
target_include_directories(${PROJECT_NAME}_module PUBLIC
    ${PROJECT_SOURCE_DIR}/src
)
target_link_libraries(
    ${PROJECT_NAME}_module
    PUBLIC
    yaml-cpp::yaml-cpp
    openvino::runtime
    ${OpenCV_LIBS}
    hikcamera
)

# --- 核心目标: Kernel ---
add_library(
    ${PROJECT_NAME}_kernel SHARED
    ${PINGPONG_TRACKER_KERNEL}
)
target_include_directories(${PROJECT_NAME}_kernel PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(
    ${PROJECT_NAME}_kernel
    PUBLIC
    ${PROJECT_NAME}_module
)

# --- 运行时目标: Runtime ---
add_executable(
    ${PROJECT_NAME}_runtime
    ${PROJECT_SOURCE_DIR}/src/runtime.cpp
)
target_link_libraries(${PROJECT_NAME}_runtime PRIVATE
    ${PROJECT_NAME}_kernel
)

# --- 测试构建 ---
option(BUILD_TESTING "Build the testing tree." ON)
if(BUILD_TESTING)
    enable_testing()
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    if(MSVC)
        set(gtest_force_shared_crt ON CACHE BOOL "")
    endif()
    FetchContent_MakeAvailable(googletest)
    
    add_subdirectory(tests)
endif()
