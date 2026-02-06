cmake_minimum_required(VERSION 3.22)
project(pingpong_tracker)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

## 依赖管理: GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

## 依赖管理: yaml-cpp
FetchContent_Declare(
  yaml-cpp
  URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

## 代码资源搜索
file(GLOB_RECURSE PINGPONG_TRACKER_MODULE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/module/*.cpp
    ${PROJECT_SOURCE_DIR}/src/module/*.cc
    ${PROJECT_SOURCE_DIR}/src/utility/*.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/*.cc
)

## 运行时构建: runtime
add_executable(
    ${PROJECT_NAME}_runtime
    ${PROJECT_SOURCE_DIR}/src/runtime.cpp
    ${PINGPONG_TRACKER_MODULE} 
)
target_include_directories(${PROJECT_NAME}_runtime PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)


enable_testing()
add_subdirectory(tests)
