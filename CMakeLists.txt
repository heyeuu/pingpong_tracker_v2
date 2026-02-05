cmake_minimum_required(VERSION 3.22)
project(pingpong_tracker)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

## 运行时构建: runtime
add_executable(
    ${PROJECT_NAME}_runtime
    ${PROJECT_SOURCE_DIR}/src/runtime.cpp
)
