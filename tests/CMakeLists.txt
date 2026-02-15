include(GoogleTest)
find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Serializable
add_executable(
    test_serializable
    serializable.cpp
)
target_link_libraries(test_serializable PRIVATE
    pingpong_tracker_module
    GTest::gtest_main
    yaml-cpp
)
target_include_directories(test_serializable PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
configure_file(serializable.yaml ${CMAKE_CURRENT_BINARY_DIR}/serializable.yaml COPYONLY)
gtest_discover_tests(test_serializable)

# Model test
add_executable(model_test model_test.cpp)

target_link_libraries(model_test PRIVATE
    pingpong_tracker_module
    GTest::gtest_main
    ${OpenCV_LIBS}
    yaml-cpp
)
target_include_directories(model_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
gtest_discover_tests(model_test)