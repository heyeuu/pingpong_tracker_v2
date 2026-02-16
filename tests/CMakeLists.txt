# --- Tests ---

# Serializable Test
add_executable(test_serializable serializable.cpp)
target_include_directories(test_serializable PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(test_serializable PRIVATE
    pingpong_tracker_module
    GTest::gtest_main
    yaml-cpp
)
configure_file(serializable.yaml ${CMAKE_CURRENT_BINARY_DIR}/serializable.yaml COPYONLY)
include(GoogleTest)
gtest_discover_tests(test_serializable)

# Model Test
add_executable(model_test model_test.cpp)
target_include_directories(model_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(model_test PRIVATE
    pingpong_tracker_module
    GTest::gtest_main
    yaml-cpp
    ${OpenCV_LIBS}
)
gtest_discover_tests(model_test)