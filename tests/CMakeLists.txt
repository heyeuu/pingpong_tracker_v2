include(GoogleTest)

# Serializable
add_executable(
    test_serializable
    serializable.cpp
)
target_link_libraries(
    test_serializable
    GTest::gtest_main
    yaml-cpp
)
target_include_directories(test_serializable PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)
configure_file(serializable.yaml ${CMAKE_CURRENT_BINARY_DIR}/serializable.yaml COPYONLY)
gtest_discover_tests(test_serializable)