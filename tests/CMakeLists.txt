include(GoogleTest)

if(NOT yaml-cpp_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
endif()

# Serializable
add_executable(
    test_serializable
    serializable.cpp
)
target_link_libraries(
    test_serializable
    GTest::gtest_main
    GTest::gtest
    yaml-cpp
)
configure_file(serializable.yaml ${CMAKE_CURRENT_BINARY_DIR}/serializable.yaml COPYONLY)
gtest_discover_tests(test_serializable)