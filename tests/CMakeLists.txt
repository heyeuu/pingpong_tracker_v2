add_executable(
    ${PROJECT_NAME}_test
    test_main.cpp
)
target_link_libraries(${PROJECT_NAME}_test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_test)