# --- Tests ---
include(GoogleTest)

# Serializable Test
add_executable(test_serializable serializable.cpp)
target_include_directories(test_serializable PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(test_serializable PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
    yaml-cpp::yaml-cpp
)
configure_file(serializable.yaml ${CMAKE_CURRENT_BINARY_DIR}/serializable.yaml COPYONLY)
gtest_discover_tests(test_serializable)

# Model Test
add_executable(model_test model_test.cpp)
target_include_directories(model_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(model_test PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
    yaml-cpp::yaml-cpp
    ${OpenCV_LIBS}
)
target_compile_definitions(model_test PRIVATE PROJECT_ROOT=\"${PROJECT_SOURCE_DIR}\")
gtest_discover_tests(model_test)

# Predictor BallState Test
add_executable(ball_state_test ball_state_test.cpp)
target_include_directories(ball_state_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(ball_state_test PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
)
gtest_discover_tests(ball_state_test)

# Predictor IMM + UKF Test
add_executable(imm_ball_state_test imm_ball_state_test.cpp)
target_include_directories(imm_ball_state_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(imm_ball_state_test PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
)
gtest_discover_tests(imm_ball_state_test)

# Predictor IMM Config Test
add_executable(imm_ball_state_config_test imm_ball_state_config_test.cpp)
target_include_directories(imm_ball_state_config_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(imm_ball_state_config_test PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
)
gtest_discover_tests(imm_ball_state_config_test)

# Predictor Replay Evaluation Test
add_executable(replay_evaluation_test replay_evaluation_test.cpp)
target_include_directories(replay_evaluation_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(replay_evaluation_test PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
)
gtest_discover_tests(replay_evaluation_test)

# Mono Reconstructor Test
add_executable(mono_reconstructor_test mono_reconstructor_test.cpp)
target_include_directories(mono_reconstructor_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(mono_reconstructor_test PRIVATE
    ${PROJECT_NAME}_module
    GTest::gtest_main
)
gtest_discover_tests(mono_reconstructor_test)
