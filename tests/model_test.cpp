#include "module/identifier/model.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "utility/image/image.details.hpp"

using namespace pingpong_tracker::identifier;
using namespace pingpong_tracker;

class OpenVinoNetTest : public ::testing::Test {
protected:
    OpenVinoNet net;
    YAML::Node config;
    std::string test_image_path;

    void SetUp() override {
        const auto project_root = std::filesystem::path(PROJECT_ROOT);
        const auto get_path     = [](const char* env_var, std::filesystem::path fallback) {
            if (const auto env = std::getenv(env_var)) {
                return std::filesystem::path(env);
            }
            return fallback;
        };

        const auto assets_root = get_path("TEST_ASSETS_ROOT", "/tmp/pingpong_tracker");
        const auto models_root = get_path("TEST_MODELS_ROOT", project_root / "models");

        // Setup a valid base config structure
        config["model_location"]  = (models_root / "yolov8.onnx").string();
        config["infer_device"]    = "CPU";
        config["input_rows"]      = 640;
        config["input_cols"]      = 640;
        config["score_threshold"] = 0.5;
        config["nms_threshold"]   = 0.45;

        test_image_path = (assets_root / "jump.png").string();
    }
};

TEST_F(OpenVinoNetTest, ConfigureFailsWithEmptyConfig) {
    auto empty_config = YAML::Node{};
    auto result       = net.configure(empty_config);
    EXPECT_FALSE(result.has_value());
}

TEST_F(OpenVinoNetTest, ConfigureFailsWithInvalidModelPath) {
    // The config is valid structurally, but model path doesn't exist
    config["model_location"] = "non_existent_model.onnx";

    auto result = net.configure(config);

    ASSERT_FALSE(result.has_value());
    // Expect OpenVINO to complain about file not found or similar
    EXPECT_NE(result.error().find("Failed to load model"), std::string::npos)
        << "Actual error: " << result.error();
}

TEST_F(OpenVinoNetTest, SyncInferFailsWithEmptyImage) {
    // Even without configure, empty image check happens first
    auto empty_image = Image{};

    auto result = net.sync_infer(empty_image);
    EXPECT_FALSE(result.has_value());
}

TEST_F(OpenVinoNetTest, AsyncInferFailsWithEmptyImage) {
    auto empty_image      = Image{};
    auto callback_invoked = false;

    net.async_infer(empty_image, [&](auto result) {
        callback_invoked = true;
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), "Empty image mat");
    });

    EXPECT_TRUE(callback_invoked);
}

TEST_F(OpenVinoNetTest, ConfigureSuccessWithValidModelPath) {
    auto result = net.configure(config);
    EXPECT_TRUE(result.has_value()) << "Configuration failed: " << result.error();
}

TEST_F(OpenVinoNetTest, SyncInferSuccessWithValidImage) {
    if (const auto model_path = config["model_location"].as<std::string>();
        !std::filesystem::exists(model_path)) {
        GTEST_SKIP() << "Model file not found at: " << model_path;
    }

    auto conf_res = net.configure(config);
    ASSERT_TRUE(conf_res.has_value()) << "Failed to configure network: " << conf_res.error();

    if (!std::filesystem::exists(test_image_path)) {
        GTEST_SKIP() << "Test image not found at: " << test_image_path;
    }

    // create valid image
    auto img = cv::imread(test_image_path);
    if (img.empty()) {
        GTEST_SKIP() << "Failed to load image from: " << test_image_path;
    }

    auto valid_image          = Image{};
    valid_image.details().mat = img;

    auto result = net.sync_infer(valid_image);

    // Check that the inference was successful and the result is not empty
    ASSERT_TRUE(result.has_value());
    EXPECT_NE(result.value().size(), 0);  // Expect at least one detection
}
