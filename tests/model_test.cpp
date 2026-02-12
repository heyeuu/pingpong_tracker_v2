#include "module/identifier/model.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
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
        // Setup a valid base config structure
        config["model_location"]  = "/workspaces/alliance/pingpong_tracker/models/yolov8.onnx";
        config["infer_device"]    = "CPU";
        config["input_rows"]      = 640;
        config["input_cols"]      = 640;
        config["score_threshold"] = 0.25;
        config["nms_threshold"]   = 0.5;
        test_image_path           = "place/test/path";
    }
};

TEST_F(OpenVinoNetTest, ConfigureFailsWithEmptyConfig) {
    YAML::Node empty_config;
    auto result = net.configure(empty_config);
    EXPECT_FALSE(result.has_value());
}

TEST_F(OpenVinoNetTest, ConfigureFailsWithInvalidModelPath) {
    // The config is valid structurally, but model path doesn't exist
    config["model_location"] = "non_existent_model.onnx";

    auto result = net.configure(config);

    ASSERT_FALSE(result.has_value());
    // Expect OpenVINO to complain about file not found or similar
    EXPECT_NE(result.error().find("Failed to load model"), std::string::npos)  // fixed typo here
        << "Actual error: " << result.error();
}

TEST_F(OpenVinoNetTest, SyncInferFailsWithEmptyImage) {
    // Even without configure, empty image check happens first
    Image empty_image;

    auto result = net.sync_infer(empty_image);
    EXPECT_FALSE(result.has_value());
}

TEST_F(OpenVinoNetTest, AsyncInferFailsWithEmptyImage) {
    Image empty_image;
    bool callback_invoked = false;

    net.async_infer(empty_image, [&](auto result) {
        callback_invoked = true;
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), "Empty image mat");  // fixed typo here
    });

    EXPECT_TRUE(callback_invoked);
}

TEST_F(OpenVinoNetTest, ConfigureSuccessWithValidModelPath) {
    auto result = net.configure(config);
    EXPECT_TRUE(result.has_value()) << "Configuration failed: " << result.error();
}

TEST_F(OpenVinoNetTest, SyncInferSuccessWithValidImage) {
    auto conf_res = net.configure(config);
    ASSERT_TRUE(conf_res.has_value()) << "Failed to configure network: " << conf_res.error();

    // create valid image
    cv::Mat img = cv::imread(test_image_path);
    ASSERT_FALSE(img.empty()) << "Failed to load image from: " << test_image_path;
    Image valid_image;
    valid_image.details().mat = img;

    auto result = net.sync_infer(valid_image);

    // Check that the inference was successful and the result is not empty
    ASSERT_TRUE(result.has_value());
    EXPECT_NE(result.value().size(), 0);  // Expect at least one detection
}
