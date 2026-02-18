#include "module/identifier/model.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <numbers>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <vector>

#include "utility/image/image.details.hpp"
#include "utility/math/point.hpp"

using namespace pingpong_tracker::identifier;
using namespace pingpong_tracker;

namespace {

template <typename T>
auto ComputeCircleIoU(const Point2D& c1, double r1, const T& c2, double r2) -> double {
    const auto d = std::hypot(c1.x - c2.x, c1.y - c2.y);

    if (d >= r1 + r2) {
        return 0.0;
    }

    if (d <= std::abs(r1 - r2)) {
        const auto r_min = std::min(r1, r2);
        const auto r_max = std::max(r1, r2);
        return (r_min * r_min) / (r_max * r_max);
    }

    const auto r1_sq = r1 * r1;
    const auto r2_sq = r2 * r2;

    const auto clamp = [](double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); };

    const auto angle1 = 2.0 * std::acos(clamp((r1_sq + d * d - r2_sq) / (2.0 * r1 * d), -1.0, 1.0));
    const auto angle2 = 2.0 * std::acos(clamp((r2_sq + d * d - r1_sq) / (2.0 * r2 * d), -1.0, 1.0));

    const auto intersection =
        0.5 * r1_sq * (angle1 - std::sin(angle1)) + 0.5 * r2_sq * (angle2 - std::sin(angle2));
    const auto union_area = std::numbers::pi * (r1_sq + r2_sq) - intersection;

    return intersection / union_area;
}

struct ExpectedDetection {
    Point2D center;
    double radius;
    double min_confidence;
};

}  // namespace

class OpenVinoNetTest : public ::testing::Test {
protected:
    void SetUp() override {
        const auto project_root = std::filesystem::path(PROJECT_ROOT);
        const auto get_path     = [](const char* env_var, std::filesystem::path fallback) {
            if (const auto env = std::getenv(env_var))
                return std::filesystem::path(env);
            return fallback;
        };

        const auto assets_root = get_path("TEST_ASSETS_ROOT", "/tmp/pingpong_tracker");
        const auto models_root = get_path("TEST_MODELS_ROOT", project_root / "models");

        // Setup a valid base config structure
        config_["model_location"]  = (models_root / "yolov8.onnx").string();
        config_["infer_device"]    = "CPU";
        config_["input_rows"]      = 640;
        config_["input_cols"]      = 640;
        config_["score_threshold"] = 0.5;
        config_["nms_threshold"]   = 0.45;

        test_image_path_ = (assets_root / "jump.png").string();
    }

    [[nodiscard]] bool HasValidModel() const {
        return std::filesystem::exists(config_["model_location"].as<std::string>());
    }

    [[nodiscard]] bool HasValidImage() const {
        return std::filesystem::exists(test_image_path_);
    }

    auto LoadTestImage() -> std::optional<Image> {
        auto cv_img = cv::imread(test_image_path_);
        if (cv_img.empty())
            return std::nullopt;

        auto img = Image{};
        img.details().set_mat(cv_img);
        return img;
    }

    void ValidateDetections(const std::vector<Ball2D>& actual,
                            const std::vector<ExpectedDetection>& expected_list) {
        ASSERT_FALSE(actual.empty()) << "Inference returned no detections!";

        for (const auto& expected : expected_list) {
            auto max_iou = 0.0;

            for (const auto& det : actual) {
                if (det.confidence < expected.min_confidence)
                    continue;

                const auto iou =
                    ComputeCircleIoU(expected.center, expected.radius, det.center, det.radius);
                max_iou = std::max(max_iou, iou);
            }

            EXPECT_GE(max_iou, 0.5)
                << "Best match IoU (" << max_iou << ") is too low for target at "
                << expected.center.x << "," << expected.center.y;
        }
    }

    OpenVinoNet net_;
    YAML::Node config_;
    std::string test_image_path_;
};

TEST_F(OpenVinoNetTest, ConfigureFailsWithEmptyConfig) {
    EXPECT_FALSE(net_.configure(YAML::Node{}).has_value());
}

TEST_F(OpenVinoNetTest, ConfigureFailsWithInvalidModelPath) {
    config_["model_location"] = "non_existent.onnx";
    auto result               = net_.configure(config_);
    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Failed to load model"), std::string::npos);
}

TEST_F(OpenVinoNetTest, SyncInferFailsWithEmptyImage) {
    EXPECT_FALSE(net_.sync_infer(Image{}).has_value());
}

TEST_F(OpenVinoNetTest, AsyncInferFailsWithEmptyImage) {
    bool invoked = false;
    net_.async_infer(Image{}, [&](auto result) {
        invoked = true;
        ASSERT_FALSE(result.has_value());
        EXPECT_EQ(result.error(), "Empty image mat");
    });
    EXPECT_TRUE(invoked);
}

TEST_F(OpenVinoNetTest, ConfigureSuccessWithValidModelPath) {
    if (!HasValidModel())
        GTEST_SKIP() << "Model file missing";
    EXPECT_TRUE(net_.configure(config_).has_value());
}

TEST_F(OpenVinoNetTest, SyncInferSuccessWithValidImage) {
    if (!HasValidModel())
        GTEST_SKIP() << "Model file missing";
    if (!HasValidImage())
        GTEST_SKIP() << "Test image missing";

    ASSERT_TRUE(net_.configure(config_).has_value());

    auto image_opt = LoadTestImage();
    if (!image_opt)
        GTEST_SKIP() << "Failed to read image";

    auto result = net_.sync_infer(*image_opt);
    ASSERT_TRUE(result.has_value());

    const auto ground_truth = std::vector<ExpectedDetection>{{{184.0, 349.0}, 6.0, 0.9}};

    ValidateDetections(result.value(), ground_truth);
}

TEST_F(OpenVinoNetTest, AsyncInferSuccessWithValidImage) {
    if (!HasValidModel())
        GTEST_SKIP() << "Model file missing";
    if (!HasValidImage())
        GTEST_SKIP() << "Test image missing";

    ASSERT_TRUE(net_.configure(config_).has_value());

    auto image_opt = LoadTestImage();
    if (!image_opt)
        GTEST_SKIP() << "Failed to read image";

    const auto ground_truth = std::vector<ExpectedDetection>{{{184.0, 349.0}, 6.0, 0.9}};

    std::promise<void> promise;
    auto future = promise.get_future();

    net_.async_infer(*image_opt, [&](auto result) {
        if (result.has_value()) {
            ValidateDetections(result.value(), ground_truth);
        } else {
            ADD_FAILURE() << "Async inference failed: " << result.error();
        }
        promise.set_value();
    });

    ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready) << "Async "
                                                                                      "inference "
                                                                                      "timed out!";
}
