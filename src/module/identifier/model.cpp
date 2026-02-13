#include "model.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/exception.hpp>
#include <span>

#include "utility/ball/ball.hpp"
#include "utility/image/image.details.hpp"
#include "utility/serializable.hpp"

namespace pingpong_tracker::identifier {

namespace util = pingpong_tracker::util;

using dimension_type = ov::Dimension::value_type;
struct Dimensions {
    dimension_type N = 1;
    dimension_type C = 3;
    dimension_type W = 0;
    dimension_type H = 0;

    template <char D>
    [[nodiscard]] constexpr auto at() const noexcept {
        if constexpr (D == 'N') {
            return N;
        } else if constexpr (D == 'C') {
            return C;
        } else if constexpr (D == 'W') {
            return W;
        } else if constexpr (D == 'H') {
            return H;
        } else {
            static_assert(false, "Wrong dimension char");
        }
    }
};

template <char _1, char _2, char _3, char _4>
struct TensorLayout {
    static constexpr std::array kChars{_1, _2, _3, _4, '\0'};

    constexpr static auto layout() noexcept {
        return ov::Layout{kChars.data()};
    }

    constexpr static auto partial_shape(const Dimensions& dimensions) noexcept {
        return ov::PartialShape{
            dimensions.template at<_1>(),
            dimensions.template at<_2>(),
            dimensions.template at<_3>(),
            dimensions.template at<_4>(),
        };
    }
    constexpr static auto shape(const Dimensions& dimensions) noexcept {
        return ov::Shape{{
            static_cast<std::size_t>(dimensions.template at<_1>()),
            static_cast<std::size_t>(dimensions.template at<_2>()),
            static_cast<std::size_t>(dimensions.template at<_3>()),
            static_cast<std::size_t>(dimensions.template at<_4>()),
        }};
    }
};

struct OpenVinoNet::Impl {
    using InputLayout = TensorLayout<'N', 'H', 'W', 'C'>;
    using ModelLayout = TensorLayout<'N', 'C', 'H', 'W'>;

    using Result   = std::expected<std::vector<Ball2D>, std::string>;
    using Callback = std::function<void(Result)>;

    ov::CompiledModel openvino_model;
    ov::Core openvino_core;

    struct Nothing {};
    std::shared_ptr<Nothing> living_flag{std::make_shared<Nothing>()};

    struct PreprocessInfo {
        float adapt_scaling;
        float pad_x;
        float pad_y;
    };

    struct ChannelIndex {
        static constexpr size_t kCx    = 0;
        static constexpr size_t kCy    = 1;
        static constexpr size_t kW     = 2;
        static constexpr size_t kH     = 3;
        static constexpr size_t kScore = 4;
    };

    struct Config : util::SerializableMixin {
        std::string model_location{"../../../models/yolov8.onnx"};
        std::string infer_device{"AUTO"};

        int input_rows = 640;
        int input_cols = 640;

        float score_threshold = 0.5F;
        float nms_threshold   = 0.5F;

        constexpr static std::tuple metas{
            // clang-format off
            "model_location",           &Config::model_location,
            "infer_device",             &Config::infer_device,
            "input_rows",               &Config::input_rows,
            "input_cols",               &Config::input_cols,
            "score_threshold",          &Config::score_threshold,
            "nms_threshold",            &Config::nms_threshold,
            // clang-format on
        };
    } config;

    auto configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected{result.error()};
        }

        return compile_openvino_model();
    }

    auto compile_openvino_model() noexcept -> std::expected<void, std::string> try {
        auto origin_model = openvino_core.read_model(config.model_location);
        if (!origin_model) {
            return std::unexpected{"Empty model resource was loaded from openvino core"};
        }

        auto preprocess = ov::preprocess::PrePostProcessor{origin_model};

        const auto dimensions = Dimensions{
            .W = config.input_cols,
            .H = config.input_rows,
        };

        auto& input = preprocess.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_shape(InputLayout::partial_shape(dimensions))
            .set_layout(InputLayout::layout())
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB);

        input.model().set_layout(ModelLayout::layout());

        // For real-time process, use this mode
        const auto performance  = ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
        const auto processe_out = preprocess.build();
        openvino_model =
            openvino_core.compile_model(processe_out, config.infer_device, performance);
        return {};

    } catch (const std::runtime_error& e) {
        return std::unexpected{std::string{"Failed to load model | "} + e.what()};

    } catch (...) {
        return std::unexpected{"Failed to load model caused by unknown exception"};
    }

    auto generate_openvino_request(const Image& image) noexcept
        -> std::expected<std::pair<ov::InferRequest, PreprocessInfo>, std::string> {
        const auto& origin_mat = image.details().mat;
        if (origin_mat.empty()) [[unlikely]] {
            return std::unexpected{"Empty image mat"};
        }

        const auto rows   = config.input_rows;
        const auto cols   = config.input_cols;
        auto input_tensor = ov::Tensor{ov::element::u8, InputLayout::shape({.W = cols, .H = rows})};
        auto adapt_scaling = 1.0F;
        auto pad_x         = 0.0F;
        auto pad_y         = 0.0F;

        {
            adapt_scaling = std::min(static_cast<float>(1. * cols / origin_mat.cols),
                                     static_cast<float>(1. * rows / origin_mat.rows));

            const auto scaled_w = static_cast<int>(1. * origin_mat.cols * adapt_scaling);
            const auto scaled_h = static_cast<int>(1. * origin_mat.rows * adapt_scaling);

            // Calculate padding for center alignment
            const auto pad_w_int = (cols - scaled_w) / 2;
            const auto pad_h_int = (rows - scaled_h) / 2;
            pad_x                = static_cast<float>(pad_w_int);
            pad_y                = static_cast<float>(pad_h_int);

            auto input_mat = cv::Mat{rows, cols, CV_8UC3, input_tensor.data()};
            input_mat.setTo(0);

            auto input_roi = cv::Rect2i{pad_w_int, pad_h_int, scaled_w, scaled_h};
            cv::resize(origin_mat, input_mat(input_roi), {scaled_w, scaled_h});
        }

        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(input_tensor);

        return std::make_pair(
            std::move(request),
            PreprocessInfo{.adapt_scaling = adapt_scaling, .pad_x = pad_x, .pad_y = pad_y});
    }

    auto explain_infer_result(ov::InferRequest& finished_request,
                              const PreprocessInfo& info) const noexcept -> std::vector<Ball2D> {
        auto tensor       = finished_request.get_output_tensor();
        const auto& shape = tensor.get_shape();

        // YOLOv8 output shape: [1, 5, 8400] -> [Batch, Channels, Anchors]
        // Channels: cx, cy, w, h, score

        std::size_t anchors  = 0;
        std::size_t channels = 0;
        bool is_channel_last = false;

        if (shape.size() == 3) {
            if (shape[1] > shape[2]) {
                // [1, 8400, 5]
                anchors         = shape[1];
                channels        = shape[2];
                is_channel_last = true;
            } else {
                // [1, 5, 8400]
                anchors         = shape[2];
                channels        = shape[1];
                is_channel_last = false;
            }
        } else {
            anchors  = shape[2];
            channels = shape[1];
        }

        auto scores = std::vector<float>{};
        auto boxes  = std::vector<cv::Rect>{};
        scores.reserve(anchors);
        boxes.reserve(anchors);

        const auto data = std::span{tensor.data<float>(), tensor.get_size()};

        for (std::size_t i = 0; i < anchors; ++i) {
            const auto offset = is_channel_last ? (i * channels) : i;
            const auto stride = is_channel_last ? 1 : anchors;

            const auto score = data[offset + (ChannelIndex::kScore * stride)];

            if (score > config.score_threshold) {
                const auto cx = data[offset + (ChannelIndex::kCx * stride)];
                const auto cy = data[offset + (ChannelIndex::kCy * stride)];
                const auto w  = data[offset + (ChannelIndex::kW * stride)];
                const auto h  = data[offset + (ChannelIndex::kH * stride)];

                const auto x = static_cast<int>(cx - w * 0.5f);
                const auto y = static_cast<int>(cy - h * 0.5f);

                boxes.emplace_back(x, y, static_cast<int>(w), static_cast<int>(h));
                scores.push_back(score);
            }
        }

        auto indices = std::vector<int>{};
        cv::dnn::NMSBoxes(boxes, scores, config.score_threshold, config.nms_threshold, indices);

        auto final_result = std::vector<Ball2D>{};
        final_result.reserve(indices.size());

        for (const auto idx : indices) {
            const auto& rect  = boxes[idx];
            const auto radius = (rect.height + rect.width) / 4.0F;

            // Coordinate restoration with padding and scaling
            const auto center_x = (rect.width / 2.0F + rect.x - info.pad_x) / info.adapt_scaling;
            const auto center_y = (rect.height / 2.0F + rect.y - info.pad_y) / info.adapt_scaling;

            final_result.push_back(Ball2D{
                .center     = {center_x, center_y},
                .radius     = radius,
                .confidence = scores[idx],
            });
        }

        return final_result;
    }

    auto sync_infer(const Image& image) noexcept -> Result {
        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            return std::unexpected{result.error()};
        }

        auto [request, info] = std::move(result.value());
        request.infer();

        return explain_infer_result(request, info);
    }

    auto async_infer(const Image& image, Callback callback) noexcept -> void {
        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            std::invoke(callback, std::unexpected{result.error()});
            return;
        }

        auto [request, info] = std::move(result.value());
        auto living_weak     = std::weak_ptr{living_flag};

        request.set_callback([request = std::move(request), callback = std::move(callback),
                              info = info, living_weak, this](const auto& e) mutable {
            if (!living_weak.lock()) {
                callback(std::unexpected{"Model source is no longer living"});
                return;
            }

            if (e) {
                auto error = std::string{};
                try {
                    std::rethrow_exception(e);
                } catch (const ov::Cancelled& e) {
                    error += "Cancelled | ";
                    error += e.what();
                } catch (const ov::Busy& e) {
                    error = "Busy | ";
                    error += e.what();
                } catch (const std::exception& e) {
                    error = "Unknown | ";
                    error += e.what();
                }
                callback(std::unexpected{error});
                return;
            }

            auto result = explain_infer_result(request, info);
            callback(std::move(result));

            request.set_callback([](const std::exception_ptr&) {});
        });
        request.start_async();
    }
};

OpenVinoNet::OpenVinoNet() : pimpl{std::make_unique<Impl>()} {
}

OpenVinoNet::~OpenVinoNet() = default;

auto OpenVinoNet::configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->configure(yaml);
}

auto OpenVinoNet::sync_infer(const Image& image) noexcept -> std::optional<std::vector<Ball2D>> {
    auto result = pimpl->sync_infer(image);
    if (result.has_value()) {
        return result.value();
    }
    return std::nullopt;
}

auto OpenVinoNet::async_infer(
    const Image& image,
    std::function<void(std::expected<std::vector<Ball2D>, std::string>)> callback) noexcept
    -> void {
    pimpl->async_infer(image, std::move(callback));
}

}  // namespace pingpong_tracker::identifier
