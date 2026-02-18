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

template <char Dim1, char Dim2, char Dim3, char Dim4>
struct TensorLayout {
    static constexpr std::array kChars{Dim1, Dim2, Dim3, Dim4, '\0'};

    constexpr static auto layout() noexcept {
        return ov::Layout{kChars.data()};
    }

    constexpr static auto shape(const Dimensions& dimensions) noexcept {
        return ov::Shape{{
            static_cast<std::size_t>(dimensions.template at<Dim1>()),
            static_cast<std::size_t>(dimensions.template at<Dim2>()),
            static_cast<std::size_t>(dimensions.template at<Dim3>()),
            static_cast<std::size_t>(dimensions.template at<Dim4>()),
        }};
    }
};

struct OpenVinoNet::Impl : std::enable_shared_from_this<Impl> {
    using InputLayout = TensorLayout<'N', 'H', 'W', 'C'>;
    using ModelLayout = TensorLayout<'N', 'C', 'H', 'W'>;

    using Result   = std::expected<std::vector<Ball2D>, std::string>;
    using Callback = std::function<void(Result)>;

    ov::CompiledModel openvino_model;
    ov::Core openvino_core;

    struct PreprocessInfo {
        float scale;
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
            .W = static_cast<dimension_type>(config.input_cols),
            .H = static_cast<dimension_type>(config.input_rows),
        };

        auto& input = preprocess.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_shape(InputLayout::shape(dimensions))
            .set_layout(InputLayout::layout())
            .set_color_format(ov::preprocess::ColorFormat::BGR);

        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB);

        input.model().set_layout(ModelLayout::layout());

        // For real-time process, use this mode
        const auto performance = ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
        const auto preprocessed_out = preprocess.build();
        openvino_model =
            openvino_core.compile_model(preprocessed_out, config.infer_device, performance);
        return {};

    } catch (const std::runtime_error& e) {
        return std::unexpected{std::string{"Failed to load model | "} + e.what()};

    } catch (...) {
        return std::unexpected{"Failed to load model caused by unknown exception"};
    }

    auto generate_openvino_request(const Image& image) noexcept
        -> std::expected<std::pair<ov::InferRequest, PreprocessInfo>, std::string> {
        const auto& origin_mat = image.details().get_mat();
        if (origin_mat.empty()) [[unlikely]] {
            return std::unexpected{"Empty image mat"};
        }

        const auto input_w = static_cast<float>(config.input_cols);
        const auto input_h = static_cast<float>(config.input_rows);
        const auto img_w   = static_cast<float>(origin_mat.cols);
        const auto img_h   = static_cast<float>(origin_mat.rows);

        const auto scale = std::min(input_w / img_w, input_h / img_h);
        const auto new_w = static_cast<int>(img_w * scale);
        const auto new_h = static_cast<int>(img_h * scale);

        auto resized_mat = cv::Mat{};
        cv::resize(origin_mat, resized_mat, {new_w, new_h});

        const auto pad_w      = static_cast<int>(input_w) - new_w;
        const auto pad_h      = static_cast<int>(input_h) - new_h;
        const auto pad_top    = pad_h / 2;
        const auto pad_bottom = pad_h - pad_top;
        const auto pad_left   = pad_w / 2;
        const auto pad_right  = pad_w - pad_left;

        const auto dimensions = Dimensions{
            .W = static_cast<dimension_type>(config.input_cols),
            .H = static_cast<dimension_type>(config.input_rows),
        };

        auto input_tensor = ov::Tensor(ov::element::u8, InputLayout::shape(dimensions));
        auto tensor_mat =
            cv::Mat{config.input_rows, config.input_cols, CV_8UC3, input_tensor.data()};

        cv::copyMakeBorder(resized_mat, tensor_mat, pad_top, pad_bottom, pad_left, pad_right,
                           cv::BORDER_CONSTANT, cv::Scalar{114, 114, 114});

        auto request = openvino_model.create_infer_request();
        request.set_input_tensor(input_tensor);

        return std::make_pair(std::move(request),
                              PreprocessInfo{.scale = scale,
                                             .pad_x = static_cast<float>(pad_left),
                                             .pad_y = static_cast<float>(pad_top)});
    }

    auto explain_infer_result(ov::InferRequest& finished_request,
                              const PreprocessInfo& info) const noexcept -> std::vector<Ball2D> {
        auto tensor          = finished_request.get_output_tensor();
        auto [boxes, scores] = parse_inference_output(tensor);

        auto indices = std::vector<int>{};
        cv::dnn::NMSBoxes(boxes, scores, config.score_threshold, config.nms_threshold, indices);

        return restore_coordinates(boxes, scores, indices, info);
    }

    auto parse_inference_output(const ov::Tensor& tensor) const noexcept
        -> std::pair<std::vector<cv::Rect>, std::vector<float>> {
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
        } else if (shape.size() >= 3) {
            anchors  = shape[2];
            channels = shape[1];
        } else {
            return {};
        }

        auto scores = std::vector<float>{};
        auto boxes  = std::vector<cv::Rect>{};
        scores.reserve(anchors);
        boxes.reserve(anchors);

        const auto data = std::span<const float>{const_cast<ov::Tensor&>(tensor).data<float>(),
                                                 tensor.get_size()};

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

        return {std::move(boxes), std::move(scores)};
    }

    auto restore_coordinates(const std::vector<cv::Rect>& boxes, const std::vector<float>& scores,
                             const std::vector<int>& indices,
                             const PreprocessInfo& info) const noexcept -> std::vector<Ball2D> {
        auto final_result = std::vector<Ball2D>{};
        final_result.reserve(indices.size());

        for (const auto idx : indices) {
            const auto& rect = boxes[idx];

            // Map back to original image using letterbox parameters
            const auto center_x = (rect.x + rect.width / 2.0F - info.pad_x) / info.scale;
            const auto center_y = (rect.y + rect.height / 2.0F - info.pad_y) / info.scale;

            // Radius scaling
            const auto radius = ((rect.height + rect.width) / 4.0F) / info.scale;

            final_result.push_back(Ball2D{
                .center     = {center_x, center_y},
                .radius     = radius,
                .confidence = scores[idx],
            });
        }

        return final_result;
    }

    auto sync_infer(const Image& image) noexcept -> std::optional<std::vector<Ball2D>> {
        auto result = generate_openvino_request(image);
        if (!result.has_value()) {
            return std::nullopt;
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
        auto weak_self       = weak_from_this();

        request.set_callback([request, callback = std::move(callback), info = info,
                              weak_self](const auto& e) mutable {
            auto self = weak_self.lock();
            if (!self) {
                // Lifecycle expired, do not execute callback logic
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
            } else {
                auto result = self->explain_infer_result(request, info);
                callback(std::move(result));
            }

            request.set_callback([](const std::exception_ptr&) {});
        });
        request.start_async();
    }
};

OpenVinoNet::OpenVinoNet() : pimpl{std::make_shared<Impl>()} {
}

OpenVinoNet::~OpenVinoNet() = default;

auto OpenVinoNet::configure(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->configure(yaml);
}

auto OpenVinoNet::sync_infer(const Image& image) noexcept -> std::optional<std::vector<Ball2D>> {
    return pimpl->sync_infer(image);
}

auto OpenVinoNet::async_infer(
    const Image& image,
    std::function<void(std::expected<std::vector<Ball2D>, std::string>)> callback) noexcept
    -> void {
    pimpl->async_infer(image, std::move(callback));
}

}  // namespace pingpong_tracker::identifier
