#pragma once
#include <yaml-cpp/exceptions.h>

#include <concepts>
#include <expected>
#include <format>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace rmcs::util {

using SerialResult = std::expected<void, std::string>;

namespace details {

    template <typename T>
    concept yaml_cpp_trait = requires(T node) {
        { node.IsNull() } -> std::same_as<bool>;
        { node[""] };
        { node.template as<std::string>() } -> std::convertible_to<std::string>;
    };

    template <typename T>
    concept serializable_metas_trait =
        requires { typename std::tuple_size<decltype(T::metas)>::type; }
        && (std::tuple_size_v<decltype(T::metas)> != 0)
        && (std::tuple_size_v<decltype(T::metas)> % 2 == 0);

    template <class Node>
    struct NodeAdapter {
        Node& node;

        explicit NodeAdapter(Node& node_ref) noexcept
            : node { node_ref } { }

        template <typename T>
            requires details::yaml_cpp_trait<Node>
        auto get_param(const std::string& name, T& target) -> std::expected<void, std::string> {
            try {
                auto child = node[name];
                if (!child.IsDefined()) {
                    return std::unexpected {
                        std::format("Missing key '{}'", name),
                    };
                }
                if (child.IsNull()) {
                    return std::unexpected {
                        std::format("Key '{}' is null", name),
                    };
                }
                target = child.template as<T>();
            } catch (const YAML::BadConversion& e) {
                return std::unexpected {
                    std::format("Type mismatch for '{}': {}", name, e.what()),
                };
            } catch (const std::exception& e) {
                return std::unexpected {
                    std::format("Exception while reading '{}': {}", name, e.what()),
                };
            }
            return {};
        }
    };

    template <class Data, typename Mem>
    struct MemberMeta final {
        using D = Data;
        using M = Mem;
        using P = Mem Data::*;

        std::string_view meta_name;
        Mem Data::* mem_ptr;

        constexpr explicit MemberMeta(Mem Data::* member_ptr, std::string_view id) noexcept
            : meta_name { id }
            , mem_ptr { member_ptr } { }

        template <typename T>
        constexpr decltype(auto) extract_from(T&& data) const noexcept {
            return std::forward<T>(data).*mem_ptr;
        }
    };

    template <class Data, typename... Mem>
    struct Serializable final {
        std::tuple<MemberMeta<Data, Mem>...> metas;

        constexpr explicit Serializable(MemberMeta<Data, Mem>... metas) noexcept
            : metas { std::tuple { metas... } } { }

        template <class Node>
        auto serialize(std::string_view prefix, Node& source, Data& target) const
            -> std::expected<void, std::string> {
            using Ret = std::expected<void, std::string>;

            auto adapter = NodeAdapter { source };

            const auto deserialize = [&]<typename T>(MemberMeta<Data, T> meta) -> Ret {
                auto& target_member = meta.extract_from(target);
                if (prefix.empty()) {
                    return adapter.get_param(std::string { meta.meta_name }, target_member);
                } else {
                    return adapter.get_param(
                        std::format("{}.{}", prefix, meta.meta_name), target_member);
                }
            };
            const auto apply_function = [&]<typename... T>(MemberMeta<Data, T>... meta) {
                auto result = Ret {};
                std::ignore = ((result = deserialize(meta), result.has_value()) && ...);
                return result;
            };
            return std::apply(apply_function, metas);
        }

        auto make_printable_from(const Data& source) const -> std::string {
            auto result = std::string {};

            auto print_one = [&](const auto& meta) {
                using val_t = std::decay_t<decltype(meta.extract_from(source))>;

                if constexpr (std::formattable<val_t, char>) {
                    result += std::format("{} = {}\n", meta.meta_name, meta.extract_from(source));
                } else {
                    result += std::format("{} = ...\n", meta.meta_name);
                }
            };

            std::apply([&](const auto&... meta) { (print_one(meta), ...); }, metas);

            return result;
        }
    };

} // namespace details

struct SerializableMixin {
    template <typename Metas, std::size_t... Idx>
    constexpr auto make_serializable_impl(Metas metas, std::index_sequence<Idx...>) const {
        []<std::size_t... I>(std::index_sequence<I...>) {
            (
                []<std::size_t J>() {
                    using NameType = std::tuple_element_t<J * 2, Metas>;
                    using PtrType  = std::tuple_element_t<J * 2 + 1, Metas>;
                    static_assert(std::is_convertible_v<NameType, std::string_view>,
                        "SerializableMixin metas: Even index must be a string (name)");
                    static_assert(std::is_member_object_pointer_v<PtrType>,
                        "SerializableMixin metas: Odd index must be a member pointer");
                }.template operator()<I>(),
                ...);
        }(std::index_sequence<Idx...> {});

        return details::Serializable {
            details::MemberMeta { std::get<Idx * 2 + 1>(metas), std::get<Idx * 2>(metas) }...,
        };
    }
    template <typename Metas>
    constexpr auto make_serializable(Metas metas) const {
        constexpr auto metas_size = std::tuple_size_v<Metas>;
        return make_serializable_impl(metas, std::make_index_sequence<metas_size / 2> {});
    }

    template <typename T>
    auto serialize(this T& self, std::string_view prefix, const auto& source)
        -> std::expected<void, std::string> {

        static_assert(details::serializable_metas_trait<T>,
            "SerializableMixin T must have a valid metas tuple");

        auto s = self.make_serializable(self.metas);
        return s.serialize(prefix, source, self);
    }

    template <class T>
    auto serialize(this T& self, const auto& source) {
        static_assert(details::serializable_metas_trait<T>,
            "SerializableMixin T must have a valid metas tuple");
        return self.serialize("", source);
    }

    auto printable(this auto& self) {
        auto s = self.make_serializable(self.metas);
        return s.make_printable_from(self);
    }
};

} // namespace rmcs::util
