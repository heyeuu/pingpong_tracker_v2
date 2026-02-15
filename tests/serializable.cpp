#include "utility/serializable.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <vector>

using namespace pingpong_tracker::util;

static_assert(details::yaml_cpp_trait<YAML::Node>, "YAML::Node must satisfy yaml_cpp_trait");

struct T : SerializableMixin {
    int mem1 = 0;
    std::string mem2{};
    double mem3 = 0.0;
    std::vector<double> mem4{};

    static constexpr std::tuple metas{
        "mem1", &T::mem1, "mem2", &T::mem2, "mem3", &T::mem3, "mem4", &T::mem4,
    };
};

TEST(serializable, NodeAdapter) {
    auto param = std::string{};

    auto yaml_node    = YAML::Node{};
    auto yaml_adapter = details::NodeAdapter<YAML::Node>{yaml_node};

    auto ret2 = yaml_adapter.get_param("", param);
    ASSERT_FALSE(ret2.has_value()) << "Expected error for missing key on empty node";
    ASSERT_TRUE(param.empty());
}

TEST(serializable, NodeAdapterTypeMismatch) {
    // Case 1: Scalar instead of sequence (Container-level failure)
    {
        std::vector<double> vec{};
        auto node    = YAML::Node{};
        node["vec"]  = 123;
        auto adapter = details::NodeAdapter<YAML::Node>{node};
        auto ret     = adapter.get_param("vec", vec);
        ASSERT_FALSE(ret.has_value());
        EXPECT_NE(ret.error().find("Type mismatch"), std::string::npos) << "Error: " << ret.error();
    }

    // Case 2: Sequence with wrong element type (Element-level failure)
    {
        std::vector<double> vec{};
        auto node = YAML::Node{};
        node["vec"].push_back(1.0);
        node["vec"].push_back("not_a_number");
        auto adapter = details::NodeAdapter<YAML::Node>{node};
        auto ret     = adapter.get_param("vec", vec);
        ASSERT_FALSE(ret.has_value());
        EXPECT_NE(ret.error().find("Type mismatch"), std::string::npos) << "Error: " << ret.error();
    }

    // Case 3: Null node
    {
        std::vector<double> vec{};
        auto node    = YAML::Node{};
        node["vec"]  = YAML::Null;
        auto adapter = details::NodeAdapter<YAML::Node>{node};
        auto ret     = adapter.get_param("vec", vec);
        ASSERT_FALSE(ret.has_value());
        EXPECT_EQ(ret.error(), "Key 'vec' is null");
    }
}

TEST(serializable, YamlCpp) {
    auto yaml_root = YAML::LoadFile("serializable.yaml");
    auto yaml_node = yaml_root["serializable"]["ros__parameters"]["test"];

    ASSERT_TRUE(yaml_node.IsMap());

    auto t   = T{};
    auto ret = t.serialize("", yaml_node);
    ASSERT_TRUE(ret.has_value()) << "YAML Error: " << ret.error();

    std::cout << "YAML Print Data:\n" << t.printable();

    EXPECT_EQ(t.mem1, 23332);
    EXPECT_EQ(t.mem2, "this is a string");
    EXPECT_DOUBLE_EQ(t.mem3, 1.234);
    ASSERT_EQ(t.mem4.size(), 3);
    EXPECT_DOUBLE_EQ(t.mem4[0], 1.0);
    EXPECT_DOUBLE_EQ(t.mem4[1], 2.0);
    EXPECT_DOUBLE_EQ(t.mem4[2], 3.0);
}

TEST(serializable, YamlCppWithPrefix) {
    auto node           = YAML::Node{};
    node["prefix.mem1"] = 100;
    node["prefix.mem2"] = "prefixed";
    node["prefix.mem3"] = 99.9;
    node["prefix.mem4"].push_back(1.1);
    node["prefix.mem4"].push_back(2.2);

    auto t   = T{};
    auto ret = t.serialize("prefix", node);
    ASSERT_TRUE(ret.has_value()) << "YAML Error: " << ret.error();

    EXPECT_EQ(t.mem1, 100);
    EXPECT_EQ(t.mem2, "prefixed");
    EXPECT_DOUBLE_EQ(t.mem3, 99.9);
    ASSERT_EQ(t.mem4.size(), 2);
    EXPECT_DOUBLE_EQ(t.mem4[0], 1.1);
    EXPECT_DOUBLE_EQ(t.mem4[1], 2.2);
}
