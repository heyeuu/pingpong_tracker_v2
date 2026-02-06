#include "utility/serializable.hpp"

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

using namespace rmcs::util;

static_assert(details::yaml_cpp_trait<YAML::Node>, " ");

struct T : Serializable {
  int mem1 = 0;
  std::string mem2{};
  double mem3 = 0.0;
  std::vector<double> mem4{};

  static constexpr std::tuple metas{
      &T::mem1, "mem1", &T::mem2, "mem2", &T::mem3, "mem3", &T::mem4, "mem4",
  };
};

TEST(serializable, node_adapter) {
  auto param = std::string{};

  static_assert(details::yaml_cpp_trait<YAML::Node>, " ");

  auto yaml_node = YAML::Node{};
  auto yaml_adapter = details::NodeAdapter<YAML::Node>{yaml_node};

  auto ret2 = yaml_adapter.get_param("", param);
}

// @NOTE:
//  只引入 yaml-cpp/node/node.h 会找不到链接符号
//  哭（
TEST(serializable, yaml_cpp) {
  using namespace rmcs::util;

  auto yaml_root = YAML::LoadFile("serializable.yaml");
  auto yaml_node = yaml_root["serializable"]["ros__parameters"]["test"];

  ASSERT_TRUE(yaml_node.IsMap());

  auto t = T{};
  auto adapter = details::NodeAdapter<YAML::Node>{yaml_node};

  auto ret = t.serialize("", adapter);
  if (!ret.has_value()) {
    std::cerr << "YAML Error: " << ret.error() << "\n";
    GTEST_FAIL();
  }

  std::cout << "YAML Print Data:\n" << t.printable();
}
