// Copyright 2026, MIGHTY Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "px4_mighty_bridge/px4_mighty_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4_mighty_bridge::Px4MightyBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
