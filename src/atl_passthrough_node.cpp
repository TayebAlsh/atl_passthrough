// Copyright 2023 ATL Robotics, USA

#include "atl_passthrough/atl_passthrough_component.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec{};
  auto options = rclcpp::NodeOptions{};
  auto atl_passthrough_comp = std::make_shared<atl::AtlPassthroughComponent>(options);

  exec.add_node(atl_passthrough_comp);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}

// test the ssh github