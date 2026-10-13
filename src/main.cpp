// px4ctrl2 process entrypoint. ROS-facing implementation lives in px4ctrl_node.cpp.
#include <rclcpp/rclcpp.hpp>
#include "px4ctrl2/px4ctrl2_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<px4ctrl2::Node>());
  } catch (const std::exception &e) {
    fprintf(stderr, "px4ctrl2: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
