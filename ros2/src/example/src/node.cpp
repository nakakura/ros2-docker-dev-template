#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("example_node");
  RCLCPP_INFO(node->get_logger(), "hello ros2");
  rclcpp::shutdown();
  return 0;
}
