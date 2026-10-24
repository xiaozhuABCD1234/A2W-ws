#include <rclcpp/rclcpp.hpp>

#include <memory>

// TODO(M1): 完整节点（订阅、关键帧、回环、位姿图、TF/话题输出）。
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("point_lio_backend");
  RCLCPP_INFO(node->get_logger(), "point_lio_backend skeleton: waiting for implementation");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
