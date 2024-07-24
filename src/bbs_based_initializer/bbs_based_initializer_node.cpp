#include <bbs_based_initializer.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BbsBasedInitializer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}