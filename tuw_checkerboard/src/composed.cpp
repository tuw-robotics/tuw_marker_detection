#include <memory>
#include "tuw_checkerboard/checkerboard_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto publisher_node = std::make_shared<CheckerboardDetectionNode>(options);
  exec.add_node(publisher_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
