#include <memory>
#include "tuw_checkerboard/board_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
