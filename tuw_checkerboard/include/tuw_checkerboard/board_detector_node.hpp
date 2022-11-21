#ifndef TUW_CHECKERBOARD__BOARD_DETECTOR_NODE_HPP_
#define TUW_CHECKERBOARD__BOARD_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tuw_checkerboard/visibility.h"

class PublisherNode : public rclcpp::Node
{
public:
  TUW_CHECKERBOARD_PUBLIC PublisherNode(rclcpp::NodeOptions options);

private:
  void on_timer();
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TUW_CHECKERBOARD__BOARD_DETECTOR_NODE_HPP_
