#ifndef TUW_CHECKERBOARD__BOARD_POSE_NODE_HPP_
#define TUW_CHECKERBOARD__BOARD_POSE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tuw_checkerboard/visibility.h"

class SubscriberNode : public rclcpp::Node
{
public:
  TUW_CHECKERBOARD_PUBLIC SubscriberNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // TUW_CHECKERBOARD__BOARD_POSE_NODE_HPP_
