#include "tuw_checkerboard/board_pose_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

SubscriberNode::SubscriberNode(rclcpp::NodeOptions options)
: Node("subscriber_node", options)
{
  subscription_ = create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Subscriber: '%s'", msg->data.c_str());
    });
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberNode)
