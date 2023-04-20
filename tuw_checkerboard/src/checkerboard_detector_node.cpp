#include <chrono>

#include "tuw_checkerboard/checkerboard_detector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

CheckerboardDetectionNode::CheckerboardDetectionNode(rclcpp::NodeOptions options)
    : Node("publisher_node", options), count_(0)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
      500ms, std::bind(&CheckerboardDetectionNode::on_timer, this));
}

void CheckerboardDetectionNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void CheckerboardDetectionNode::declare_parameters()
{

  auto declare_default_parameter = [this](
                                       const std::string &name,
                                       double value_default,
                                       double min, double max, double step,
                                       const std::string &description)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.floating_point_range = {range};
    descriptor.description = description;
    this->declare_parameter<double>(name, value_default, descriptor);
  };

  declare_default_parameter("map_grid", 1, 0.001, 10, 0.001, "grid size [m]");

  callback_update_parameters();
  using namespace std::chrono_literals;
  timer_update_parameter_ =
      this->create_wall_timer(
          1000ms,
          std::bind(&CheckerboardDetectionNode::callback_update_parameters, this));
}

void CheckerboardDetectionNode::callback_update_parameters() {}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(CheckerboardDetectionNode)
