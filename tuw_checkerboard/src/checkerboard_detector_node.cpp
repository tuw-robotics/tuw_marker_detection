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

  auto declare_double_parameter = [this](
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
  auto declare_integer_parameter = [this](
                                       const std::string &name,
                                       int value_default,
                                       int min, int max, int step,
                                       const std::string &description)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(min).set__to_value(max).set__step(step);
    descriptor.integer_range = {range};
    descriptor.description = description;
    this->declare_parameter<double>(name, value_default, descriptor);
  };

  auto declare_boolean_parameter = [this](
                                       const std::string &name,
                                       bool value_default,
                                       const std::string &description)
  {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
        descriptor.description = description;
        this->declare_parameter<bool>(name, value_default, descriptor);
  };

  declare_boolean_parameter("show_camera_image", true, "Show camera image");
  declare_integer_parameter("show_camera_image_waitkey", 2, 1, 1, 1000, "Time to draw the camera image in ms");
  declare_boolean_parameter("input_raw", true, "The input image is distored");
  declare_boolean_parameter("rotate_camera_image_180", false, "Rotate input camera image by 180 deg");
  declare_boolean_parameter("plubish_tf", true, "publish tf messages");
  declare_boolean_parameter("plubish_marker", true, "publish marker messages");
  declare_boolean_parameter("publish_fiducials", false, "publish fiducials to allow 3th party pose estimation");
  declare_boolean_parameter("publish_pose", false, "publish checkerboard pose");
  declare_integer_parameter("checkerboard_columns", 8, 1, 1, 20, "interior number of colomn corners");
  declare_integer_parameter("checkerboard_rows", 6, 1, 1, 20, "interior number of row corners");
  declare_double_parameter("checkerboard_square_size", 0.03, 0.001, 1, 0.001, "checkerboard square size [m]");
  declare_integer_parameter("checkerboard_min_square_size", 10, 1, 1, 100, "minimal checkerboard square size in [pix] to accept a detection");
  declare_boolean_parameter("adaptive_thresh", true, "checkerboard detection flags");
  declare_boolean_parameter("normalize_image", false, "checkerboard detection flags");
  declare_boolean_parameter("filter_quads", false, "checkerboard detection flags");
  declare_boolean_parameter("fast_check", false, "checkerboard detection flags");
  declare_boolean_parameter("subpixelfit", true, "Sub-pixel accurate corner locator");
  declare_integer_parameter("subpixelfit_window_size", 8, 1, 20, 1, "Half of the side length of the search window [pix]");

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
