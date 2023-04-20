#ifndef TUW_CHECKERBOARD__CHECKERBOARD_DETECTION_NODE_HPP_
#define TUW_CHECKERBOARD__CHECKERBOARD_DETECTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tuw_checkerboard/visibility.h"

class CheckerboardDetectionNode : public rclcpp::Node
{
public:
  TUW_CHECKERBOARD_PUBLIC CheckerboardDetectionNode(rclcpp::NodeOptions options);

private:
  void on_timer();
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void declare_parameters();                            /// ros parameters declares
  void callback_update_parameters();                    /// ros parameters callback to check changes on the parameters
  rclcpp::TimerBase::SharedPtr timer_update_parameter_; /// ros parameters timer to check regularly for changes
};

#endif // TUW_CHECKERBOARD__CHECKERBOARD_DETECTION_NODE_HPP_
