#ifndef TUW_CHECKERBOARD_NODE_H
#define TUW_CHECKERBOARD_NODE_H

#include <ros/ros.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/FiducialDetection.h>
#include <tuw_checkerboard/CheckerboardDetectionConfig.h>


class CheckerboardNode
{
public:
  CheckerboardNode(); // Constructor
private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_fiducials_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  image_geometry::PinholeCameraModel cam_model_;
  image_transport::CameraSubscriber sub_cam_;
  cv::Mat image_grey_;
  cv::Mat image_rgb_;
  tuw_checkerboard::CheckerboardDetectionConfig config_;
  std::string checkerboard_frame_id_;
  
  
  void callbackCamera(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  dynamic_reconfigure::Server<tuw_checkerboard::CheckerboardDetectionConfig>* reconfigureServer_; ///< parameter server stuff
  dynamic_reconfigure::Server<tuw_checkerboard::CheckerboardDetectionConfig>::CallbackType reconfigureFnc_;///< parameter server stuff
  void callbackConfig ( tuw_checkerboard::CheckerboardDetectionConfig &_config, uint32_t _level ); ///< callback function on incoming parameter changes
};

#endif //TUW_CHECKERBOARD_NODE_H