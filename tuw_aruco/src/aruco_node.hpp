#ifndef ARUCO_NODE_H
#define ARUCO_NODE_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <marker_msgs/MarkerDetection.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "aruco.h"

class ArUcoNode {
public:
    ArUcoNode(ros::NodeHandle &n);

    ~ArUcoNode();

private:
    ros::NodeHandle _n;

    image_transport::ImageTransport _imageTransport;
    image_transport::CameraSubscriber _cameraSubscriber;

    ros::Publisher _pub_markers;

    aruco::MarkerDetector _detector;
    std::map<uint32_t, aruco::MarkerPoseTracker> _tracker;

    tf::TransformBroadcaster _transformBroadcaster;
    std::list<tf::StampedTransform> _markerTransforms;


    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_);
    void publishMarkers(const std_msgs::Header &header);
};

#endif // ARUCO_NODE_H