#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat img_debug;
        cvtColor(img->image, img_debug, CV_GRAY2BGR);
        cv::imshow("aruco_node_debug", img_debug);
        cv::waitKey ( 5 );
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arMarker");

    ros::NodeHandle n;

    cv::namedWindow("aruco_node_debug");

    image_transport::ImageTransport imageTransport(n);
    image_transport::CameraSubscriber cameraSubscriber = imageTransport.subscribeCamera("image", 1, &imageCallback);


    ros::spin();
    return 0;
}