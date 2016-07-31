#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "aruco.h"

static aruco::MarkerDetector detector;
static std::map<uint32_t, aruco::MarkerPoseTracker> tracker;
static aruco::CameraParameters camParams;

void imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);


        // Detect markers
        vector< aruco::Marker > markers;
        markers = detector.detect(img->image);

        for (auto &marker:markers)//for each marker
            tracker[marker.id].estimatePose(marker, camParams, 6.0f); //call its tracker and estimate the pose


        // Draw markers
        cv::Mat debugImage = cv::Mat::zeros(640, 480, CV_8UC3);
        cvtColor(img->image, debugImage, cv::COLOR_GRAY2BGR);

        for (unsigned int i = 0; i < markers.size(); i++) {
            // draw 2d info
            markers[i].draw(debugImage, cv::Scalar(0, 0, 255), 1);

            // draw a 3d cube
            aruco::CvDrawingUtils::draw3dCube(debugImage, markers[i], camParams);
            aruco::CvDrawingUtils::draw3dAxis(debugImage, markers[i], camParams);
        }

        cv::imshow("aruco_node_debug", debugImage);
        cv::waitKey ( 5 );
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arMarker");

    ros::NodeHandle n;


    detector.setDictionary(aruco::Dictionary::ARTOOLKITPLUSBCH);//sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    detector.setThresholdParams(7, 7);
    detector.setThresholdParamRange(2, 0);


    // Camera parameters
    float camera_matrix_data[9] = {844.196880, 0.000000, 318.514635, 0.000000, 852.683290, 254.889584, 0.000000, 0.000000, 1.000000};
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, camera_matrix_data);

    float distortion_coefficients_data[5] = {0.130185, -0.499155, -0.009280, -0.003387, 0.000000};
    cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    camParams = aruco::CameraParameters(camera_matrix, distortion_coefficients, cv::Size(640, 480));

    cv::namedWindow("aruco_node_debug");

    image_transport::ImageTransport imageTransport(n);
    image_transport::CameraSubscriber cameraSubscriber = imageTransport.subscribeCamera("image", 1, &imageCallback);


    ros::spin();
    return 0;
}