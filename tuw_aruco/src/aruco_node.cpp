#include "aruco_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arMarker");
    ros::NodeHandle n;
    ArUcoNode arUcoNode(n);
    ros::spin();
    return 0;
}

ArUcoNode::ArUcoNode(ros::NodeHandle &n) : _n(n), _imageTransport(n) {
    _detector.setDictionary(aruco::Dictionary::ARTOOLKITPLUSBCH);//sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    _detector.setThresholdParams(7, 7);
    _detector.setThresholdParamRange(2, 0);


    // Camera parameters
    float camera_matrix_data[9] = {844.196880, 0.000000, 318.514635, 0.000000, 852.683290, 254.889584, 0.000000, 0.000000, 1.000000};
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, camera_matrix_data);

    float distortion_coefficients_data[5] = {0.130185, -0.499155, -0.009280, -0.003387, 0.000000};
    cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    _camParams = aruco::CameraParameters(camera_matrix, distortion_coefficients, cv::Size(640, 480));

    // Subscribe to image topic
    _cameraSubscriber = _imageTransport.subscribeCamera("image", 1, &ArUcoNode::imageCallback, this);

    cv::namedWindow("aruco_node_debug");
}

ArUcoNode::~ArUcoNode() {}

void ArUcoNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);


        // Detect markers
        vector< aruco::Marker > markers;
        markers = _detector.detect(img->image);

        // Estimate marker pose and create transforms
        _markerTransforms.clear();

        tf::StampedTransform st;
        for (auto &marker:markers) {

            bool success = _tracker[marker.id].estimatePose(marker, _camParams, 0.06f); //call its tracker and estimate the pose
            if(success){
                cv::Mat m = _tracker[marker.id].getRTMatrix();

                float tX = m.at<float>(0, 3);
                float tY = m.at<float>(1, 3);
                float tZ = m.at<float>(2, 3);

                tf::Matrix3x3 rm(
                    m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2),
                    m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2),
                    m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2)
                );

                char markerLabel[32];
                sprintf(markerLabel, "t%i", marker.id);

                st = tf::StampedTransform(tf::Transform(rm, tf::Vector3(tX, tY, tZ)), ros::Time::now(), image_msg->header.frame_id, markerLabel);
                _markerTransforms.push_back(st);
            }
        }

        // Broadcast transforms
        for(std::list<tf::StampedTransform>::iterator it =  _markerTransforms.begin(); it != _markerTransforms.end(); it++) {
            _transformBroadcaster.sendTransform(*it);
        }

        // Draw markers
        cv::Mat debugImage = cv::Mat::zeros(640, 480, CV_8UC3);
        cvtColor(img->image, debugImage, cv::COLOR_GRAY2BGR);

        for (unsigned int i = 0; i < markers.size(); i++) {
            // draw 2d info
            markers[i].draw(debugImage, cv::Scalar(0, 0, 255), 1);

            // draw a 3d cube
            aruco::CvDrawingUtils::draw3dCube(debugImage, markers[i], _camParams);
            aruco::CvDrawingUtils::draw3dAxis(debugImage, markers[i], _camParams);
        }

        cv::imshow("aruco_node_debug", debugImage);
        cv::waitKey ( 5 );
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
        return;
    }
}

