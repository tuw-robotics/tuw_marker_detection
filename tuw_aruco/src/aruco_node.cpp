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

    // Advert marker publisher
    _pub_markers = _n.advertise<marker_msgs::MarkerDetection>("markers", 10);

    // Subscribe to image topic
    _cameraSubscriber = _imageTransport.subscribeCamera("image", 1, &ArUcoNode::imageCallback, this);

    cv::namedWindow("aruco_node_debug");
}

ArUcoNode::~ArUcoNode() {}

static aruco::CameraParameters cameraInfoToCameraParameters(const sensor_msgs::CameraInfoConstPtr &camer_info){
    float camera_matrix_data[9];
    for(int i = 0; i < 9; i++)
        camera_matrix_data[i] = camer_info->K[i];
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, camera_matrix_data);

    float distortion_coefficients_data[5];
    for(int i = 0; i < 5; i++)
        distortion_coefficients_data[i] = camer_info->D[i];
    cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    return aruco::CameraParameters(camera_matrix, distortion_coefficients, cv::Size(camer_info->width, camer_info->height));
}

void ArUcoNode::publishMarkers(const std_msgs::Header &header) {
    marker_msgs::MarkerDetection msg;

    msg.header = header;
    msg.distance_min =  0; //TODO
    msg.distance_max =  8; //TODO
    msg.distance_max_id = 5; //TODO
    msg.view_direction.x = 0; //TODO
    msg.view_direction.y = 0; //TODO
    msg.view_direction.z = 0; //TODO
    msg.view_direction.w = 1; //TODO
    msg.fov_horizontal = 6; //TODO
    msg.fov_vertical = 0; //TODO

    msg.markers = marker_msgs::MarkerDetection::_markers_type();

    for(std::list<tf::StampedTransform>::iterator it =  _markerTransforms.begin(); it != _markerTransforms.end(); it++) {
        tf::StampedTransform stf = *it;

        // Send transform
        _transformBroadcaster.sendTransform(stf);

        // Push marker into MarkerDetection message
        marker_msgs::Marker marker;

        marker.ids.resize(1);
        marker.ids_confidence.resize(1);
        marker.ids[0] = 0; // markerTransformsID_[i]; -- TODO: Fix me, marker id missing
        marker.ids_confidence[0] = 1;
        tf::poseTFToMsg(stf, marker.pose);

        msg.markers.push_back(marker);
    }

    // Publish MarkerDetection message
    _pub_markers.publish(msg);
}

void ArUcoNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr img;
    try {
        img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        // Convert ros camera parameter
        aruco::CameraParameters camParams = cameraInfoToCameraParameters(camer_info_);

        // Detect markers
        vector< aruco::Marker > markers;
        markers = _detector.detect(img->image);

        // Estimate marker pose and create transforms
        _markerTransforms.clear();

        tf::StampedTransform st;
        for (auto &marker:markers) {

            bool success = _tracker[marker.id].estimatePose(marker, camParams, 0.06f); //call its tracker and estimate the pose
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

        // Broadcast markers and transforms
        publishMarkers(image_msg->header);

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

