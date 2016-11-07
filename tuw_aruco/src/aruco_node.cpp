/*
 * Copyright (c) 2016, Lukas Pfeifhofer <lukas.pfeifhofer@devlabs.pro>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "tuw_aruco/aruco_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arMarker");
    ros::NodeHandle n;
    ArUcoNode arUcoNode(n);
    ros::spin();
    return 0;
}

ArUcoNode::ArUcoNode(ros::NodeHandle &n) : n_(n), imageTransport_(n) {

    // Register dynamic_reconfigure callback
    configCallbackFnct_ = boost::bind(&ArUcoNode::configCallback, this ,  _1, _2);
    configServer_.setCallback(configCallbackFnct_);

    // Advert marker publisher
    pub_markers_ = n_.advertise<marker_msgs::MarkerDetection>("markers", 10);

    // Advert fiducial publisher
    pub_fiducials_ = n_.advertise<marker_msgs::FiducialDetection>("fiducials", 10);

    // Subscribe to image topic
    cameraSubscriber_ = imageTransport_.subscribeCamera("image", 1, &ArUcoNode::imageCallback, this);
    
    if(base_.getParameters().getShowDebugImage()){
      cv::namedWindow("aruco_node_debug", CV_WINDOW_NORMAL | CV_GUI_NORMAL);
    }

}

ArUcoNode::~ArUcoNode() {}

static aruco::CameraParameters cameraInfoToCameraParameters(const sensor_msgs::CameraInfoConstPtr &camer_info) {
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

static tf::StampedTransform markerPoseToStampedTransform(ArUcoMarkerPose &markerPose, const std_msgs::Header &header) {
    cv::Mat m = markerPose.getRTMatrix();

    tf::Vector3 tv(
            m.at<float>(0, 3),
            m.at<float>(1, 3),
            m.at<float>(2, 3)
    );

    tf::Matrix3x3 rm(
            m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2),
            m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2),
            m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2)
    );

    char markerLabel[64];
    sprintf(markerLabel, "t%i", markerPose.getMarkerId());
    return tf::StampedTransform(tf::Transform(rm, tv), ros::Time::now(), header.frame_id, markerLabel);
}

void ArUcoNode::publishMarkers(const std_msgs::Header &header, vector<ArUcoMarkerPose> &markerPoses) {
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

    for (auto &markerPose:markerPoses) {
        tf::StampedTransform stf = markerPoseToStampedTransform(markerPose, header);

        // Send transform
        if (base_.getParameters().getPublishTf())
            transformBroadcaster_.sendTransform(stf);

        // Push marker into MarkerDetection message
        marker_msgs::Marker marker;

        marker.ids.resize(1);
        marker.ids_confidence.resize(1);
        marker.ids[0] = markerPose.getMarkerId();
        marker.ids_confidence[0] = 1;
        tf::poseTFToMsg(stf, marker.pose);

        msg.markers.push_back(marker);
    }

    // Publish MarkerDetection message
    if (base_.getParameters().getPublishMarkers())
        pub_markers_.publish(msg);
}

void ArUcoNode::publishFiducials(const std_msgs::Header &header, vector<aruco::Marker> &markers, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    marker_msgs::FiducialDetection msg;
    msg.header = header;
    msg.camera_k = camer_info_->K;
    msg.camera_d = camer_info_->D;

    for (auto &marker:markers) {
        marker_msgs::Fiducial fiducial;

        // Add all object points
        for (auto &cvp: marker.get3DPoints(base_.getParameters().getMarkerSize())) {
            geometry_msgs::Point point;
            point.x = cvp.x;
            point.y = cvp.y;
            point.z = cvp.z;
            fiducial.object_points.push_back(point);
        }

        // Add all images points
        for (cv::Point2f &p2d: marker) {
            geometry_msgs::Point point;
            point.x = p2d.x;
            point.y = p2d.y;
            point.z = 0.0f; // imagePoints are 2d
            fiducial.image_points.push_back(point);
        }

        fiducial.ids.resize(1);
        fiducial.ids_confidence.resize(1);
        fiducial.ids[0] = marker.id;
        fiducial.ids_confidence[0] = 1;

        msg.fiducial.push_back(fiducial);
    }

    pub_fiducials_.publish(msg);
}

void ArUcoNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_) {
    cv_bridge::CvImagePtr imgPtr;
    try {
        // Convert ros image message to cv::Mat
        imgPtr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        // Convert ros camera parameter
        aruco::CameraParameters camParams = cameraInfoToCameraParameters(camer_info_);


        // Detect markers
        vector<aruco::Marker> markers;
        base_.detectMarkers(markers, imgPtr->image);

        // If enabled publish fiducials
        if(base_.getParameters().getPublishFiducials())
            publishFiducials(image_msg->header, markers, camer_info_);

        // If enabled do pose estimation for every marker found
        if (base_.getParameters().getPoseEstimationEnabled()) {
            vector<ArUcoMarkerPose> markerPoses;
            base_.estimatePose(markerPoses, markers, camParams);


            // Publish markers
            publishMarkers(image_msg->header, markerPoses);
        }


        // Draw markers if debug image is enabled
        if(base_.getParameters().getShowDebugImage()){
            cv::Mat debugImage;
            cvtColor(imgPtr->image, debugImage, cv::COLOR_GRAY2BGR);

            for (unsigned int i = 0; i < markers.size(); i++) {
                // draw 2d info
                markers[i].draw(debugImage, cv::Scalar(0, 0, 255), 1);

                // draw a 3d cube
                aruco::CvDrawingUtils::draw3dCube(debugImage, markers[i], camParams);
                aruco::CvDrawingUtils::draw3dAxis(debugImage, markers[i], camParams);
            }

            cv::imshow("aruco_node_debug", debugImage);
            cv::waitKey(5);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
        return;
    }
}

void ArUcoNode::configCallback(tuw_aruco::ArUcoConfig &config, uint32_t level) {
    base_.getParameters().setShowDebugImage(config.show_debug_image);
    base_.getParameters().setDictionary(config.marker_dictonary);
    base_.getParameters().setMarkerSize(config.marker_size);
    base_.getParameters().setPublishTf(config.publish_tf);
    base_.getParameters().setPublishMarkers(config.publish_markers);
    base_.getParameters().setPublishFiducials(config.publish_fiducials);
    base_.getParameters().setPoseEstimationEnabled(config.pose_estimation_enabled);
    base_.refreshParameters();
}

