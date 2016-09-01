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

#include "markermap/pose_estimation_markermap_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arPoseEstimation");

    /*
    {
        cv::Mat translation = cv::Mat::zeros(1, 3, CV_32FC1);
        cv::Mat rotation = cv::Mat::zeros(1, 4, CV_32FC1);
        rotation.at<float>(0, 0) = 1.0f;

        MarkerDetails markerTopLeft;
        markerTopLeft.id = 85;
        translation.at<float>(0, 0) = -0.036f;
        translation.at<float>(0, 1) = 0.036f;
        translation.copyTo(markerTopLeft.position);
        rotation.copyTo(markerTopLeft.rotation);

        MarkerDetails markerTopRight;
        markerTopRight.id = 166;
        translation.at<float>(0, 0) = 0.036f;
        translation.at<float>(0, 1) = 0.036f;
        translation.copyTo(markerTopRight.position);
        rotation.copyTo(markerTopRight.rotation);

        MarkerDetails markerBottomLeft;
        markerBottomLeft.id = 161;
        translation.at<float>(0, 0) = -0.036f;
        translation.at<float>(0, 1) = -0.036f;
        translation.copyTo(markerBottomLeft.position);
        rotation.copyTo(markerBottomLeft.rotation);

        MarkerDetails markerBottomRight;
        markerBottomRight.id = 227;
        translation.at<float>(0, 0) = 0.036f;
        translation.at<float>(0, 1) = -0.036f;
        translation.copyTo(markerBottomRight.position);
        rotation.copyTo(markerBottomRight.rotation);

        MarkerMapDetails markerMap;
        markerMap.id = 1001;
        markerMap.markers.push_back(markerTopLeft);
        markerMap.markers.push_back(markerTopRight);
        markerMap.markers.push_back(markerBottomLeft);
        markerMap.markers.push_back(markerBottomRight);

        MarkerMapConfig markerMapConfig;
        markerMapConfig.markerMaps.push_back(markerMap);

        cv::FileStorage fs("/home/privacy/Documents/ros/workspace_repo/test_marker_config.yml", cv::FileStorage::WRITE);
        fs << "markerMapConfig" << markerMapConfig;
        fs.release();
    }
    */


    // Try to get config file path
    std::string configPath;

    ros::NodeHandle pn("~");
    if (!pn.getParam("marker_map_config", configPath)) {
        ROS_ERROR("Parameter marker_map_config (Path to config file) is not provided. This node can not be started without a valid MarkerMap config.");
        return EXIT_FAILURE;
    }

    // Try to read config file
    MarkerMapConfig markerMapConfig = MarkerMapConfig::readFromFile(configPath);
    if (markerMapConfig.markerMaps.size() <= 0) {
        ROS_ERROR("The provided MarkerMap config file (%s) is not existing, is invalid or contains not a single markerMap. This node can not be started without a valid MarkerMap config.", configPath.c_str());
        return EXIT_FAILURE;
    }

    ros::NodeHandle n;
    PoseEstimationMarkerMapNode poseEstimationNode(n, markerMapConfig);
    ros::spin();
    return EXIT_SUCCESS;
}

PoseEstimationMarkerMapNode::PoseEstimationMarkerMapNode(ros::NodeHandle &n, MarkerMapConfig markerMapConfig) : n_(n), base_(markerMapConfig) {
    // Register dynamic_reconfigure callback
    configCallbackFnct_ = boost::bind(&PoseEstimationMarkerMapNode::configCallback, this, _1, _2);
    configServer_.setCallback(configCallbackFnct_);

    // Advert marker publisher
    pub_markers_ = n_.advertise<marker_msgs::MarkerDetection>("markers", 10);

    // Subscribe to FiducialDetection.msg topic
    fiducialDetectionSubscriber_ = n.subscribe("fiducials", 10, &PoseEstimationMarkerMapNode::fiducialDetectionCallback, this);
}

PoseEstimationMarkerMapNode::~PoseEstimationMarkerMapNode() {}

void PoseEstimationMarkerMapNode::fiducialDetectionCallback(const marker_msgs::FiducialDetection::ConstPtr &msg) {

    // Convert camera matrix msg to cv::Mat
    float camera_matrix_data[9];
    for(int i = 0; i < 9; i++)
        camera_matrix_data[i] = msg->camera_k[i];
    cv::Mat camera_k = cv::Mat(3, 3, CV_32F, camera_matrix_data);

    float distortion_coefficients_data[5];
    for(int i = 0; i < 5; i++)
        distortion_coefficients_data[i] = msg->camera_d[i];
    cv::Mat camera_d = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);


    std::vector<MarkerFiducials> markerFiducials;
    for (auto &fiducial:msg->fiducial) {
        MarkerFiducials marker(fiducial.ids, fiducial.ids_confidence);

        for (auto &object_point:fiducial.object_points)
            marker.object_points.push_back(cv::Point3f(object_point.x, object_point.y, object_point.z));

        for (auto &image_point:fiducial.image_points)
            marker.image_points.push_back(cv::Point2f(image_point.x, image_point.y));

        markerFiducials.push_back(marker);
    }

    // Do pose estimation
    std::vector<MarkerPose> markerPoses;
    base_.estimatePose(markerFiducials, camera_k, camera_d, markerPoses);

    // Publish ros messages
    publishMarkers(msg->header, markerPoses);
}

static tf::StampedTransform markerPoseToStampedTransform(MarkerPose &markerPose, const std_msgs::Header &header) {
    cv::Mat m = markerPose.rt_matrix;

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
    if (markerPose.ids.size() > 0) {
        sprintf(markerLabel, "t%i", markerPose.ids[0]);
    } else {
        sprintf(markerLabel, "t?");
    }
    return tf::StampedTransform(tf::Transform(rm, tv), ros::Time::now(), header.frame_id, markerLabel);
}

void PoseEstimationMarkerMapNode::publishMarkers(const std_msgs::Header &header, std::vector<MarkerPose> &markerPoses) {
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

        marker.ids = markerPose.ids;
        marker.ids_confidence = markerPose.ids_confidence;
        tf::poseTFToMsg(stf, marker.pose);

        msg.markers.push_back(marker);
    }

    // Publish MarkerDetection message
    if (base_.getParameters().getPublishMarkers())
        pub_markers_.publish(msg);
}

void PoseEstimationMarkerMapNode::configCallback(tuw_marker_pose_estimation::MarkerPoseEstimationConfig &config, uint32_t level) {
    base_.getParameters().setPoseEstimatorType(config.pose_estimation_type);
    base_.getParameters().setPublishTf(config.publish_tf);
    base_.getParameters().setPublishMarkers(config.publish_markers);
    base_.refreshParameters();
}
