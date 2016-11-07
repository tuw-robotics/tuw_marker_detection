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

#ifndef TUW_ARUCO_ARUCO_NODE_H
#define TUW_ARUCO_ARUCO_NODE_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/FiducialDetection.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <tuw_aruco/ArUcoConfig.h>

#include "aruco.h"
#include "tuw_aruco/aruco_base.h"

class ArUcoNode {
public:
    ArUcoNode(ros::NodeHandle &n);

    ~ArUcoNode();

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport imageTransport_;
    image_transport::CameraSubscriber cameraSubscriber_;

    tf::TransformBroadcaster transformBroadcaster_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_fiducials_;

    dynamic_reconfigure::Server<tuw_aruco::ArUcoConfig> configServer_;
    dynamic_reconfigure::Server<tuw_aruco::ArUcoConfig>::CallbackType configCallbackFnct_;

    ArUcoBase base_;

    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &camer_info_);

    void publishMarkers(const std_msgs::Header &header, vector<ArUcoMarkerPose> &markerPoses);

    void publishFiducials(const std_msgs::Header &header, vector<aruco::Marker> &markers, const sensor_msgs::CameraInfoConstPtr &camer_info_);

    void configCallback(tuw_aruco::ArUcoConfig &config, uint32_t level);
};

#endif // TUW_ARUCO_ARUCO_NODE_H