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

#ifndef TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_COMBOMARKER_NODE_H
#define TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_COMBOMARKER_NODE_H

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/FiducialDetection.h>
#include "marker_pose.h"

#include <dynamic_reconfigure/server.h>

class PoseEstimationComboMarkerNode {
public:
    PoseEstimationComboMarkerNode(ros::NodeHandle &n);

    ~PoseEstimationComboMarkerNode();

private:
    ros::NodeHandle n_;

    tf::TransformBroadcaster transformBroadcaster_;
    ros::Publisher pub_markers_;

    message_filters::Subscriber<marker_msgs::FiducialDetection> idFiducialDetectionSubscriber_;
    message_filters::Subscriber<marker_msgs::FiducialDetection> ellipseFiducialDetectionSubscriber_;

    message_filters::TimeSynchronizer<marker_msgs::FiducialDetection, marker_msgs::FiducialDetection> sync_;


    void synchronizedFiducialsCallback(const marker_msgs::FiducialDetection::ConstPtr &msgIdFiducialDetection,
                                       const marker_msgs::FiducialDetection::ConstPtr &msgEllipseFiducialDetection);

    void publishMarkers(const std_msgs::Header &header, std::vector<MarkerPose> &markerPoses);

};

#endif //TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_COMBOMARKER_NODE_H
