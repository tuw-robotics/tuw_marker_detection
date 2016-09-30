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

#include "combomarker/pose_estimation_combomarker_node.h"

#include "marker_fiducials.h"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "pose_estimation_base.h"
#include "combomarker/combo_marker_estimator.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "arPoseEstimation");
    ros::NodeHandle n;
    PoseEstimationComboMarkerNode poseEstimationNode(n);
    ros::spin();
    return EXIT_SUCCESS;
}

PoseEstimationComboMarkerNode::PoseEstimationComboMarkerNode(ros::NodeHandle &n) : n_(n),
                                                                                   idFiducialDetectionSubscriber_(n_, "idFiducials", 100),
                                                                                   ellipseFiducialDetectionSubscriber_(n_, "ellipseFiducials", 100),
                                                                                   sync_(idFiducialDetectionSubscriber_, ellipseFiducialDetectionSubscriber_, 10){
    // Advert marker publisher
    pub_markers_ = n_.advertise<marker_msgs::MarkerDetection>("markers", 10);

    sync_.registerCallback(boost::bind(&PoseEstimationComboMarkerNode::synchronizedFiducialsCallback, this, _1, _2));
}

PoseEstimationComboMarkerNode::~PoseEstimationComboMarkerNode() {}


void PoseEstimationComboMarkerNode::synchronizedFiducialsCallback(const marker_msgs::FiducialDetection::ConstPtr &msgIdFiducialDetection,
                                                                  const marker_msgs::FiducialDetection::ConstPtr &msgEllipseFiducialDetection){
    //ROS_INFO("fired synchronizedFiducialsCallback: %d, %d", msgIdFiducialDetection->header.seq, msgEllipseFiducialDetection->header.seq);

    // Convert camera matrix msg to cv::Mat
    float camera_matrix_data[9];
    for (int i = 0; i < 9; i++)
        camera_matrix_data[i] = msgIdFiducialDetection->camera_k[i];
    cv::Mat camera_k = cv::Mat(3, 3, CV_32F, camera_matrix_data);

    float distortion_coefficients_data[5];
    for (int i = 0; i < 5; i++)
        distortion_coefficients_data[i] = msgIdFiducialDetection->camera_d[i];
    cv::Mat camera_d = cv::Mat(1, 5, CV_32F, distortion_coefficients_data);

    std::vector<MarkerFiducials> idFiducials;
    for (auto &fiducial:msgIdFiducialDetection->fiducial) {
        MarkerFiducials mFiducial(fiducial.ids, fiducial.ids_confidence);

        for (auto &object_point:fiducial.object_points)
            mFiducial.object_points.push_back(cv::Point3f(object_point.x, object_point.y, object_point.z));

        for (auto &image_point:fiducial.image_points)
            mFiducial.image_points.push_back(cv::Point2f(image_point.x, image_point.y));

        idFiducials.push_back(mFiducial);
    }

    std::vector<MarkerFiducials> ellipseFiducials;
    for (auto &fiducial:msgEllipseFiducialDetection->fiducial) {
        MarkerFiducials mFiducial(fiducial.ids, fiducial.ids_confidence);

        for (auto &object_point:fiducial.object_points)
            mFiducial.object_points.push_back(cv::Point3f(object_point.x, object_point.y, object_point.z));

        for (auto &image_point:fiducial.image_points)
            mFiducial.image_points.push_back(cv::Point2f(image_point.x, image_point.y));

        ellipseFiducials.push_back(mFiducial);
    }


    // Do pose estimation
    std::vector<MarkerPose> markerPoses;
    ComboMarkerEstimator estimator;
    estimator.estimatePose(idFiducials, camera_k, camera_d, ellipseFiducials, markerPoses);

    // Publish ros messages
    publishMarkers(msgIdFiducialDetection->header, markerPoses);

    if(true){

        cv::Mat debugImage = cv::Mat(480, 640, CV_32FC3, cv::Scalar(255, 255, 255));

        for (auto &markerPose:markerPoses) {
            cv::Mat objectPoints(4, 3, CV_32FC1);
            float axis_size = 0.08f;
            objectPoints.at< float >(0, 0) = 0;
            objectPoints.at< float >(0, 1) = 0;
            objectPoints.at< float >(0, 2) = 0;
            objectPoints.at< float >(1, 0) = axis_size;
            objectPoints.at< float >(1, 1) = 0;
            objectPoints.at< float >(1, 2) = 0;
            objectPoints.at< float >(2, 0) = 0;
            objectPoints.at< float >(2, 1) = axis_size;
            objectPoints.at< float >(2, 2) = 0;
            objectPoints.at< float >(3, 0) = 0;
            objectPoints.at< float >(3, 1) = 0;
            objectPoints.at< float >(3, 2) = axis_size;

            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(objectPoints, markerPose.getRVec(),  markerPose.getTVec(), camera_k, camera_d, imagePoints);

            // draw lines of different colours
            cv::line(debugImage, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::line(debugImage, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0, 255), 1, CV_AA);
            cv::line(debugImage, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0, 255), 1, CV_AA);
        }




        //std::vector< cv::Point2f > imagePoints;
        //cv::projectPoints(objectPoints, Rvec,  Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);

        /*
        cv::Mat debugImage;
        cvtColor(imgPtr->image, debugImage, cv::COLOR_GRAY2BGR);

        for (unsigned int i = 0; i < markers.size(); i++) {
            // draw 2d info
            markers[i].draw(debugImage, cv::Scalar(0, 0, 255), 1);

            // draw a 3d cube
            aruco::CvDrawingUtils::draw3dCube(debugImage, markers[i], camParams);
            aruco::CvDrawingUtils::draw3dAxis(debugImage, markers[i], camParams);
        }
        */

        std::vector<MarkerPose> idMarkerPoses;
        {
            std::vector<MarkerFiducials> markers;
            for (auto &fiducial:msgIdFiducialDetection->fiducial) {
                MarkerFiducials marker(fiducial.ids, fiducial.ids_confidence);

                for (auto &object_point:fiducial.object_points)
                    marker.object_points.push_back(cv::Point3f(object_point.x, object_point.y, object_point.z));

                for (auto &image_point:fiducial.image_points)
                    marker.image_points.push_back(cv::Point2f(image_point.x, image_point.y));

                markers.push_back(marker);
            }

            // Do pose estimation
            PoseEstimationBase idPoseEstimation;
            idPoseEstimation.estimatePose(markers, camera_k, camera_d, idMarkerPoses);
        }
        for (auto &markerPose:idMarkerPoses) {
            ROS_INFO("marker[%d]", markerPose.ids[0]);


            std::vector<cv::Point3f> objectPoints;
            objectPoints.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
            objectPoints.push_back(cv::Point3f(0.055f, 0.0f, 0.0f));
            objectPoints.push_back(cv::Point3f(0.00f, 0.055f, 0.0f));
            objectPoints.push_back(cv::Point3f(-0.055f, 0.0f, 0.0f));
            objectPoints.push_back(cv::Point3f(0.00f, -0.055f, 0.0f));

            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(objectPoints, markerPose.getRVec(),  markerPose.getTVec(), camera_k, camera_d, imagePoints);
            // draw lines of different colours
            cv::line(debugImage, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::line(debugImage, imagePoints[0], imagePoints[2], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::line(debugImage, imagePoints[0], imagePoints[3], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::line(debugImage, imagePoints[0], imagePoints[4], cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            //putText(debugImage, "x", imagePoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
            //putText(debugImage, "y", imagePoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255), 2);
            //putText(debugImage, "z", imagePoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255), 2);

            int circleRadius = 5; // Pixel
            cv::circle(debugImage, imagePoints[1], circleRadius, cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::circle(debugImage, imagePoints[2], circleRadius, cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::circle(debugImage, imagePoints[3], circleRadius, cv::Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::circle(debugImage, imagePoints[4], circleRadius, cv::Scalar(0, 0, 255, 255), 1, CV_AA);



            /*
            cv::Mat objectPoints(1, 3, CV_32FC1);
            objectPoints.at<float>(0, 0) = 0;
            objectPoints.at<float>(0, 1) = 0;
            objectPoints.at<float>(0, 2) = 0;


            std::vector<cv::Point2f> imagePoints;
            cv::projectPoints(objectPoints, markerPose.getRVec(),  markerPose.getTVec(), camera_k, camera_d, imagePoints);


            putText(debugImage, "x", imagePoints[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
            */


            /*
            cv::projectPoints(objectPoints, Rvec,  Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
            // draw lines of different colours
            cv::line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), 1, CV_AA);
            cv::line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), 1, CV_AA);
            cv::line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), 1, CV_AA);
            putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
            markerPose.rt_matrix.
            */

        }

        for (auto &fiducial:msgEllipseFiducialDetection->fiducial) {
            cv::circle(debugImage, cv::Point2f(fiducial.image_points[0].x, fiducial.image_points[0].y), 5, cv::Scalar(0, 0, 0, 255), 1, CV_AA);
        }

        cv::imshow("combomarker_node_debug", debugImage);
        cv::waitKey(1);
    }
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

void PoseEstimationComboMarkerNode::publishMarkers(const std_msgs::Header &header, std::vector<MarkerPose> &markerPoses) {
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
        //if (base_.getParameters().getPublishTf())
            transformBroadcaster_.sendTransform(stf);

        // Push marker into MarkerDetection message
        marker_msgs::Marker marker;

        marker.ids = markerPose.ids;
        marker.ids_confidence = markerPose.ids_confidence;
        tf::poseTFToMsg(stf, marker.pose);

        msg.markers.push_back(marker);
    }

    // Publish MarkerDetection message
    //if (base_.getParameters().getPublishMarkers())
        pub_markers_.publish(msg);
}
