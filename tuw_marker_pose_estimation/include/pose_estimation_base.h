//
// Created by privacy on 25.08.16.
//

#ifndef TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_BASE_H
#define TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_BASE_H

#include "pose_estimation_parameters.h"
#include "opencv2/opencv.hpp"

class PoseEstimationBase {
public:
    PoseEstimationBase();

    ~PoseEstimationBase();

    /*
    void detectMarkers(vector<aruco::Marker> &markers, cv::Mat image);
    void estimatePose(vector<ArUcoMarkerPose> &markerPoses, vector<aruco::Marker> &markers, aruco::CameraParameters cameraParams);
     */

    PoseEstimationParameters &getParameters();
    void refreshParameters();

private:
    PoseEstimationParameters params_;

    /*
    aruco::MarkerDetector detector_;
    std::map <uint32_t, aruco::MarkerPoseTracker> tracker_;
    */
};

#endif //TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_BASE_H
