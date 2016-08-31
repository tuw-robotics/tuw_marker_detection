//
// Created by privacy on 31.08.16.
//

#ifndef TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_ESTIMATOR_H
#define TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_ESTIMATOR_H

#include "markermap/marker_map_config.h"
#include "opencv2/opencv.hpp"
#include "marker_fiducials.h"
#include "marker_pose.h"

class MarkerMapEstimator {
public:
    MarkerMapEstimator(MarkerMapDetails details);

    void estimatePose(std::vector<MarkerFiducials> &markers, cv::Mat &camera_k, cv::Mat &camera_d,
                      std::vector<MarkerPose> &markerPoses);

protected:
    MarkerMapDetails details_;
};

#endif //TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_ESTIMATOR_H
