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

#include "markermap/pose_estimation_markermap_base.h"

#include "ros/ros.h"

PoseEstimationMarkerMapBase::PoseEstimationMarkerMapBase(MarkerMapConfig markerMapConfig)
        : params_(), markerMapConfig_(markerMapConfig) {
    refreshParameters();

    estimators_.clear();
    for (auto &markerMap:markerMapConfig_.markerMaps) {
        estimators_.push_back(MarkerMapEstimator(markerMap));
    }
}

PoseEstimationMarkerMapBase::~PoseEstimationMarkerMapBase() {}

static cv::Mat getRTMatrix(const cv::Mat &_rvec, const cv::Mat &_tvec) {
    if (_rvec.empty())
        return cv::Mat();
    cv::Mat m = cv::Mat::eye(4, 4, CV_32FC1);
    cv::Mat R33 = cv::Mat(m, cv::Rect(0, 0, 3, 3));
    cv::Rodrigues(_rvec, R33);
    for (int i = 0; i < 3; i++)
        m.at<float>(i, 3) = _tvec.ptr<float>(0)[i];
    return m;
}

void PoseEstimationMarkerMapBase::estimatePose(std::vector<MarkerFiducials> &markerFiducials,
                                               cv::Mat &camera_k, cv::Mat &camera_d,
                                               std::vector<MarkerPose> &markerPoses) {
    for (auto &estimator:estimators_) {
        estimator.estimatePose(markerFiducials, camera_k, camera_d, markerPoses);
    }
}

PoseEstimationMarkerMapParameters &PoseEstimationMarkerMapBase::getParameters() {
    return params_;
}

void PoseEstimationMarkerMapBase::refreshParameters() {

}
