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

#include "tuw_aruco/aruco_base.h"

ArUcoBase::ArUcoBase() : params_() {
    refreshParameters();
}

ArUcoBase::~ArUcoBase() { }

void ArUcoBase::detectMarkers(vector<aruco::Marker> &markers, cv::Mat image) {
    detector_.detect(image, markers);
}

void ArUcoBase::estimatePose(vector<ArUcoMarkerPose> &markerPoses, vector<aruco::Marker> &markers, aruco::CameraParameters cameraParams) {
    markerPoses.clear();
    for (auto &marker:markers) {
        bool success = tracker_[marker.id].estimatePose(marker, cameraParams, params_.getMarkerSize(), 1.0f);
        if(success){
            cv::Mat rtMatrix = tracker_[marker.id].getRTMatrix();
            markerPoses.push_back(ArUcoMarkerPose(marker.id, rtMatrix));
        }
    }
}

ArUcoParameters &ArUcoBase::getParameters(){
    return params_;
}

void ArUcoBase::refreshParameters() {
    detector_.setDictionary(params_.getDictionary());
    detector_.setThresholdParams(7, 7);
    detector_.setThresholdParamRange(2, 0);
}
