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

#include "tuw_aruco/aruco_parameters.h"

ArUcoParameters::ArUcoParameters() {
    dictionary_ = "ARTOOLKITPLUSBCH";
    markerSize_ = 0.06f;
    publish_tf_ = true;
    publish_markers_ = true;
    show_debug_image_ = true;
    publish_fiducials_ = false;
    pose_estimation_enabled_ = true;
}

ArUcoParameters::~ArUcoParameters() {}

std::string ArUcoParameters::getDictionary() {
    return dictionary_;
}

float ArUcoParameters::getMarkerSize() {
    return markerSize_;
}

bool ArUcoParameters::getPublishTf() {
    return publish_tf_;
}

bool ArUcoParameters::getPublishMarkers() {
    return publish_markers_;
}

bool ArUcoParameters::getPublishFiducials() {
    return publish_fiducials_;
}

bool ArUcoParameters::getPoseEstimationEnabled() {
    return pose_estimation_enabled_;
}

bool ArUcoParameters::getShowDebugImage() {
    return show_debug_image_;
}

void ArUcoParameters::setDictionary(std::string dictionary) {
    dictionary_ = dictionary;
}

void ArUcoParameters::setMarkerSize(float mSize) {
    markerSize_ = mSize;
}

void ArUcoParameters::setPublishTf(bool b) {
    publish_tf_ = b;
}

void ArUcoParameters::setPublishMarkers(bool b) {
    publish_markers_ = b;
}

void ArUcoParameters::setPublishFiducials(bool b) {
    publish_fiducials_ = b;
}

void ArUcoParameters::setPoseEstimationEnabled(bool b) {
    pose_estimation_enabled_ = b;
}

void ArUcoParameters::setShowDebugImage(bool b) {
    show_debug_image_ = b;
}