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

#ifndef TUW_ARUCO_ARUCO_PARAMETERS_H
#define TUW_ARUCO_ARUCO_PARAMETERS_H

#include "aruco.h"

class ArUcoParameters {
public:
    ArUcoParameters();

    ~ArUcoParameters();

    void setDictionary(std::string dictionary);
    std::string getDictionary();
    bool getPublishTf();
    bool getPublishMarkers();
    bool getPublishFiducials();
    bool getPoseEstimationEnabled();
    bool getShowDebugImage();

    void setMarkerSize(float mSize);
    float getMarkerSize();
    void setPublishTf(bool b);
    void setPublishMarkers(bool b);
    void setPublishFiducials(bool b);
    void setPoseEstimationEnabled(bool b);
    void setShowDebugImage(bool b);

private:
    std::string dictionary_;
    float markerSize_;
    bool publish_tf_;
    bool publish_markers_;
    bool publish_fiducials_;
    bool pose_estimation_enabled_;
    bool show_debug_image_;

};


#endif //TUW_ARUCO_ARUCO_PARAMETERS_H
