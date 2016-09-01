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

#include "markermap/marker_map_estimator.h"

#include "ros/ros.h"

MarkerMapEstimator::MarkerMapEstimator(MarkerMapDetails details) : details_(details) {

}

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

void
MarkerMapEstimator::estimatePose(std::vector<MarkerFiducials> &markerFiducials, cv::Mat &camera_k, cv::Mat &camera_d,
                                 std::vector<MarkerPose> &markerPosesOutput) {
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    std::vector<MarkerFiducials>::iterator it = markerFiducials.begin();
    while (it != markerFiducials.end()) {
        MarkerFiducials &fiducial = *it;

        // Only try to match if marker id is known
        if (fiducial.ids.size() > 0) {
            int id = fiducial.ids[0];

            for (auto &markerDetails:details_.markers) {

                // Only match ids for now, no type check is done
                if (id == markerDetails.id) {

                    // Just add these object/image points
                    for (cv::Point3f &object_point:fiducial.object_points) {
                        object_point = object_point + cv::Point3f(markerDetails.position);
                        // FIXME: No rotation is done yet
                        // ROS_INFO("x=%f, y=%f, z=%f", object_point.x, object_point.y, object_point.z);
                        object_points.push_back(object_point);
                    }

                    for (auto &image_point:fiducial.image_points)
                        image_points.push_back(image_point);
                }

            }

            // Remove fiducial element from list to avoid multiple use in another estimator
            // Next element follows
            it = markerFiducials.erase(it);
        } else {
            std::next(it);
        }
    }

    // Estimate MarkerMap pose
    if (image_points.size() > 2 && object_points.size() > 2) {
        MarkerPose pose({details_.id}, {1});
        {
            cv::Mat rv, tv;
            cv::solvePnP(object_points, image_points, camera_k, camera_d, rv, tv);

            cv::Mat rvec, tvec;
            rv.convertTo(rvec, CV_32F);
            tv.convertTo(tvec, CV_32F);

            pose.rt_matrix = getRTMatrix(rvec, tvec);
        }
        if (!pose.rt_matrix.empty())
            markerPosesOutput.push_back(pose);
    }

}
