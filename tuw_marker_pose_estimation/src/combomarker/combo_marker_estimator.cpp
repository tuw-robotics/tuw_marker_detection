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

#include "combomarker/combo_marker_estimator.h"
#include "pose_estimation_base.h"

ComboMarkerEstimator::ComboMarkerEstimator() {

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

static double max_search_pixel_radius = 10;

void
ComboMarkerEstimator::estimatePose(std::vector <MarkerFiducials> &idFiducials, cv::Mat &camera_k, cv::Mat &camera_d,
                                 std::vector <MarkerFiducials> &ellipseFiducials,
                                 std::vector <MarkerPose> &markerPoses) {

    for (auto &fiducial:idFiducials) {
        std::cout << "FiducialId - i: " << std::endl << fiducial.image_points << std::endl << " o: " << std::endl
                  << fiducial.object_points << std::endl;
    }

    std::vector<MarkerPose> idMarkerPoses;
    PoseEstimationBase idPoseEstimation;
    idPoseEstimation.estimatePose(idFiducials, camera_k, camera_d, idMarkerPoses);

    // FIXME: Provide this via config file
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(0.055f, 0.0f, 0.0f));
    objectPoints.push_back(cv::Point3f(0.00f, 0.055f, 0.0f));
    objectPoints.push_back(cv::Point3f(-0.055f, 0.0f, 0.0f));
    objectPoints.push_back(cv::Point3f(0.00f, -0.055f, 0.0f));

    /*
    objectPoints.push_back(cv::Point3f(0.08f, 0.0f, 0.0f));
    objectPoints.push_back(cv::Point3f(0.00f, 0.06f, 0.0f));
    objectPoints.push_back(cv::Point3f(-0.08f, 0.0f, 0.0f));
    */


    std::vector<MarkerFiducials> combinedFiucials;

    for (auto &markerPose:idMarkerPoses) {
        MarkerFiducials marker(markerPose.ids, markerPose.ids_confidence);

        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(objectPoints, markerPose.getRVec(), markerPose.getTVec(), camera_k, camera_d, imagePoints);

        for (int i = 0; i < objectPoints.size(); i++) {
            cv::Point2f ellipseCenter = cv::Point2f(-1.0f, -1.0f);

            // Iterate trough all ellipses found and try to match the closest
            cv::Point2f searchCenter = imagePoints[i];
            for (auto &ellipseFiducial:ellipseFiducials) {
                cv::Point2f c = ellipseFiducial.image_points[0];
                double dist = cv::norm(searchCenter - c);
                if (dist < max_search_pixel_radius) {
                    std::cout << dist << std::endl;
                    ellipseCenter = c; // FIXME: This is just using the first point found in range. No match closest is done yet.
                }
            }

            // Check if a valid candidate was found
            if (ellipseCenter.x >= 0 && ellipseCenter.y >= 0) {
                marker.object_points.push_back(objectPoints[i]);
                marker.image_points.push_back(ellipseCenter);
            }
        }

        // Check if there is a minimum of 3 image points
        if (marker.image_points.size() > 3) {
            combinedFiucials.push_back(marker);
        }
    }

    for (auto &fiducial:combinedFiucials) {
        std::cout << "FiducialCombined - i: " << std::endl << fiducial.image_points << std::endl << " o: " << std::endl
                  << fiducial.object_points << std::endl;
    }

    PoseEstimationBase combinedPoseEstimation;
    combinedPoseEstimation.estimatePose(combinedFiucials, camera_k, camera_d, markerPoses);

    /*
    for (auto &fiducial:combinedFiucials) {
        // Estimate MarkerMap pose
        if (fiducial.image_points.size() > 2 && fiducial.object_points.size() > 2) {
            MarkerPose pose(fiducial.ids, fiducial.ids_confidence);
            {
                cv::Mat rv, tv;
                cv::solvePnP(fiducial.object_points, fiducial.image_points, camera_k, camera_d, rv, tv, cv::SOLVEPNP_DLS);
                //cv::solvePnPRansac(fiducial.object_points, fiducial.image_points, camera_k, camera_d, rv, tv, cv::SOLVEPNP_P3P);

                cv::Mat rvec, tvec;
                rv.convertTo(rvec, CV_32F);
                tv.convertTo(tvec, CV_32F);

                pose.rt_matrix = getRTMatrix(rvec, tvec);
            }
            if (!pose.rt_matrix.empty())
                markerPoses.push_back(pose);
        }
    }
    */
}