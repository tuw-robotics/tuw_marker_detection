/***************************************************************************
 *   Copyright (C) 2014 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tuw_ellipses/ellipses_detection.h>
#include <tuw_ellipses/ellipses_defaults.h>
#include <boost/foreach.hpp>

using namespace tuw;

void EllipsesDetection::draw_ellipses(cv::Mat &img) {
    std::vector<cv::Point2f> vtx(4);
    std::vector<cv::Point2f> vtxDis;
    cv::Scalar colourContour(255,0,255);
    cv::Scalar colourEllipse(0,0,255);
    char text[0xFF];
    for(unsigned int i = 0; i < ellipses_.size(); i++) {
        Ellipse &ellipse = ellipses_[i];
        if(ellipse.detection != VALID) continue;
        //cv::drawContours(img, contours_, (int) i, colourContour, 1, 8);
        cv::RotatedRect box1, box2;
        camera_.distort(ellipse.boxEllipse, box1);
        cv::ellipse(img, box1, colourEllipse, 1, CV_AA);
        camera_.distort(ellipses_[ellipse.innerRing].boxEllipse, box2);
        if(ellipse.innerRing >= 0) cv::ellipse(img, box2, colourEllipse, 1, CV_AA);
        box1.points(&vtx[0]);
        for( int j = 0; j < 4; j++ ) {
            cv::line(img, vtx[j], vtx[(j+2)%4], cv::Scalar(0,255,0), 1, CV_AA);
            sprintf(text, "%i", j);
            //cv::putText(img, text, vtx[j], cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0), 1, CV_AA);
        }
        camera_.distort(*ellipse.contourUndistort, vtxDis);
        BOOST_FOREACH(const cv::Point2f &p, vtxDis ) {
            img.at<cv::Vec3b>(p) = cv::Vec3b(0,255,255);
        }

        // draw normals into the image
        if(ellipse.cone.projections[0] != cv::Point2d(0,0)) {
            for(unsigned int k = 0; k < 2; k++) {
                cv::Mat pi, pw(ellipse.cone.translations[k]);
                cv::Mat pin , pn = pw + cv::Mat(ellipse.cone.normals[k])*param_->circle_diameter;
                cv::projectPoints(pw.t(), cv::Mat_<double>::zeros(1,3), cv::Mat_<double>::zeros(1,3), camera_.cameraMatrix, camera_.distCoeffs, pi);
                cv::projectPoints(pn.t(), cv::Mat_<double>::zeros(1,3), cv::Mat_<double>::zeros(1,3), camera_.cameraMatrix, camera_.distCoeffs, pin);
                for(int j = 0; j < pi.rows; j++) {
                    cv::Point p0(pi.at<double>(j,0),pi.at<double>(j,1));
                    cv::circle(img, p0,2, cv::Scalar(125*j,125*j,255));
                    cv::Point p1(pin.at<double>(j,0),pin.at<double>(j,1));
                    cv::line(img, p0, p1, cv::Scalar(125*j,125*j,255), 1, CV_AA);
                }
            }
        }

        /// Matlab Debug output
        if(param_->debug) {
            std::cout << "% === " << std::setw(4) <<  loop_count;
            std::cout << "  ===== ellipse " << std::setw(4) << i << " ===" << std::endl;
            std::cout << "ellipse.center     = " <<  ellipse.boxEllipse.center << "; " << std::endl;
            std::cout << "ellipse.size       = " <<  (cv::Point2f) ellipse.boxEllipse.size << "; " << std::endl;
            std::cout << "ellipse.angle      = " <<   M_PI/180.0 *ellipse.boxEllipse.angle << "; " << std::endl;
            std::cout << "ellipse.C          = " << ellipse.cone.C << "; " << "  % Ellipse Image" << std::endl;
            std::cout << "ellipse.radius     = " << param_->circle_diameter/2. << "; " << std::endl;
            std::cout << "ellipse.nr_of_edges=" <<  ellipse.contourUndistort->size() << "; " << std::endl;
            std::cout << "ellipse.contour    = " << cv::Mat(*ellipse.contourUndistort) << "; " << std::endl;
            std::cout << "camera.intrinsic   = " << camera_.cameraMatrix << "; " << std::endl;
            std::cout << "camera.distortions = " << camera_.distCoeffs << "; " << std::endl;
            std::cout << "camera.projectionMatrix = " << camera_.projectionMatrix << "; " << std::endl;
        }
    }
    if(param_->debug) {
        contour_detector_.Draw(img.data);
    }
}
