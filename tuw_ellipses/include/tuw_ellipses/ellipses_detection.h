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


#include <iostream>
#include <list>

#ifndef TUW_ELLIPSES_H
#define TUW_ELLIPSES_H

#include "opencv2/core/core.hpp"
#include "tuw_utils/contour.h"
#include "tuw_utils/camera.h"
#include "boost/shared_ptr.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>

namespace tuw {

class EllipsesDetection {
public:
    enum DetectionState {
        VALID = 0,
        NA = 1,
        INVALID_CONTOUR_POINTS,
        INVALID_CONTOUR_CONVEX,
        INVALID_ROTATED_RECT_RATIO,
        INVALID_CONTOUR_MEAN,
        INVALID_MIN_RAIDUS,
        INVALID_MAX_RAIDUS,
        INVALID_NO_RING,
        INVALID_IS_INNER_RING,
        INVALID_ELLIPSE
    };
    enum PoseEstimation {
        POSE_ESTIMATION_OFF = 0,
        POSE_ESTIMATION_SOLVEPNP = 1,
        POSE_ESTIMATION_FROM_ELLIPSE = 2,
    };
    enum EdgeDetection {
        EDGE_DETECTION_THRESHOLD = 0,
        EDGE_DETECTION_CANNY = 1
    };
    enum EdgeLinking {
        EDGE_LINKING_OPENCV_APPROX_NONE = 0,
        EDGE_LINKING_OPENCV_APPROX_SIMPLE = 1,
        EDGE_LINKING_TUW_SIMPLE = 2,
        EDGE_LINKING_TUW_COMPLEX = 3,
        EDGE_LINKING_TUW_CONTOUR = 4
    };
    struct Parameters {
        Parameters();
        bool debug;
        bool distorted_input;
        EdgeDetection edge_detection;
        int threshold_edge_detection1;
        int threshold_edge_detection2;
        int kernel_size_edge_detection;
        EdgeLinking edge_linking;
        int threshold_contour_min_points;
        int threshold_polygon;
        bool filter_convex;
        double threshold_rotated_rect_ratio;
        bool ellipse_redefinement;
        bool filter_contour_mean;
        double threshold_contour_mean;
        double threshold_min_radius;
        double threshold_max_radius;
        bool filter_rings;
        double threshold_ring_center;
        double threshold_ring_ratio;
        PoseEstimation pose_estimation;
        double circle_diameter;
    };
    class ObliqueCone {
    public:
        ObliqueCone() {};
        ObliqueCone(const ObliqueCone& c) {
            set(c);
        };
        cv::Mat_<double> C;  /// ellipse equation
        cv::Point3d translations[2]; /// two plausible translations solutions
        cv::Vec3d normals[2];/// two plausible translations solutions
        cv::Point2d projections[2]; /// two plausible translations solutions
        cv::Mat_<double> R[2]; /// two plausible roations
        void set(const ObliqueCone& c) {
            C = c.C.clone();
            for(unsigned int i = 0; i < 2; i++) {
                translations[i] = c.translations[i], normals[i] = c.normals[i], projections[i] = c.projections[i];
                R[i] = c.R[i].clone();
            }
        }
        void pose(cv::Mat_<double> intrinsic, cv::Mat_<double> distCoeffs, double radius);
        void set(cv::RotatedRect box, cv::Mat_<double> intrinsic);
        double distance(const cv::Point2d p);
        void rotation2Normal(int i);
        void normal2Roation(int i);
    };
    class Ellipse {
    public:
        Ellipse(): id(-1), outerRing(-1), innerRing(-1), boxContour(), boxEllipse(), detection(VALID) {
        };
        void init() {
            contourUndistort = boost::shared_ptr<std::vector<cv::Point2f> > (new std::vector<cv::Point2f>);
            contourDistort = boost::shared_ptr<std::vector<cv::Point2f> > (new std::vector<cv::Point2f>);
            polygon = boost::shared_ptr<std::vector<cv::Point> > (new std::vector<cv::Point>);
            distances = boost::shared_ptr<std::vector<double> > (new std::vector<double>);
        }
        int id;
        int outerRing;
        int innerRing;
        DetectionState detection;
        cv::RotatedRect boxEllipse;
        cv::Rect boxContour;
        cv::Point2d centerContour;
        double radiusContour;
        double boxEllipseRatio;
        double radiusEllipseMax;
        double radiusEllipseMin;
        ObliqueCone cone;
        boost::shared_ptr<std::vector<cv::Point2f> > contourUndistort;
        boost::shared_ptr<std::vector<cv::Point2f> > contourDistort;
        boost::shared_ptr<std::vector<cv::Point> > polygon;
        boost::shared_ptr<std::vector<double> > distances;
    };
    class Marker : public ObliqueCone {
    public:
        Marker() {};
        Marker(const ObliqueCone &c)
            : ObliqueCone(c), id(-1), tstamp(), A(cv::Mat_<double>::eye(4,4)) {
        }
        Marker(const Marker &c)
            : ObliqueCone(c), id(c.id), tstamp(c.tstamp), A(c.A.clone()) {
        }
        int id;    
        boost::posix_time::ptime tstamp;
        cv::Mat_<double> A;
        void update(const boost::posix_time::ptime &t);
    };
    EllipsesDetection (Parameters *parm);
    ~EllipsesDetection();
protected:
    Parameters *param_;
    void fit_ellipses_opencv (const cv::Mat &m, const cv::Mat cameraMatrix, const cv::Mat distCoeffs, const cv::Mat projectionMatrix, const boost::posix_time::ptime &tstamp);
    void edge_detection(const cv::Mat &m);
    void contour_detection();
    void draw_ellipses(cv::Mat &m);
    void next();
    void createEllipseCanditates () ;
    DetectionState filterContour (Ellipse &ellipse);
    DetectionState filterEllipse (Ellipse &ellipse);
    DetectionState filterContourMean(Ellipse &ellipse);    
    DetectionState EllipseRedefinement(Ellipse &ellipse);
    void estimatePoses();
    void createRings();
    void filterShapes();
    std::vector< std::vector<cv::Point> > contours_;
    std::vector<Ellipse> ellipses_;
    cv::Mat imgGray_;
    cv::Mat imgBlured_;
    cv::Mat imgEdges_;
    cv::Mat imgGradient_;
    cv::Mat imgDirection_;
    cv::Mat imgSobelDx_;
    cv::Mat imgSobelDy_;
    cv::Mat_<cv::Point2f>  lookupUndistor_;
    std::list<Marker> markers_;
    unsigned long loop_count;
    tuw::Contour contour_detector_;
    tuw::Camera camera_;
    boost::posix_time::ptime tstamp_;
    boost::posix_time::ptime tstampLast_;
};
};
#endif // TUW_ELLIPSES_H
