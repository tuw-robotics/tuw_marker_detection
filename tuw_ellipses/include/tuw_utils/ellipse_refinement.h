/**
 * $Id$
 *
 * (C) 2014, Johann Prankl, Aitor Aldoma Buchaca
 */

#ifndef  TUW_ELLIPSE_REFINEMENT_H
#define  TUW_ELLIPSE_REFINEMENT_H

#include <vector>
#include <opencv2/core/core.hpp>

namespace tuw
{

class EllipseRefinement
{
public:


    class Parameter
    {
    public:
        int border;
        double min_gradient;
        Parameter(int _border=3, double _min_gradient=30)
            : border(_border), min_gradient(_min_gradient) {}
    };


    class Ellipse
    {
    private:

        double a2, b2, a4, b4;  // a^2 etc., stored for Distance()
        double x2,y2;
    public:
        int id;
        // ellipse position      [px]
        double x, y;
        // ellipse long/short axis length [px]
        double a, b;
        // ellipse rotation in radiants [rad]
        double phi;
        // mean fit error
        double fit_error;
        // ellipse support
        double support;

        /** Ellipse constructor
         **/
        Ellipse();
        /** Ellipse constructor
         **/
        Ellipse(const Ellipse &e);
        
        /** Ellipse constructor
         * @param _x ellipse center point x [px]
         * @param _y ellipse center point y [px]
         * @param _a ellipse major semiaxes [px]
         * @param _a ellipse minor semiaxes [px]
         * @param _phi ellipse rotation [rad]
         **/
        void setEllipse(const double &_x, const double &_y, const double &_a, const double &_b, const double &_phi);

        /**
        * Sets the ellipse parameter to get ready for detection based on an opencv rectancle
        * @rect opencv rotated rect
        */
        void setEllipse(const cv::RotatedRect& rect);

        /**
         * returns an opencv rotated rect
         * @r des of the type transform
         **/
        void get(cv::RotatedRect &r);

        /**
        * Approximation to ellipse circumference.
        * (from Bartsch: Mathematische Formeln, Buch- und Zeit-Verlagsgesellschaft Koeln, 1988, p. 221)
        * @return circumference
        */
        double ellipseCircumference(double a, double b);
        
        /**
        * @return true if the position x/y is inside the given ellipse using the ellipse equation
        */
        bool insideEllipse(double _a, double _b, double _x0, double _y0, double _phi, double _x, double _y) const;

        unsigned ellipseSupport(const std::vector<cv::Point2f> &points, double inl_dist, std::vector<bool> &inl_idx);
        
        bool computeAndSetGeomFromConic(double A, double B, double C, double D, double E, double F);

    };
private:

    std::vector<cv::Point2d> pointsToUse;

public:
    Parameter param;

    EllipseRefinement(const Parameter &_param = Parameter());
    ~EllipseRefinement();

    /** Calculate exact ellipse parameters of the given ellipse using gradient images
     * @param im_dx image gradients x ( e.g. cv::Sobel( im_gray, im_dx, CV_16S, 1, 0, 3, 1, 0) )
     * @param im_dy image gradients y ( e.g. cv::Sobel( im_gray, im_dx, CV_16S, 0, 1, 3, 1, 0) )
     * @param points contour edgels
     * @param ellipse to be refined 
     **/
    bool refine(const cv::Mat_<short> &im_dx, const cv::Mat_<short> &im_dy, const std::vector<cv::Point2f> &points, Ellipse &ellipse);
};

}

#endif //TUW_ELLIPSE_REFINEMENT_H