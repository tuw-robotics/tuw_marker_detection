/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader               *
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


#ifndef V4R_CHECKERBOARD_POSE_H
#define V4R_CHECKERBOARD_POSE_H

#include <opencv/cv.h>


namespace V4R {
/**
* @class Checkerboard
* @author Markus Bader
**/
class Checkerboard
{
public:
    /**
     * Constructor
     **/
    Checkerboard() {};
    /**
     * init
     * @param patternSize number of inner boxes of the checkerboard
     * @param boxSize size of a single checkerboard
     * @param useSubPix on true it uses sub pixel to estimate the checkerboard corners see cvFindCornerSubPix
     **/
    void init(cv::Size patternSize, cv::Size_<double> boxSize, bool useSubPix = true);
    /**
     * find the checkerboard pose in cv image
     * @param img image
     * @param cameraMatrix intrinsic matrix for the format have a look on cv::solvePnP
     * @param distCoeffs distCoeffs for the format have a look on cv::solvePnP
     * @param R estiamted roation vector to the board
     * @param T estiamted translation vector to the board
     * @return true of board was found
     * @pre init
     **/
    int find(const cv::Mat &img, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Mat &R, cv::Mat &T);
    /**
     * draws a detected board in the image using cv::drawChessboardCorners
     * @param img
     **/
    void drawBoard(cv::Mat &img);
    /**
     * draws a coordinate system into the image
     * @param img
     * @param cameraMatrix intrinsic matrix 3x3, 3x4 or 4x4
     * @param distCoeffs distCoeffs matrix 4x1, 5x1, 1x4 or 1x5
     * @param rvec roation vector to the board
     * @param tvec estiamted translation vector to the board
     **/
    void drawSystem(cv::Mat &img, const cv::Mat_<double>& cameraMatrix, const cv::Mat_<double>& distCoeffs, const cv::Mat_<double> &rvec, const cv::Mat_<double> &tvec);

private:
    cv::Size mPatternSize;
    cv::Size_<double> mBoxSize;
    cv::Mat_<double> mCameraMatrix;
    cv::Mat_<double> mDistCoeffs;
    cv::Mat_<double> mRVec;
    cv::Mat_<double> mTVec;
    std::vector<cv::Point2f> mImagePoints;
    std::vector<cv::Point3f> mObjectPoints;
    int mDetectionOK;
    bool mUseSubPix;
    
};
}
#endif //V4R_CHECKERBOARD_POSE_H
