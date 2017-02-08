/***************************************************************************
 * Copyright (c) 2014 Markus Bader <markus.bader@tuwien.ac.at>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the TU-Wien.
 * 4. Neither the name of the TU-Wien nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY Markus Bader ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************/

#ifndef TUW_CHECKERBOARD_H
#define TUW_CHECKERBOARD_H

#include <opencv/cv.hpp>


namespace tuw {
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
#endif //TUW_CHECKERBOARD_H
