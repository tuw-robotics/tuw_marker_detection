/**
 * @file camera
 * @author Markus Bader
 * @date March 2014
 * @version 0.1
 * @brief
 *
 * @see
 **/


#include "tuw_utils/camera.h"

namespace tuw {
  
  
void Camera::distort(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &des) {
    const double &srcFx = projectionMatrix(0,0);
    const double &srcFy = projectionMatrix(1,1);
    const double &srcCx = projectionMatrix(0,2);
    const double &srcCy = projectionMatrix(1,2);
    const double &k1 = distCoeffs(0,0);
    const double &k2 = distCoeffs(0,1);
    const double &p1 = distCoeffs(0,2);
    const double &p2 = distCoeffs(0,3);
    const double &k3 = distCoeffs(0,4);
    const double &desFx = cameraMatrix(0,0);
    const double &desFy = cameraMatrix(1,1);
    const double &desCx = cameraMatrix(0,2);
    const double &desCy = cameraMatrix(1,2);
    des.resize(src.size());
    double x, y, x1, y1, r2, r4, r6, a1, a2, a3, cdist;
    for(unsigned int i = 0; i < src.size(); i++) {
 
        x = (src[i].x - srcCx) / srcFx;
        y = (src[i].y - srcCy) / srcFy;    
        r2 = x*x + y*y;
        r4 = r2*r2;     
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1+k1*r2+k2*r4+k3*r6;
        x1 = (x*cdist + p1*a1 + p2*a2);
        y1 = (y*cdist + p1*a3 + p2*a1);

        des[i].x = x1 * desFx + desCx;
        des[i].y = y1 * desFy + desCy;
    }
}

void Camera::distort(const cv::RotatedRect &src, cv::RotatedRect &des) {

    std::vector<cv::Point2f> vtx(5);
    src.points(&vtx[0]);
    vtx[4] = src.center;
    std::vector<cv::Point2f> vtxDistort;
    distort(vtx, vtxDistort);
    des.size = cv::Size2f((cv::norm(vtx[2] - vtx[1])+cv::norm(vtx[0] - vtx[3]))/2., (cv::norm(vtx[1] - vtx[0])+cv::norm(vtx[3] - vtx[2]))/2.);
    des.angle = atan2(vtx[2].y - vtx[1].y, vtx[2].x - vtx[1].x) * 180.0/M_PI;
    des.center = vtxDistort[4];

}


}