#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "tuw_utils/contour.h"



int main(int argc, char *argv[]) {
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );

    cv::Mat imgGray = cv::imread(argv[1], 0);
    cv::Mat imgGauss, imgCanny, imgLinkedEdges(imgGray.rows, imgGray.cols, CV_8UC3);
    cv::namedWindow("img",1);
    cv::namedWindow("canny",1);
    cv::namedWindow("linked_edges",1);
    V4R::Contour el;
    el.Init(imgGray.cols, imgGray.rows);


    do {
        cv::GaussianBlur(imgGray, imgGauss, cv::Size(7,7), 1.5, 1.5);
        cv::Canny(imgGauss, imgCanny, 0, 30, 3);
        el.Perform(imgCanny.data, V4R::Contour::MODE_SIMPLE);
        el.Draw(imgLinkedEdges.data);
        cv::imshow("img", imgGray);
        cv::imshow("canny", imgCanny);
        cv::imshow("linked_edges", imgLinkedEdges);
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

