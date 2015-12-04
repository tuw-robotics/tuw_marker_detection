#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "tuw_utils/canny.h"

void help() {
    std::cout << "\nThis is a demo that shows an anhanced canny edge detection\n"
              "Usage: \n"
              "	./canny file # for a single image\n"
              "	./canny file # for camera usage\n";
}

int main ( int argc, char *argv[] ) {

    cv::VideoCapture cap;
    cv::Mat imgSrc;
    cv::Mat imgGray;
    if ( argc != 2 ) {
        cap.open ( 0 );
        if ( !cap.isOpened() ) {
            std::cout << "***Could not initialize capturing...***\n";
            help();
            return -1;
        }
        cap.set ( CV_CAP_PROP_FRAME_WIDTH, 800 );
        cap.set ( CV_CAP_PROP_FRAME_HEIGHT, 600 );
        std::cout << "Video " <<
                  ": width=" << cap.get ( CV_CAP_PROP_FRAME_WIDTH ) <<
                  ", height=" << cap.get ( CV_CAP_PROP_FRAME_HEIGHT ) <<
                  ", nframes=" << cap.get ( CV_CAP_PROP_FRAME_COUNT ) << std::endl;
    }
    if ( !cap.isOpened() ) {
        printf ( "%s\n", argv[1] );
        imgSrc = cv::imread ( argv[1] );
    } else {
        cap >> imgSrc;
    }
    if ( imgSrc.empty() ) {
        std::cout << "***image...***\n";
        return -1;
    }
    cv::cvtColor ( imgSrc, imgGray, CV_RGB2GRAY );

    cv::Mat imgGauss, imgCanny, imgGradient, imgDirection;
    cv::namedWindow ( "src",1 );
    cv::namedWindow ( "img",1 );
    cv::namedWindow ( "canny",1 );
    cv::namedWindow ( "gradient",1 );
    cv::namedWindow ( "direction",1 );
    do {
        if ( cap.isOpened() ) {
            cap >> imgSrc;
            cv::cvtColor ( imgSrc, imgGray, CV_RGB2GRAY );
        }
        if ( imgSrc.empty() ) {
            continue;
        }
        cv::GaussianBlur ( imgGray, imgGauss, cv::Size ( 7,7 ), 1.5, 1.5 );
        V4R::Canny ( imgGauss, imgCanny, imgGradient, imgDirection, 0, 30, 3 );
        cv::imshow ( "src", imgGray );
        cv::imshow ( "img", imgGray );
        cv::imshow ( "canny", imgCanny );
        cv::imshow ( "gradient", ( imgGradient*65335/360 ) );
        cv::imshow ( "direction", ( imgDirection*65335/360 ) );
    } while ( cv::waitKey ( 30 ) < 0 ) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

