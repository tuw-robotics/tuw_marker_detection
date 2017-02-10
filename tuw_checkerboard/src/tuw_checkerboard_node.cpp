/***************************************************************************
 * Copyright (c) 2017 
 * Florian Beck <florian.beck@tuwien.ac.at>
 * Markus Bader <markus.bader@tuwien.ac.at>
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

#include "tuw_checkerboard_node.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using std::vector;
using std::string;

CheckerboardNode::CheckerboardNode() : nh_private_ ( "~" ) {
    image_transport::ImageTransport it_ ( nh_ );

    // Advert checkerboard pose publisher
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped> ( "pose", 1 );
    // Advert marker publisher
    pub_markers_ = nh_.advertise<marker_msgs::MarkerDetection> ( "markers", 10 );
    // Advert fiducial publisher
    pub_fiducials_ = nh_.advertise<marker_msgs::FiducialDetection> ( "fiducials", 10 );

    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    nh_.param<std::string> ( "frame_id", checkerboard_frame_id_, "checkerboard" );

    reconfigureServer_ = new dynamic_reconfigure::Server<tuw_checkerboard::CheckerboardDetectionConfig> ( ros::NodeHandle ( "~" ) );
    reconfigureFnc_ = boost::bind ( &CheckerboardNode::callbackConfig, this,  _1, _2 );
    reconfigureServer_->setCallback ( reconfigureFnc_ );

    sub_cam_ = it_.subscribeCamera ( "image", 1, &CheckerboardNode::callbackCamera, this );

}

void CheckerboardNode::callbackConfig ( tuw_checkerboard::CheckerboardDetectionConfig &_config, uint32_t _level ) {
    config_ = _config;

    object_corners_.clear();
    for ( int i = 0; i < config_.checkerboard_rows; i++ ) {
        for ( int j = 0; j < config_.checkerboard_columns; j++ ) {
            object_corners_.push_back ( Point3f ( float ( i * config_.checkerboard_square_size ), float ( j * config_.checkerboard_square_size ), 0.f ) );
        }
    }
}
/*
 * Camera callback
 * detects chessboard pattern using opencv and finds camera to image tf using solvePnP
 */
void CheckerboardNode::callbackCamera ( const sensor_msgs::ImageConstPtr& image_msg,
                                        const sensor_msgs::CameraInfoConstPtr& info_msg ) {

    Size patternsize ( config_.checkerboard_columns, config_.checkerboard_rows );
    cv_bridge::CvImagePtr input_bridge;
    try {
        input_bridge = cv_bridge::toCvCopy ( image_msg, sensor_msgs::image_encodings::MONO8 );
        if(config_.rotate_camera_image_180) {
            cv::flip(input_bridge->image, image_grey_, -1);
        } else {
            image_grey_ = input_bridge->image;
        }
        cvtColor ( image_grey_, image_rgb_, CV_GRAY2BGR, 0 );

    } catch ( cv_bridge::Exception& ex ) {
        ROS_ERROR ( "[camera_tf_node] Failed to convert image" );
        return;
    }



    int flags = 0;
    if ( config_.adaptive_thresh ) flags += CV_CALIB_CB_ADAPTIVE_THRESH;
    if ( config_.normalize_image ) flags += CV_CALIB_CB_NORMALIZE_IMAGE;
    if ( config_.filter_quads ) flags += CV_CALIB_CB_FILTER_QUADS;
    if ( config_.fast_check ) flags += CALIB_CB_FAST_CHECK;
    bool patternfound = findChessboardCorners ( image_grey_, patternsize, image_corners_, flags );

    if ( patternfound ) {
        if ( config_.subpixelfit ) {

            int winSize = config_.subpixelfit_window_size;
            cornerSubPix ( image_grey_, image_corners_, Size ( winSize, winSize ), Size ( -1, -1 ), TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
        }


        cam_model_.fromCameraInfo ( info_msg );
        intrinsic_matrix_ = cv::Mat_<double>::eye ( 4,4 );
        Mat camera_matrix = Mat ( cam_model_.intrinsicMatrix() );
        Mat dist_coeff = Mat ( cam_model_.distortionCoeffs() );

        if ( config_.input_raw == false ) {
            Mat projection_matrix = Mat ( cam_model_.projectionMatrix() );
            camera_matrix = projection_matrix ( cv::Rect ( 0,0,3,3 ) );
            dist_coeff = Mat::zeros ( 1,5,CV_32F );
        }
        /// @ToDo the following for loops can be done better
        for ( int r = 0; r < 3; r++ )
            for ( int c = 0; c < 3; c++ )
                intrinsic_matrix_ ( c,r ) = camera_matrix.at<double> ( c, r );

        const double &fx = intrinsic_matrix_ ( 0, 0 );
        const double &fy = intrinsic_matrix_ ( 1, 1 );
        const double &cx = intrinsic_matrix_ ( 0, 2 );
        const double &cy = intrinsic_matrix_ ( 1, 2 );
        double f = ( fx+fy ) /2.;





        Vec3d rotation_vec;
        Vec3d translation_vec;

        solvePnP ( object_corners_, image_corners_, cv::Mat ( intrinsic_matrix_, cv::Rect ( 0, 0, 3, 3 ) ) , dist_coeff, rotation_vec, translation_vec );

        // generate rotation matrix from vector
        extrinsic_matrix_ = cv::Mat_<double>::eye ( 4,4 );
        extrinsic_matrix_ ( 0,3 ) = translation_vec ( 0 );
        extrinsic_matrix_ ( 1,3 ) = translation_vec ( 1 );
        extrinsic_matrix_ ( 2,3 ) = translation_vec ( 2 );
        Rodrigues ( rotation_vec, cv::Mat ( extrinsic_matrix_, cv::Rect ( 0, 0, 3, 3 ) ), noArray() );
        projection_matrix_ = intrinsic_matrix_ * extrinsic_matrix_;

        // generate tf model to camera
        tf::Matrix3x3 R ( extrinsic_matrix_( 0, 0 ), extrinsic_matrix_( 0, 1 ), extrinsic_matrix_( 0, 2 ),
                          extrinsic_matrix_( 1, 0 ), extrinsic_matrix_( 1, 1 ), extrinsic_matrix_( 1, 2 ),
                          extrinsic_matrix_( 2, 0 ), extrinsic_matrix_( 2, 1 ), extrinsic_matrix_( 2, 2 ) );

        tf::Vector3 t ( translation_vec ( 0 ), translation_vec ( 1 ), translation_vec ( 2 ) );
        transform_ =  tf::Transform ( R, t );
        tf::Quaternion q = transform_.getRotation();

        if ( config_.plubish_tf ) {
            tf_broadcaster_->sendTransform ( tf::StampedTransform ( transform_, image_msg->header.stamp, image_msg->header.frame_id, checkerboard_frame_id_ ) );
        }
        if ( config_.plubish_marker ) {
            marker_detection_.header = image_msg->header;
            marker_detection_.distance_min = ( f*config_.checkerboard_square_size* ( config_.checkerboard_columns+1 ) ) / ( 2.0*cx );
            marker_detection_.distance_max = ( f*config_.checkerboard_square_size ) /config_.checkerboard_min_square_size;
            marker_detection_.distance_max_id = 0; //TODO
            marker_detection_.view_direction.x = 0; //TODO
            marker_detection_.view_direction.y = 0; //TODO
            marker_detection_.view_direction.z = 0; //TODO
            marker_detection_.view_direction.w = 1; //TODO
            marker_detection_.fov_horizontal = 2.0 * atan2 ( cx, fx );
            marker_detection_.fov_vertical = 2.0 * atan2 ( cy, fy );
            marker_detection_.type = "checkerboard";
            marker_detection_.markers.resize ( 1 );
            marker_msgs::Marker &marker =  marker_detection_.markers[0];

            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();
            marker.pose.position.x = t.x();
            marker.pose.position.y = t.y();
            marker.pose.position.z = t.z();
            pub_markers_.publish ( marker_detection_ );
        }
        if ( config_.publish_fiducials ) {
            fiducial_detection_.header = image_msg->header;
            fiducial_detection_.camera_d.resize(5);
            for (size_t i = 0; i < 5; i++)
                fiducial_detection_.camera_d[i] = dist_coeff.at<double> (0,i);
            for (size_t r = 0; r < 3; r++)
                for (size_t c = 0; c < 3; c++) 
                    fiducial_detection_.camera_k[r*3+c] = intrinsic_matrix_ ( r, c );
            fiducial_detection_.type = "checkerboard";
            fiducial_detection_.fiducial.resize ( 1 );
            marker_msgs::Fiducial &fiducial =  fiducial_detection_.fiducial[0];
            fiducial.object_points.resize(object_corners_.size());
            fiducial.image_points.resize(image_corners_.size());
            for (size_t i = 0; i < object_corners_.size(); i++){
                fiducial.object_points[i].x = object_corners_[i].x;
                fiducial.object_points[i].y = object_corners_[i].y;
                fiducial.object_points[i].z = object_corners_[i].z;
                
                fiducial.image_points[i].x = image_corners_[i].x;
                fiducial.image_points[i].y = image_corners_[i].y;
            }
            pub_fiducials_.publish ( fiducial_detection_ );
        }
        if ( config_.publish_pose ) {
            geometry_msgs::PoseStamped pose;
            pose.header = image_msg->header;
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            pose.pose.position.x = t.x();
            pose.pose.position.y = t.y();
            pose.pose.position.z = t.z();
            pub_pose_.publish(pose);
        }
        if ( config_.show_camera_image ) {

            double square_size = config_.checkerboard_square_size;
            double nr_of_square =  std::max ( config_.checkerboard_columns, config_.checkerboard_rows );
            double size =  square_size * nr_of_square;

            int font = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 1.0;
            double thickness = 1.0;
            double lineType = CV_AA;
            double lineThickness = 3;

            cv::Mat_<double> Pi0 = projection_matrix_ * ( cv::Mat_<double> ( 4,1 ) << 0, 0, 0, 1 );
            cv::Point2d pi0 ( Pi0 ( 0,0 ) / Pi0 ( 0,2 ), Pi0 ( 0,1 ) / Pi0 ( 0,2 ) );
            cv::circle ( image_rgb_, pi0, 3, CV_RGB ( 255,255,255 ) );

            cv::Mat_<double> Pi1 = projection_matrix_ * ( cv::Mat_<double> ( 4,1 ) << size, 0, 0, 1 );;
            cv::Point2d pi1 ( Pi1 ( 0,0 ) / Pi1 ( 0,2 ), Pi1 ( 0,1 ) / Pi1 ( 0,2 ) );
            cv::circle ( image_rgb_, pi1, 3, CV_RGB ( 255,0,0 ) );
            putText ( image_rgb_, "X", pi1, font, fontScale, CV_RGB ( 255,0,0 ), thickness, CV_AA );
            cv::line ( image_rgb_, pi0, pi1, CV_RGB ( 255,0,0 ), lineThickness );

            cv::Mat_<double> Pi2 = projection_matrix_ * ( cv::Mat_<double> ( 4,1 ) << 0, size, 0, 1 );
            cv::Point2d pi2 ( Pi2 ( 0,0 ) / Pi2 ( 0,2 ), Pi2 ( 0,1 ) / Pi2 ( 0,2 ) );
            cv::circle ( image_rgb_, pi2, 3, CV_RGB ( 0,255,0 ) );
            putText ( image_rgb_, "Y", pi2, font, fontScale, CV_RGB ( 0,255,0 ), thickness, CV_AA );
            cv::line ( image_rgb_, pi0, pi2, CV_RGB ( 0,255,0 ), lineThickness );

            cv::Mat_<double> Pi3 = projection_matrix_ * ( cv::Mat_<double> ( 4,1 ) << 0, 0, size, 1 );
            cv::Point2d pi3 ( Pi3 ( 0,0 ) / Pi3 ( 0,2 ), Pi3 ( 0,1 ) / Pi3 ( 0,2 ) );
            cv::circle ( image_rgb_, pi3, 3, CV_RGB ( 0,0,255 ) );
            putText ( image_rgb_, "Z", pi3, font, fontScale, CV_RGB ( 0,0,255 ) , thickness, CV_AA );
            cv::line ( image_rgb_, pi0, pi3, CV_RGB ( 0,0,255 ), lineThickness );

        }
    }
    if ( config_.show_camera_image ) {

        drawChessboardCorners ( image_rgb_, patternsize, Mat ( image_corners_ ), patternfound );
        cv::imshow ( nh_private_.getNamespace(), image_rgb_ );
        cv::waitKey ( config_.show_camera_image_waitkey );
    }
}


int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "tuw_checkerboard" );

    CheckerboardNode checkerboard_node;

    ros::spin();

    return 0;
}




