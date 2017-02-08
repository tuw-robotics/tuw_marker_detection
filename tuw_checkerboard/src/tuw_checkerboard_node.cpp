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
    pub_markers_ = nh_.advertise<marker_msgs::MarkerDetection>("markers", 10);
    // Advert fiducial publisher
    pub_fiducials_ = nh_.advertise<marker_msgs::FiducialDetection>("fiducials", 10);

    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
    
    nh_.param<std::string>("frame_id", checkerboard_frame_id_, "checkerboard");

    reconfigureServer_ = new dynamic_reconfigure::Server<tuw_checkerboard::CheckerboardDetectionConfig> ( ros::NodeHandle ( "~" ) );
    reconfigureFnc_ = boost::bind ( &CheckerboardNode::callbackConfig, this,  _1, _2 );
    reconfigureServer_->setCallback ( reconfigureFnc_ );

    sub_cam_ = it_.subscribeCamera ( "image", 1, &CheckerboardNode::callbackCamera, this );

}

void CheckerboardNode::callbackConfig ( tuw_checkerboard::CheckerboardDetectionConfig &_config, uint32_t _level ) {
    config_ = _config;
}
/*
 * Camera callback
 * detects chessboard pattern using opencv and finds camera to image tf using solvePnP
 */
void CheckerboardNode::callbackCamera ( const sensor_msgs::ImageConstPtr& image_msg,
                                        const sensor_msgs::CameraInfoConstPtr& info_msg ) {
    vector<Point2f> image_corners;
    Size patternsize ( config_.checkerboard_columns, config_.checkerboard_rows );
    cv_bridge::CvImagePtr input_bridge;
    try {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
        image_grey_ = input_bridge->image;

    } catch ( cv_bridge::Exception& ex ) {
        ROS_ERROR ( "[camera_tf_node] Failed to convert image" );
        return;
    }
    int flags = 0;
    if(config_.adaptive_thresh) flags += CV_CALIB_CB_ADAPTIVE_THRESH;
    if(config_.normalize_image) flags += CV_CALIB_CB_NORMALIZE_IMAGE;
    if(config_.filter_quads) flags += CV_CALIB_CB_FILTER_QUADS;
    if(config_.fast_check) flags += CALIB_CB_FAST_CHECK;
    bool patternfound = findChessboardCorners ( image_grey_, patternsize, image_corners, flags );

    if ( patternfound ) {
        if(config_.subpixelfit){
            int winSize = config_.subpixelfit_window_size;
            cornerSubPix ( image_grey_, image_corners, Size ( winSize, winSize ), Size ( -1, -1 ), TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ) );
        }
        // generate object points
        float square_size = float ( config_.checkerboard_square_size ); // chessboard square size unit defines output unit
        vector<Point3f> object_corners;

        for ( int i = 0; i < patternsize.height; i++ ) {
            for ( int j = 0; j < patternsize.width; j++ ) {
                object_corners.push_back ( Point3f ( float ( i * square_size ), float ( j * square_size ), 0.f ) );
            }
        }

        cam_model_.fromCameraInfo ( info_msg );
        Mat camera_matrix = Mat ( cam_model_.intrinsicMatrix() );
        Mat dist_coeff = cam_model_.distortionCoeffs();
        Vec3d rotation_vec;
        Vec3d translation_vec;

        solvePnP ( object_corners, image_corners, camera_matrix, dist_coeff, rotation_vec, translation_vec );

        // generate rotation matrix from vector
        Mat rotation_mat;
        Rodrigues ( rotation_vec, rotation_mat, noArray() );

        // generate tf model to camera
        tf::Matrix3x3 R ( rotation_mat.at<double> ( 0, 0 ), rotation_mat.at<double> ( 0, 1 ), rotation_mat.at<double> ( 0, 2 ),
                          rotation_mat.at<double> ( 1, 0 ), rotation_mat.at<double> ( 1, 1 ), rotation_mat.at<double> ( 1, 2 ),
                          rotation_mat.at<double> ( 2, 0 ), rotation_mat.at<double> ( 2, 1 ), rotation_mat.at<double> ( 2, 2 ) );

        tf::Vector3 T = tf::Vector3 ( translation_vec ( 0 ), translation_vec ( 1 ), translation_vec ( 2 ) );

        tf::Transform cam_to_checker ( R, T );
        
        if(config_.plubishTF){
            tf_broadcaster_->sendTransform(tf::StampedTransform(cam_to_checker, image_msg->header.stamp, checkerboard_frame_id_, image_msg->header.frame_id));
        }
    }

    if ( config_.show_camera_image ) {
        cvtColor ( image_grey_, image_rgb_, CV_GRAY2BGR, 0 );
        drawChessboardCorners ( image_rgb_, patternsize, Mat ( image_corners ), patternfound );
        cv::imshow ( nh_private_.getNamespace(), image_rgb_ );
        cv::waitKey ( config_.show_camera_image_waitkey );
    }

    //pub_image_.publish(input_bridge->toImageMsg());
}

int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "tuw_checkerboard" );

    CheckerboardNode checkerboard_node;

    ros::spin();

    return 0;
}


