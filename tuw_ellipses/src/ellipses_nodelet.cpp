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
#include "ellipses_nodelet.h"
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <marker_msgs/MarkerDetection.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(tuw, EllipsesDetectionNode, tuw::EllipsesDetectionNode, nodelet::Nodelet)
using namespace tuw;

EllipsesDetectionNode::~EllipsesDetectionNode() {
}

EllipsesDetectionNode::EllipsesDetectionNode() :
    EllipsesDetection(new EllipsesDetectionNode::ParametersNode()), n_(), callback_counter_(0), imageTransport_(n_) {

}

const EllipsesDetectionNode::ParametersNode *EllipsesDetectionNode::param() {
    return (EllipsesDetectionNode::ParametersNode*) param_;
}

void EllipsesDetectionNode::init() {
    sub_camera_ = imageTransport_.subscribeCamera( "image", 1, &EllipsesDetectionNode::imageCallback, this );
    pub_perceptions_ =  n_.advertise<marker_msgs::MarkerDetection>("marker", 1000);
    pub_fiducials_ = n_.advertise<marker_msgs::FiducialDetection>("fiducials", 1000);
}

void EllipsesDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camer_info) {

    if(callback_counter_ == 0) timeCallbackReceived_ = boost::posix_time::microsec_clock::local_time();
    callback_counter_++;
    if((param()->image_skip >= 0) && (callback_counter_ % (param()->image_skip+1) != 0)) return;
    timeCallbackReceivedLast_ = timeCallbackReceived_;
    timeCallbackReceived_ = boost::posix_time::microsec_clock::local_time();

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo ( camer_info );
    try {
        if((image_mono_ == NULL) || !param()->debug_freeze) {
            image_mono_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            camera_info_ = camer_info;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::Mat image = cv::imread("/home/max/Downloads/image001.png", CV_LOAD_IMAGE_COLOR); cv::cvtColor(image, image_mono_->image, CV_RGB2GRAY);
    timeDetectionStart_ = boost::posix_time::microsec_clock::local_time();
    next();
    std::vector<cv::RotatedRect> ellipses;
    cv::Mat intrinsic(cam_model.intrinsicMatrix());
    cv::Mat projection(cam_model.projectionMatrix());
    fit_ellipses_opencv (image_mono_->image, intrinsic, cam_model.distortionCoeffs(), projection, image_msg->header.stamp.toBoost() );
    createRings();
    createTransforms(image_msg->header);
    timeDetectionEnd_ = boost::posix_time::microsec_clock::local_time();
    if(param()->publishTF) publishTf();
    if(param()->publishMarker) publishMarker(image_msg->header);
    if(param()->publishFiducials) publishFiducials(image_msg->header);

    if (param()->show_camera_image) {
        cv::Mat img_debug;
        std::stringstream ss;
        ss << "capture: " << std::setw(3) << (timeDetectionStart_ - timeCallbackReceived_).total_milliseconds() << "ms, ";
        ss << "detection: " << std::setw(3) << (timeDetectionEnd_ - timeDetectionStart_).total_milliseconds() << "ms, ";
        ss << "total: " << std::setw(3) << (timeDetectionEnd_ - timeCallbackReceived_).total_milliseconds() << "ms, ";
        ss << "interval: " << std::setw(3) << (timeCallbackReceived_ - timeCallbackReceivedLast_).total_milliseconds() << "ms";
        if((timeCallbackReceived_ - timeCallbackReceivedLast_).total_milliseconds() > 0) {
            ss << " = " << std::setw(3) << 1000 /(timeCallbackReceived_ - timeCallbackReceivedLast_).total_milliseconds() << "Hz";
        }
        cvtColor(image_mono_->image, img_debug, CV_GRAY2BGR);
        draw_ellipses(img_debug);
        cv::putText(img_debug, ss.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar::all(0), 1, CV_AA);
        cv::imshow( param()->node_name + std::string(" - input"), img_debug);
        if(param()->debug) {
            cv::imshow( param()->node_name + std::string(" - edges"), imgEdges_);
            if(!imgGradient_.empty()) {
                cv::imshow( param()->node_name + std::string(" - gradient"), imgGradient_ * (0xFFFF/0x1FF));
            }
            if(!imgDirection_.empty()) {
                cv::imshow( param()->node_name + std::string(" - direction"), imgDirection_ * 0xFFFF/0x1FF);
            }
            if(!imgSobelDx_.empty()) {
                cv::imshow( param()->node_name + std::string(" - imgSobelDx_"), imgSobelDx_ * (0xFFFF/0x1FF));
            }
            if(!imgSobelDy_.empty()) {
                cv::imshow( param()->node_name + std::string(" - imgSobelDy_"), imgSobelDy_ * (0xFFFF/0x1FF));
            }
        }
        cv::waitKey(param()->show_camera_image_waitkey);
    }
}

void EllipsesDetectionNode::onInit()
{
    init();
}

void EllipsesDetectionNode::createTransforms(const std_msgs::Header &header) {
    if(param()->pose_estimation == POSE_ESTIMATION_OFF) return;
    estimatePoses();
    tf::Transform trans;
    tf::StampedTransform st;
    char frame[0xFF];
    markerTransforms_.clear();
    for(unsigned int i = 0; i < 2; i++) {
        if(param()->skip_second_tf && (i == 1)) continue;
        for(std::list<Marker>::iterator it = markers_.begin(); it != markers_.end(); it++) {
            Marker &m = *it;
            sprintf(frame, "t%i-%i", i, m.id);
            std::string child_frame = tf::resolve(param()->tf_prefix, frame);
            tf::Quaternion roation;
            tf::Vector3 translation;
            tf::Matrix3x3 R;
            translation.setValue(m.translations[i].x, m.translations[i].y, m.translations[i].z);
            R.setValue (m.R[i](0,0), m.R[i](0,1), m.R[i](0,2), m.R[i](1,0), m.R[i](1,1), m.R[i](1,2), m.R[i](2,0), m.R[i](2,1), m.R[i](2,2));
            R.getRotation(roation);
            trans = tf::Transform(roation, translation);
            st = tf::StampedTransform(trans, header.stamp, header.frame_id, child_frame);
            markerTransforms_.push_back(st);
        }
    }
}


void EllipsesDetectionNode::publishTf() {
    for(std::list<tf::StampedTransform>::iterator it =  markerTransforms_.begin(); it != markerTransforms_.end(); it++) {
        transformBroadcaster_.sendTransform(*it);
    }

}

void EllipsesDetectionNode::publishMarker (const std_msgs::Header &header) {
    if(pub_perceptions_.getNumSubscribers() < 1) return;
    marker_msgs::MarkerDetection msg;
    if(markerTransforms_.size() > 0) {
        msg.header = header;
        msg.distance_min =  0; //TODO
        msg.distance_max =  8; //TODO
        msg.distance_max_id = 5; //TODO
        msg.view_direction.x = 0; //TODO
        msg.view_direction.y = 0; //TODO
        msg.view_direction.z = 0; //TODO
        msg.view_direction.w = 1; //TODO
        msg.fov_horizontal = 6; //TODO
        msg.fov_vertical = 0; //TODO
        msg.type = "ellipses";
        msg.markers.resize(markerTransforms_.size());
        std::list<tf::StampedTransform>::iterator it =  markerTransforms_.begin();
        for(size_t i = 0; i < markerTransforms_.size(); it++, i++) {
            marker_msgs::Marker &marker = msg.markers[i];
            // marker.ids              ellipses have no id
            // marker.ids_confidence   ellipses have no id
            tf::Vector3 &srcT = it->getOrigin();
            marker.pose.position.x = srcT.x();
            marker.pose.position.y = srcT.y();
            marker.pose.position.z = srcT.z();
            tf::Quaternion srcQ = it->getRotation();
            marker.pose.orientation.x = srcQ.x();
            marker.pose.orientation.y = srcQ.y();
            marker.pose.orientation.z = srcQ.z();
            marker.pose.orientation.w = srcQ.w();
        }
        pub_perceptions_.publish(msg);
    }

}

void EllipsesDetectionNode::publishFiducials(const std_msgs::Header &header) {
    marker_msgs::FiducialDetection msg;
    msg.header = header;
    msg.camera_k = camera_info_->K;
    msg.camera_d = camera_info_->D;

    for (std::vector<Ellipse>::iterator it = ellipses_.begin(); it != ellipses_.end(); it++) {
        Ellipse &e = *it;
        if(e.detection != VALID) continue;

        marker_msgs::Fiducial fiducial;

        geometry_msgs::Point objectPoint;
        objectPoint.x = 0.0f;
        objectPoint.y = 0.0f;
        objectPoint.z = 0.0f;
        fiducial.object_points.push_back(objectPoint);

        geometry_msgs::Point imagePoint;
        imagePoint.x = e.boxEllipse.center.x;
        imagePoint.y = e.boxEllipse.center.y;
        imagePoint.z = 0.0f; // imagePoints are 2d
        fiducial.image_points.push_back(imagePoint);

        fiducial.ids.resize(0);
        fiducial.ids_confidence.resize(0);

        msg.fiducial.push_back(fiducial);
    }

    pub_fiducials_.publish(msg);
}
