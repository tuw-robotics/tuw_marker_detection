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


#ifndef TUW_ELLIPSES_NODE_H
#define TUW_ELLIPSES_NODE_H

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_ellipses/EllipsesDetectionConfig.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tf/transform_broadcaster.h>
#include <tuw_ellipses/ellipses_detection.h>
#include <marker_msgs/FiducialDetection.h>
#include <marker_msgs/MarkerDetection.h>

namespace tuw {

/// ROS Node
class EllipsesDetectionNode : public tuw::EllipsesDetection, public nodelet::Nodelet {
public:
    struct ParametersNode : public Parameters {
        ParametersNode();
        void update(const unsigned long &counter);
        void callbackParameters (tuw_ellipses::EllipsesDetectionConfig &config, uint32_t level );
        ros::NodeHandle node;
        dynamic_reconfigure::Server<tuw_ellipses::EllipsesDetectionConfig> reconfigureServer_;
        dynamic_reconfigure::Server<tuw_ellipses::EllipsesDetectionConfig>::CallbackType reconfigureFnc_;
        std::string node_name;
        bool debug_freeze;
        bool show_camera_image;
        bool publishTF;
        bool publishMarker;
        bool publishFiducials;
        int show_camera_image_waitkey;
        int image_skip;
        bool skip_second_tf;
        std::string tf_prefix;
    };
    EllipsesDetectionNode ( );
    ~EllipsesDetectionNode();
    void init ();
    virtual void onInit();
    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                       const sensor_msgs::CameraInfoConstPtr& info_msg);
private: //functions
    const ParametersNode *param();
    void update ();
    void publishTf();
    void publishMarker (const std_msgs::Header &header);
    void publishFiducials (const std_msgs::Header &header);
    void createTransforms(const std_msgs::Header &header);
private: // variables
    ros::NodeHandle n_;
    unsigned long  callback_counter_;
    tf::TransformBroadcaster transformBroadcaster_;
    ros::Publisher pub_ellipses_;
    ros::Publisher pub_perceptions_;
    ros::Publisher pub_fiducials_;
    image_transport::ImageTransport imageTransport_;
    image_transport::CameraSubscriber  sub_camera_;
    cv_bridge::CvImagePtr image_mono_;
    sensor_msgs::CameraInfoConstPtr camera_info_;
    boost::posix_time::ptime timeCallbackReceivedLast_;
    boost::posix_time::ptime timeCallbackReceived_;
    boost::posix_time::ptime timeDetectionStart_;
    boost::posix_time::ptime timeDetectionEnd_;
    std::list<tf::StampedTransform> markerTransforms_;
};
}

#endif //TUW_ELLIPSES_NODE_H
