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



#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "checkerboard_detection/GetPose.h"

#include <boost/concept_check.hpp>

#define DEFAULT_LOOP_RATE 1.0
#define SERVICE_NAME "pose" //Default "pose"

#include <boost/filesystem/operations.hpp>
#include <v4r/tf/pose.h>

cv::Mat_<double> tvec, quat;

bool existsFile ( const std::string &rFile ) {
    boost::filesystem::path dir_path = boost::filesystem::complete ( boost::filesystem::path ( rFile, boost::filesystem::native ) );
    if ( boost::filesystem::exists ( dir_path ) && boost::filesystem::is_regular_file(dir_path)) {
        return true;
    } else {
        return false;
    }
}

bool response (geometry_msgs::pose::Request &req, geometry_msgs::pose::Response &resp)
{
    resp.pose.position.x = tvec( 0, 0);
    resp.pose.position.y = tvec( 1, 0);
    resp.pose.position.z = tvec( 2, 0);
    resp.pose.orientation.x = quat(0,0);
    resp.pose.orientation.y = quat(1,0);ros
    resp.pose.orientation.z = quat(2,0);
    resp.pose.orientation.w = quat(3,0);
    return true;
}

int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "static_pose" );
    cvStartWindowThread();
    ros::NodeHandle nh;
    ros::NodeHandle private_nh ( "~" );
    tf::TransformBroadcaster broadcaster;

    std::string pose_file;
    private_nh.getParam ( "pose_file", pose_file );
    if (pose_file.empty()) {
        ROS_ERROR("pose_file parameter is empty");
        return 1;
    }
    ROS_INFO("Pose file: %s", pose_file.c_str());

    std::string service_name(SERVICE_NAME);
    private_nh.getParam ( "service_name", service_name );
    ROS_INFO("\tService name: %s", service_name.c_str());

    if (!existsFile(pose_file)) {
        ROS_INFO("The pose file: %s does not exist, I will create a empty one", pose_file.c_str());
        cv::FileStorage fs ( pose_file, cv::FileStorage::WRITE );
        fs << "Base_frame" << "base_link";
        fs << "Frame_id" << "static_link";

        fs << "Q" << (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        fs << "T" << (cv::Mat_<double>(3, 1) << 1, 0, 0);
    }

    double loopFrq;
    private_nh.param ( "loopFrq", loopFrq, DEFAULT_LOOP_RATE );

    std::string base_frame;
    std::string frame_id;
    {
      V4R::PoseD pose;
      pose.read(pose_file, base_frame, frame_id);
      quat = pose.quaterion();
      tvec = pose.tvec();
    }
    if ( tvec.empty() || quat.empty() || base_frame.empty() || frame_id.empty()) {
        ROS_ERROR("Error in pose file!");
        return 1;
    }
    tf::Transform transform(tf::Quaternion(quat(0,0), quat(1,0), quat(2,0), quat(3,0)), tf::Vector3 ( tvec( 0, 0), tvec( 1, 0), tvec( 2, 0) ) );


    ros::ServiceServer service;
    if (!service_name.empty()) {
        ros::NodeHandle nh_toplevel_;
        service = private_nh.advertiseService(service_name, response);
    }


    ros::Rate rate ( loopFrq );
    while ( ros::ok() )
    {
        broadcaster.sendTransform (tf::StampedTransform ( transform, ros::Time::now(), base_frame, frame_id ) );
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
