/***************************************************************************
 *   Copyright (C) 2012 by Markus Bader                                    *
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


#ifndef V4R_CAM_NODE_H
#define V4R_CAM_NODE_H

#include <ros/ros.h>
#include <tuw_uvc/uvc.h>
#include <tuw_uvc/Sphere.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

/// ROS camera abstraction
class V4RCamNode : public V4RCam {
public:
    static const int CONVERT_RAW = 0;   	
    static const int CONVERT_YUV422toRGB = 1;
    static const int CONVERT_YUV422toBGR = 2;
    static const int CONVERT_YUV422toGray = 3;

    V4RCamNode ( ros::NodeHandle &n );
    ~V4RCamNode();
    void publishCamera();
    void showCameraImage();
protected:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    image_transport::ImageTransport  imageTransport_;
    image_transport::CameraPublisher cameraPublisher_;
    image_transport::Publisher cameraThumbnailPublisher_;
    sensor_msgs::CameraInfo cameraInfo_;
    sensor_msgs::Image cameraImage_;
    sensor_msgs::Image cameraThumbnail_;
    ros::Subscriber subSphere_;
    void callbackSphere (const tuw_uvc::SphereConstPtr& msg);
    bool generate_dynamic_reconfigure_;
    bool show_camera_image_;
    bool camera_freeze_;
    bool queueRosParamToV4LCommit_;
    bool showCameraImageThreadActive_;
    boost::thread showCameraImageThread_;
protected:
    void readInitParams();
    void commitRosParamsToV4L(bool force = false);
    void commitV4LToRosParams();
    void loopCamera();
    int convert_image_;
    int ratioThumbnail_;

    /**
     * reads and updates all local control values
     * @param controls control parameters
     **/
    void readCameraControls();
    /**
     * writes does parameter which are different to the current ones to the camera
     * @param control control parameters
     **/
    void writeCameraControls();

    /**
     * reads a control entr
     * @param entry control entry
     **/
    void readControlEntryInfo ( ControlEntry *entry );
    /**
     * updates a control entr
     * @param entry control entry
     **/
    void updateControlEntry ( ControlEntry *entry );
    /**
     * generates a new reconfigure file
     **/
    void updateDynamicReconfigureFile ( const char* filename ) const;
};

#endif // V4R_CAM_NODE_H
