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


#include <camera_info_manager/camera_info_manager.h>
#include <tuw_uvc/uvc_defaults.h>
#include <tuw_uvc/uvc_ros.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include "luvcview/v4l2uvc.h"
#include <cv_bridge/cv_bridge.h>

typedef boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> Lock;

extern "C" {
    unsigned int Pyuv422torgb24(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height);
    unsigned int Pyuv422tobgr24(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height);
    unsigned int Pyuv422togray8(unsigned char *input_ptr, unsigned char *output_ptr, unsigned int image_width, unsigned int image_height);
}


V4RCamNode::~V4RCamNode()
{
    show_camera_image_ = false;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

V4RCamNode::V4RCamNode(ros::NodeHandle &n)
    : n_(n), n_param_("~"), imageTransport_(n_param_), generate_dynamic_reconfigure_(false), show_camera_image_(false), camera_freeze_(false), queueRosParamToV4LCommit_(0), showCameraImageThreadActive_(false)
{
  
    //n_param_.setParam("show_camera_image", show_camera_image_);
    cameraPublisher_ = imageTransport_.advertiseCamera("image_raw", 1);
    cameraThumbnailPublisher_ = imageTransport_.advertise("image_thumbnail", 1);
    //subSphere_ = n_.subscribe("sphere", 1000, &V4RCamNode::callbackSphere, this);
    readInitParams();
    initCamera();
    detectControlEnties();
    commitRosParamsToV4L(true);
    for(unsigned int  i = 0; i  < controlEntries_.size(); i++) {
        ROS_INFO_STREAM(controlEntries_[i]->getQueryCtrlInfo());
    }
}

void V4RCamNode::commitRosParamsToV4L(bool force)
{
    if((!queueRosParamToV4LCommit_) && (!force)) return;
    bool tmp;
    n_param_.getParam("show_camera_image", tmp);
    if(show_camera_image_ != tmp) {
        show_camera_image_ = tmp;
        if(show_camera_image_) {
            showCameraImageThread_ = boost::thread(&V4RCamNode::showCameraImage, this);
            ROS_INFO("start show camera image thread");
        }
    }
    for(unsigned int  i = 0; i < controlEntries_.size(); i++) {
        const std::string &name = controlEntries_[i]->varName;
        int default_value = controlEntries_[i]->queryctrl->default_value;
        int &target_value = controlEntries_[i]->targetValue;
        if(controlEntries_[i]->queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN) {
            n_param_.param<bool>(name, tmp, (bool) default_value);
            target_value = tmp;
        } else {
            n_param_.param<int>(name, target_value, default_value);
        }
        v4lset(controlEntries_[i]);
        if(controlEntries_[i]->hasErrorMsg()) ROS_ERROR_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullErrorMsg());
        if(controlEntries_[i]->hasInfoMsg()) ROS_INFO_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullInfoMsg());
    }
    //commitV4LToRosParams();
    queueRosParamToV4LCommit_ = false;
}
void V4RCamNode::commitV4LToRosParams()
{
    for(unsigned int  i = 0; i < controlEntries_.size(); i++) {
        v4lget(controlEntries_[i]);
        if(controlEntries_[i]->queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN) {
            n_param_.setParam(controlEntries_[i]->varName, (bool) controlEntries_[i]->currentValue);
            std::cout << controlEntries_[i]->varName << ": bool " << controlEntries_[i]->currentValue << std::endl;
        } else {
            n_param_.setParam(controlEntries_[i]->varName, (int) controlEntries_[i]->currentValue);
            std::cout << controlEntries_[i]->varName << ": int  " << controlEntries_[i]->currentValue << std::endl;
        }
        if(controlEntries_[i]->hasErrorMsg()) ROS_ERROR_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullErrorMsg());
        if(controlEntries_[i]->hasInfoMsg()) ROS_INFO_STREAM(controlEntries_[i]->varName << ": " << controlEntries_[i]->pullInfoMsg());
    }
}
void V4RCamNode::readInitParams()
{
    double tmp;
    ROS_INFO("V4RCamNode::readParams()");
    n_param_.param<bool>("show_camera_image", show_camera_image_, DEFAULT_SHOW_CAMERA_IMAGE);
    ROS_INFO("show_camera_image: %s", ((show_camera_image_) ? "true" : "false"));
    n_param_.param<bool>("camera_freeze", camera_freeze_, DEFAULT_CAMERA_FREEZE);
    ROS_INFO("camera_freeze: %s", ((camera_freeze_) ? "true" : "false"));

    n_param_.param<std::string>("video_device", videoDevice_, DEFAULT_VIDEODEVICE);
    ROS_INFO("video_device: %s", videoDevice_.c_str());
    n_param_.param<std::string>("avi_filename", aviFilename_, DEFAULT_AVIFILENAME);
    ROS_INFO("avi_filename: %s", aviFilename_.c_str());
    n_param_.param<int>("convert_image_", convert_image_, DEFAULT_CONVERT_IMAGE);
    ROS_INFO("convert_image: %i", convert_image_);
    n_param_.param<int>("ratioThumbnail", ratioThumbnail_, DEFAULT_RATIO_THUMBNAIL);
    ROS_INFO("ratioThumbnail: %i", ratioThumbnail_);
    n_param_.param<int>("width", width_, DEFAULT_WIDTH);
    ROS_INFO("width: %i", width_);
    n_param_.param<int>("height", height_, DEFAULT_HEIGHT);
    ROS_INFO("height: %i", height_);
    n_param_.param<double>("fps", tmp, DEFAULT_FPS);
    fps_ = tmp;
    ROS_INFO("fps: %4.3f", fps_);

    std::string camera_info_url;
    if(n_param_.getParam("camera_info_url", camera_info_url)) {
        camera_info_manager::CameraInfoManager cinfo(n_param_);
        if(cinfo.validateURL(camera_info_url)) {
            cinfo.loadCameraInfo(camera_info_url);
            cameraInfo_ = cinfo.getCameraInfo();
        } else {
            ROS_FATAL("camera_info_url not valid.");
            n_param_.shutdown();
            return;
        }
    } else {
        XmlRpc::XmlRpcValue double_list;
        n_param_.getParam("K", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
            for(int i = 0; i < 9; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.K[i] = double_list[i];
            }
        }

        n_param_.getParam("D", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 5)) {
            for(int i = 0; i < 5; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.D[i] = double_list[i];
            }
        }

        n_param_.getParam("R", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
            for(int i = 0; i < 9; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.R[i] = double_list[i];
            }
        }

        n_param_.getParam("P", double_list);
        if((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 12)) {
            for(int i = 0; i < 12; i++) {
                ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                cameraInfo_.P[i] = double_list[i];
            }
        }
    }
    ROS_INFO("tf_camera_id: %s", cameraInfo_.header.frame_id.c_str());
    n_param_.param<std::string>("frame_id", cameraInfo_.header.frame_id, DEFAULT_FRAME_ID);
    ROS_INFO("frame_id: %s", cameraInfo_.header.frame_id.c_str());
    cameraInfo_.header.seq  = 0;
}

void V4RCamNode::publishCamera()
{
    bool freeze = camera_freeze_ && (cameraImage_.header.seq > 100);
    cameraInfo_.header.stamp.sec = timeLastFrame_.tv_sec;
    cameraInfo_.header.stamp.nsec = timeLastFrame_.tv_usec * 1000;
    cameraImage_.header = cameraInfo_.header;
    if( !freeze) {
        cameraImage_.height = cameraInfo_.height = pVideoIn_->height;
        cameraImage_.width = cameraInfo_.width = pVideoIn_->width;
        cameraImage_.is_bigendian = true;
        cameraImage_.step = cameraInfo_.width * 2;
        switch(convert_image_) {
        case CONVERT_RAW:
            cameraImage_.encoding = "yuvu";
            cameraImage_.data.resize(pVideoIn_->framesizeIn);
            memcpy(&cameraImage_.data[0], pVideoIn_->framebuffer, cameraImage_.data.size());
            break;
        case CONVERT_YUV422toRGB:
            cameraImage_.encoding = "rgb8";
            cameraImage_.data.resize(width_ * height_ * 3);
            cameraImage_.step = cameraInfo_.width * 3;
            Pyuv422torgb24(pVideoIn_->framebuffer, &cameraImage_.data[0], width_, height_);
            break;
        case CONVERT_YUV422toBGR:
            cameraImage_.encoding = "bgr8";
            cameraImage_.data.resize(width_ * height_ * 3);
            cameraImage_.step = cameraInfo_.width * 3;
            Pyuv422tobgr24(pVideoIn_->framebuffer, &cameraImage_.data[0], width_, height_);
            break;
        case CONVERT_YUV422toGray:
            cameraImage_.encoding = "mono8";
            cameraImage_.data.resize(width_ * height_);
            cameraImage_.step = cameraInfo_.width * 1;
            Pyuv422togray8(pVideoIn_->framebuffer, &cameraImage_.data[0], width_, height_);
            break;
        default:
            cameraImage_.encoding = "yuv422";
            cameraImage_.data.resize(width_ * height_ * 2);
            memcpy(&cameraImage_.data[0], pVideoIn_->framebuffer, cameraImage_.data.size());
        }
    }
    cameraPublisher_.publish(cameraImage_, cameraInfo_);

    if(cameraThumbnailPublisher_.getNumSubscribers() > 0) {
        cameraThumbnail_.header = cameraInfo_.header;
        if( !freeze) {
            cameraThumbnail_.height = cameraInfo_.height = pVideoIn_->height / ratioThumbnail_;
            cameraThumbnail_.width = cameraInfo_.width = pVideoIn_->width / ratioThumbnail_;
            cameraThumbnail_.is_bigendian = true;
            cameraThumbnail_.step = cameraInfo_.width;
            cameraThumbnail_.encoding = "mono8";
            cameraThumbnail_.data.resize(cameraThumbnail_.width * cameraThumbnail_.height);
            unsigned char *des = (unsigned char *) &cameraThumbnail_.data[0];
            for(unsigned int h = 0; h < cameraThumbnail_.height; h++) {
                unsigned char *src = (unsigned char *) pVideoIn_->framebuffer + h*ratioThumbnail_ * pVideoIn_->width*2;
                for(unsigned int w = 0; w < cameraThumbnail_.width; w++) {
                    *des++ = src[w * 2*ratioThumbnail_];
                }
            }
            cameraThumbnailPublisher_.publish(cameraThumbnail_);
        }
    }

    commitRosParamsToV4L();
    cameraInfo_.header.seq ++;
    if(show_camera_image_ && showCameraImageThreadActive_ == false){
            showCameraImageThread_ = boost::thread(&V4RCamNode::showCameraImage, this);
  }
}


void V4RCamNode::showCameraImage()
{
    std::string window_name = ros::NodeHandle ( "~" ).getNamespace();
    timeval lastShownFrame = timeLastFrame_;
    double lastDuration = durationLastFrame_;
    cv::Mat img;
    int key = -1;
    showCameraImageThreadActive_ = true;
    do {
        if((lastShownFrame.tv_sec != timeLastFrame_.tv_sec) || (lastShownFrame.tv_usec != timeLastFrame_.tv_usec)) {
            img.create(height_, width_, CV_8UC3);
            lastShownFrame = timeLastFrame_;
            std::stringstream ss;
            ss << 1000.0 / ((durationLastFrame_ + lastDuration) / 2.0);
            lastDuration = durationLastFrame_;
            Lock myLock(mutexImage_);
	    if(!camera_freeze_){
	      Pyuv422tobgr24(pVideoIn_->framebuffer, img.data, width_, height_);
	      cv::putText(img, ss.str(), cv::Point(10, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0), 1, CV_AA);
	    }
            cv::imshow(window_name, img);
        }
        key =  cv::waitKey(50);
    } while((key == -1) && show_camera_image_ && showCameraImageThreadActive_);
    
    cv::putText(img, "OFFLINE", cv::Point(img.cols/4, img.rows/4), cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar::all(0), 3, CV_AA);
    cv::imshow(window_name, img);
    cv::waitKey(50);
    cv::destroyWindow(window_name);
    cv::waitKey(50);
    showCameraImageThreadActive_ = false;
    if(key != -1) {
        pVideoIn_->signalquit = 0;
    }
}

void V4RCamNode::callbackSphere (const tuw_uvc::SphereConstPtr& msg){
  
    ROS_INFO("action %s, pitch = %4.2f, yaw = %4.2f", msg->action.c_str(), msg->pitch, msg->yaw);
  
}