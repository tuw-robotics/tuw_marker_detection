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


#include <tuw_uvc/uvc_ros.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_uvc/CameraGeneralConfig.h>

class V4RLogitechNode : public V4RCamNode {
public:
    V4RLogitechNode ( ros::NodeHandle &n ):V4RCamNode(n) {
        reconfigureFnc_ = boost::bind(&V4RLogitechNode::callbackParameters, this,  _1, _2);
        reconfigureServer_.setCallback(reconfigureFnc_);
    }
    void callbackParameters ( tuw_uvc::CameraGeneralConfig &config, uint32_t level ) {  
        show_camera_image_ = config.show_camera_image; 
        camera_freeze_ = config.camera_freeze;
	queueRosParamToV4LCommit_ = true;
    }
protected:
    dynamic_reconfigure::Server<tuw_uvc::CameraGeneralConfig> reconfigureServer_;
    dynamic_reconfigure::Server<tuw_uvc::CameraGeneralConfig>::CallbackType reconfigureFnc_;
protected:
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "tuw_uvc_general");
    ros::NodeHandle n;
    V4RLogitechNode node(n);
    ros::Rate rate(100);
    while(ros::ok() && node.grab()) {
        node.publishCamera();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

