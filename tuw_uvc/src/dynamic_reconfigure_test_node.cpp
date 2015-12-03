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


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_uvc/CameraLogitechConfig.h>

class TestNode  {
public:
    TestNode ( ros::NodeHandle &n )
        : n_(n), n_param_("~") {
        reconfigureFnc_ = boost::bind(&TestNode::callbackParameters, this,  _1, _2);
        reconfigureServer_.setCallback(reconfigureFnc_);
    }
    void callbackParameters ( tuw_uvc::CameraLogitechConfig &config, uint32_t level ) {
        ROS_INFO("callbackParameters");
    }
protected:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    dynamic_reconfigure::Server<tuw_uvc::CameraLogitechConfig> reconfigureServer_;
    dynamic_reconfigure::Server<tuw_uvc::CameraLogitechConfig>::CallbackType reconfigureFnc_;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    TestNode node(n);
    ros::Rate rate(100);
    ROS_INFO("Starting to spin...");
    ros::spin();
    return 0;
}

