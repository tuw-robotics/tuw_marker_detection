/***************************************************************************
 *   Copyright (C) 2013 by Markus Bader                                    *
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

#include <tuw_artoolkitplus/artoolkitplus.h>
#include <tuw_artoolkitplus/artoolkitplus_defaults.h>

#include <ARToolKitPlus/ARToolKitPlus.h>



void ARToolKitPlusNode::Parameter::callbackParameters ( tuw_artoolkitplus::ARParamConfig &config, uint32_t level ) {
  show_camera_image_ = config.show_camera_image;
  distorted_input = config.distorted_input;
  skip_frames = config.skip_frames;
  useBCH = config.useBCH;
  borderWidth = config.borderWidth;
  patternWidth = config.patternWidth;
  edge_threshold = config.edge_threshold;
  undist_mode = config.undist_mode;
  pose_estimation_mode = config.pose_estimation_mode;
  use_multi_marker_lite_detection = config.use_multi_marker_lite_detection;
}

ARToolKitPlusNode::Parameter::Parameter()
    : node("~") 
    , node_name(node.getNamespace()){

    std::string tmp;
    
    node.param<bool>("show_camera_image", show_camera_image_, ARTOOLKITPLUS_DEFAULT_SHOW_CAMERA_IMAGE);
    ROS_INFO("%s: show_camera_image:  %s", node_name.c_str(), ((show_camera_image_) ? "true" : "false"));

    node.param<int>("skip_frames", skip_frames, ARTOOLKITPLUS_DEFAULT_SKIP_FRAMES);
    ROS_INFO("%s: skip_frames: %i", node_name.c_str(), skip_frames);


    node.param<bool>("tracker_single_marker", tracker_single_marker,  ARTOOLKITPLUS_DEFAULT_TRACKER_SINGLE_MARKER);
    ROS_INFO("%s: tracker_single_marker:  %s", node_name.c_str(), ((tracker_single_marker) ? "true" : "false"));

    node.param<bool>("tracker_multi_marker", tracker_multi_marker, ARTOOLKITPLUS_DEFAULT_TRACKER_MULTI_MARKER);
    ROS_INFO("%s: tracker_multi_marker:  %s", node_name.c_str(), ((tracker_multi_marker) ? "true" : "false"));

    if(!tracker_multi_marker && !tracker_single_marker) {
        ROS_ERROR("%s: at least tracker_multi_marker or tracker_single_marker must be true", node_name.c_str());
    }

    node.param<bool>("use_multi_marker_lite_detection", use_multi_marker_lite_detection, ARTOOLKITPLUS_DEFAULT_MULIT_MARKER_LITE_DETECTION);
    ROS_INFO("%s: use_multi_marker_lite_detection:  %s", node_name.c_str(), ((use_multi_marker_lite_detection) ? "true" : "false"));
    
    node.param<std::string>("pattern_frame", pattern_frame, ARTOOLKITPLUS_DEFAULT_PATTERN_FRAME);
    ROS_INFO("%s: pattern_frame: %s", node_name.c_str(), pattern_frame.c_str());

    node.param<std::string>("pattern_file", pattern_file, ARTOOLKITPLUS_DEFAULT_PATTERN_FILE);
    ROS_INFO("%s: pattern_file: %s", node_name.c_str(), pattern_file.c_str());
    if(!tracker_multi_marker && !pattern_file.empty()) {
        ROS_WARN("%s: tracker_multi_marker must be true in order to use mutli patterns with a pattern file", node_name.c_str());
    }

    node.param<std::string>("tf_prefix", tf_prefix, node_name);
    ROS_INFO("%s: tf_prefix: %s", node_name.c_str(), tf_prefix.c_str());


    node.param<std::string>("marker_mode", tmp, ARTOOLKITPLUS_MARKER_MODE_BCH);
    if ((tmp.compare(ARTOOLKITPLUS_MARKER_MODE_BCH) == 0) || (tmp.compare(ARTOOLKITPLUS_MARKER_MODE_SIMPEL) == 0)) {
        if (tmp.compare(ARTOOLKITPLUS_MARKER_MODE_BCH) == 0)
            useBCH = true;
        else
            useBCH = false;
        ROS_INFO("%s: marker_mode:  %s", node_name.c_str(), tmp.c_str());
    } else {
        ROS_ERROR("%s: marker_mode:  %s does not match any known type use %s or %s", node_name.c_str(), tmp.c_str(), ARTOOLKITPLUS_MARKER_MODE_SIMPEL, ARTOOLKITPLUS_MARKER_MODE_BCH);
    }

    node.param<double>("pattern_width", patternWidth, ARTOOLKITPLUS_DEFAULT_PATTERN_WITH);
    ROS_INFO("%s: pattern_width: %4.3f [m] only for single marker", node_name.c_str(), patternWidth);

    node.param<int>("edge_threshold", edge_threshold, ARTOOLKITPLUS_DEFAULT_THRESHOLD);
    ROS_INFO("%s: edge_threshold: %i, (0 = auto edge_threshold)", node_name.c_str(), edge_threshold);

    node.param<double>("border_width", borderWidth, ARTOOLKITPLUS_DEFAULT_BOARDER_WIDTH);
    ROS_INFO("%s: border_width: %4.3f, (0 = auto border_width)", node_name.c_str(), borderWidth);

    node.param<int>("undist_iterations", undist_iterations, ARTOOLKITPLUS_DEFAULT_UNDIST_INTERATIONS);
    ROS_INFO("%s: undist_iterations: %i", node_name.c_str(), undist_iterations);

    node.param<bool>("distorted_input", distorted_input, ARTOOLKITPLUS_DEFAULT_DISTORTED_INPUT);
    ROS_INFO("%s: distorted_input:  %s", node_name.c_str(), ((distorted_input) ? "true" : "false"));

    node.param<std::string>("undist_mode", tmp, ARTOOLKITPLUS_DEFAULT_UNDIST_MODE);
    if ((tmp.compare(ARTOOLKITPLUS_UNDIST_MODE_NONE) == 0) || (tmp.compare(ARTOOLKITPLUS_UNDIST_MODE_STD) == 0) || (tmp.compare(ARTOOLKITPLUS_UNDIST_MODE_LUT) == 0)) {
        if (tmp.compare(ARTOOLKITPLUS_UNDIST_MODE_NONE) == 0)
            undist_mode = ARToolKitPlus::UNDIST_NONE;
        if (tmp.compare(ARTOOLKITPLUS_UNDIST_MODE_STD) == 0)
            undist_mode = ARToolKitPlus::UNDIST_STD;
        if (tmp.compare(ARTOOLKITPLUS_UNDIST_MODE_LUT) == 0)
            undist_mode = ARToolKitPlus::UNDIST_LUT;
        ROS_INFO("%s: undist_mode:  %s", node_name.c_str(), tmp.c_str());
    } else {
        ROS_ERROR("%s: undist_mode:  %s does not match any known type use %s, %s  or %s", node_name.c_str(), tmp.c_str(), ARTOOLKITPLUS_UNDIST_MODE_NONE, ARTOOLKITPLUS_UNDIST_MODE_STD,
                  ARTOOLKITPLUS_UNDIST_MODE_LUT);
    }

    node.param<std::string>("pose_estimation_mode", tmp, ARTOOLKITPLUS_DEFAULT_POSE_ESTIMATION_MODE);
    if ((tmp.compare(ARTOOLKITPLUS_POSE_ESTIMATION_MODE_NORMAL) == 0) || (tmp.compare(ARTOOLKITPLUS_POSE_ESTIMATION_MODE_CONT) == 0) || (tmp.compare(ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP) == 0)) {
        if (tmp.compare(ARTOOLKITPLUS_POSE_ESTIMATION_MODE_NORMAL) == 0)
            pose_estimation_mode = ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL;
        if (tmp.compare(ARTOOLKITPLUS_POSE_ESTIMATION_MODE_CONT) == 0)
            pose_estimation_mode = ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL_CONT;
        if (tmp.compare(ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP) == 0)
            pose_estimation_mode = ARToolKitPlus::POSE_ESTIMATOR_RPP;
        ROS_INFO("%s: pose_estimation_mode:  %s", node_name.c_str(), tmp.c_str());
    } else {
        ROS_ERROR("%s: pose_estimation_mode:  %s does not match any known type use %s, %s  or %s", node_name.c_str(), tmp.c_str(), ARTOOLKITPLUS_POSE_ESTIMATION_MODE_NORMAL,
                  ARTOOLKITPLUS_POSE_ESTIMATION_MODE_CONT, ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP);
    }

    
    nPattern = -1;
    nUpdateMatrix = true;
    reconfigureFnc_ = boost::bind(&ARToolKitPlusNode::Parameter::callbackParameters, this ,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

