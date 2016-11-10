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
#include "ellipses_nodelet_defaults.h"

using namespace tuw;

EllipsesDetectionNode::ParametersNode::ParametersNode()
    : Parameters() 
    , node("~")
    , node_name(node.getNamespace())
    , debug_freeze(TUW_ELLIPSES_NODE_DEFAULT_DEBUG_FREEZE)
    , publishTF(TUW_ELLIPSES_NODE_PUBLISH_TF)
    , publishMarker(TUW_ELLIPSES_NODE_PUBLISH_MARKER)
    , publishFiducials(TUW_ELLIPSES_NODE_PUBLISH_FIDUCIALS)
    , show_camera_image(TUW_ELLIPSES_NODE_DEFAULT_SHOW_CAMERA_IMAGE)
    , show_camera_image_waitkey(TUW_ELLIPSES_NODE_DEFAULT_SHOW_CAMERA_IMAGE_WAITKEY)
    , image_skip(TUW_ELLIPSES_NODE_DEFAULT_IMAGE_SKIP)
    , skip_second_tf(TUW_ELLIPSES_NODE_DEFAULT_SKIP_SECOND_TF)
    , tf_prefix(node_name)
    {
    node.getParam("debug_freeze", debug_freeze);
    ROS_INFO("%s - debug_freeze:  %s", node_name.c_str(), (debug_freeze ? "true" : "false"));
    node.getParam("show_camera_image", show_camera_image);
    ROS_INFO("%s - show_camera_image:  %s", node_name.c_str(), (show_camera_image ? "true" : "false"));
    node.getParam("show_camera_image_waitkey", show_camera_image_waitkey);
    ROS_INFO("%s - show_camera_image_waitkey: %i", node_name.c_str(), show_camera_image_waitkey);
    node.getParam("image_skip", image_skip);
    ROS_INFO("%s - image_skip: %i", node_name.c_str(), image_skip);
    node.param<std::string>("tf_prefix", tf_prefix, node_name);
    ROS_INFO("%s: tf_prefix: %s", node_name.c_str(), tf_prefix.c_str());
    node.getParam("skip_second_tf", skip_second_tf);
    ROS_INFO("%s - skip_second_tf:  %s", node_name.c_str(), (skip_second_tf ? "true" : "false"));

    reconfigureFnc_ = boost::bind(&EllipsesDetectionNode::ParametersNode::callbackParameters, this ,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);


}


void EllipsesDetectionNode::ParametersNode::callbackParameters (tuw_ellipses::EllipsesDetectionConfig &config, uint32_t level ) {
  int kernal_sizes[] = {1, 3, 5, 7};
  show_camera_image = config.show_camera_image;
  show_camera_image_waitkey = config.show_camera_image_waitkey;
  debug = config.debug;
  distorted_input = config.distorted_input;
  debug_freeze = config.debug_freeze;
  image_skip = config.image_skip;
  edge_detection = (EdgeDetection) config.edge_detection;
  threshold_edge_detection1 = config.threshold_edge_detection1;
  threshold_edge_detection2 = config.threshold_edge_detection2;
  if(config.kernel_size_edge_detection > 7) kernel_size_edge_detection = 7;
  else if(config.kernel_size_edge_detection < 3) kernel_size_edge_detection = 3;
  else if(config.kernel_size_edge_detection % 2) kernel_size_edge_detection = config.kernel_size_edge_detection;
  else kernel_size_edge_detection = config.kernel_size_edge_detection+1;
  edge_linking = (EdgeLinking) config.edge_linking;
  threshold_contour_min_points = config.threshold_contour_min_points;
  threshold_polygon = config.threshold_polygon;
  filter_convex = config.filter_convex;
  ellipse_redefinement = config.ellipse_redefinement;
  threshold_rotated_rect_ratio = config.threshold_rotated_rect_ratio;
  filter_contour_mean = config.filter_contour_mean;
  threshold_min_radius = config.threshold_min_radius;
  threshold_max_radius = config.threshold_max_radius;
  filter_rings = config.filter_rings;
  threshold_ring_center = config.threshold_ring_center;
  threshold_ring_ratio = config.threshold_ring_ratio;
  pose_estimation = (PoseEstimation) config.pose_estimation;
  circle_diameter = config.circle_diameter;
  skip_second_tf = config.skip_second_tf;
  
}
