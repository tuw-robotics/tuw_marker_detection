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

#ifndef ARTOOLKITPLUS_NODE_DEFAULTS_H
#define ARTOOLKITPLUS_NODE_DEFAULTS_H

#define ARTOOLKITPLUS_IMAGE_SRC "image"
#define ARTOOLKITPLUS_DEBUG_WINDOWS_NAME "artoolkitplus"
#define ARTOOLKITPLUS_DEFAULT_DISTORTED_INPUT true
#define ARTOOLKITPLUS_DEFAULT_SKIP_FRAMES 0
#define ARTOOLKITPLUS_DEFAULT_SHOW_CAMERA_IMAGE true
#define ARTOOLKITPLUS_DEFAULT_BASE_FRAME "camera"
#define ARTOOLKITPLUS_DEFAULT_PATTERN_FRAME "pattern"
#define ARTOOLKITPLUS_DEFAULT_PATTERN_FILE ""
#define ARTOOLKITPLUS_DEFAULT_TF_PREFIX ""

#define ARTOOLKITPLUS_MARKER_MODE_BCH "bch"
#define ARTOOLKITPLUS_MARKER_MODE_SIMPEL "simple"
#define ARTOOLKITPLUS_DEFAULT_MARKER_MODE ARTOOLKITPLUS_MARKER_MODE_BCH // "SIMPLE
#define ARTOOLKITPLUS_DEFAULT_PATTERN_WITH 0.1
#define ARTOOLKITPLUS_DEFAULT_THRESHOLD 0
#define ARTOOLKITPLUS_DEFAULT_BOARDER_WIDTH 0
#define ARTOOLKITPLUS_DEFAULT_UNDIST_INTERATIONS 10
#define ARTOOLKITPLUS_UNDIST_MODE_NONE "none"
#define ARTOOLKITPLUS_UNDIST_MODE_STD "std"
#define ARTOOLKITPLUS_UNDIST_MODE_LUT "lut"
#define ARTOOLKITPLUS_DEFAULT_UNDIST_MODE ARTOOLKITPLUS_UNDIST_MODE_STD
#define ARTOOLKITPLUS_POSE_ESTIMATION_MODE_NORMAL "normal"
#define ARTOOLKITPLUS_POSE_ESTIMATION_MODE_CONT "cont"
#define ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP "rpp"
#define ARTOOLKITPLUS_DEFAULT_POSE_ESTIMATION_MODE ARTOOLKITPLUS_POSE_ESTIMATION_MODE_RPP
#define ARTOOLKITPLUS_DEFAULT_TRACKER_SINGLE_MARKER true
#define ARTOOLKITPLUS_DEFAULT_TRACKER_MULTI_MARKER false
#define ARTOOLKITPLUS_DEFAULT_MULIT_MARKER_LITE_DETECTION true

#endif // ARTOOLKITPLUS_NODE_DEFAULTS_H
