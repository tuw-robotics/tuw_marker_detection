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


#ifndef LUVC_DEFAULTS_H
#define LUVC_DEFAULTS_H
#include <linux/videodev2.h>

#define DEFAULT_SHOW_CAMERA_IMAGE true
#define DEFAULT_CAMERA_FREEZE false
#define DEFAULT_CONVERT_IMAGE 1
#define DEFAULT_FRAME_ID "UVC_CAM"
#define DEFAULT_VIDEODEVICE "/dev/video0"
#define DEFAULT_AVIFILENAME ""
#define DEFAULT_FORMAT      V4L2_PIX_FMT_MJPEG
#define DEFAULT_GRABMETHODE 1
#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define DEFAULT_FPS 30.0
#define DEFAULT_RATIO_THUMBNAIL 8



#endif // LUVC_DEFAULTS_H
