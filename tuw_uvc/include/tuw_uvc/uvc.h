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


#ifndef V4R_CAM_H
#define V4R_CAM_H

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <string>
#include <vector>
#include <sys/time.h>

#define V4L2_CID_BASE_EXTCTR				0x009A0900
#define V4L2_CID_BASE_LOGITECH				V4L2_CID_BASE_EXTCTR

struct vdIn;
struct v4l2_queryctrl;
struct v4l2_control;

/// v4l2 camera abstraction
class V4RCam
{
public:
    static const int OK = 0;
    static const int ERROR = 1;
    static const int NA = 0;
    static const int LOAD = 1;
    static const int SAVE = 2;
    typedef int FD;
    typedef boost::shared_ptr<v4l2_control> v4l2_controlPtr;  /// shared pointer for v4l2 control entries
    
    /// v4l2 control abstraction
    class ControlEntry
    {
    public:
        ControlEntry(int id); /// construtor
        ~ControlEntry(); /// destructor
        bool valid; 
        std::string varName; /// name of the v4l2 control
        v4l2_queryctrl *queryctrl; /// pointer to the original related control
        int currentValue;    /// current value of the control after read or set
        int targetValue;     /// target value to set after a write
        std::stringstream info_msg;  /// info msgs stream
        std::stringstream error_msg; /// error msgs in the case something happend
        std::string getQueryCtrlInfo() const; /// creates an info string related to the v4l2 control
        bool hasValidType() const;  /// return true if the value type is supported
        std::string pullErrorMsg();  /// clears the error_msgs stringstream and crates a info string
        std::string pullInfoMsg() ;  /// clears the info_msgs stringstream and crates a info string
        bool hasErrorMsg() const; /// returns false if therer are any error msgs waiting for pull
        bool hasInfoMsg() const; /// returns true if there are any info msgs waiting for pull
    };
    typedef boost::shared_ptr<ControlEntry> ControlEntryPtr;  /// shared pointer for ControlEntry
    V4RCam(); /// construtor
    ~V4RCam(); /// destructor
    /** grabs the next image from the butter 
     * @return false on an error or problem
     **/
    bool grab(); 
    ControlEntryPtr getControlEntry(std::string varName);
protected:
    vdIn *pVideoIn_; /// pointer to the v4l2 device
    std::string videoDevice_; /// device name /dev/videoX
    std::string aviFilename_; /// not supported yet @ToDo
    int format_; /// image formate
    int grabmethod_; /// @see v4l2 lib for more information
    int width_; /// image width
    int height_; /// image height
    float fps_; /// frames per second
    timeval timeLastFrame_; /// time stamp of the last frame
    double durationLastFrame_; /// duration between last and the frame before the last one
    boost::interprocess::interprocess_mutex mutexImage_; /// mutex to secure critical sections
    std::vector<ControlEntryPtr > controlEntries_;  /// vector of the current supported control entries

public:
    /**
     * Initializedes a video device
     * @param videoDevice like /dev/video0
     * @return fd to the video device (integer) usabel for v4l2_ioctl calles
     **/
    FD initCamera(const std::string &videoDevice = "");

    /// v4lcontrols

    /**
     * Delivers info to a control entry
     * @param entry entry to check
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     **/
    int v4lgetInfo(ControlEntryPtr entry);
    /**
     * reads a control entry and stores the value into entry.currentValue
     * @param entry entry to get
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     * @pre v4lgetInfo must be called at least one befor with this control instance
     **/
    int v4lget(ControlEntryPtr entry);

    /**
     * sets a control entry using entry.targetValue
     * @param entry entry to set
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     * @pre v4lgetInfo must be called at least one befor with this control instance
     **/
    int  v4lset(ControlEntryPtr entry);
    /**
     * updates a control entry using entry.targetValue
     * it also checks control min, max, stepsize
     * it also verifies the written data by reading the value again into  entry.currentValue
     * @param entry entry to update
     * @return error >= 1, ok = 0 details are written into entry.error_msg and entry.info_msg
     * @pre v4lgetInfo must be called at least one befor with this control instance
     **/
    int  v4lupdate(ControlEntryPtr entry);

    /** check if a control is backlisted
     * @param control
     * @return true if it is backlisted
     **/
    bool isBlackListed(int control);

    /**
     * detects the supporting v4l controls of the device
     * @returns controls
     **/
    const std::vector<ControlEntryPtr > &detectControlEnties();
    /**
     * saves control entries to a file
     * @return error >= 1, ok = 0 details are written into info_msg_ and entry.error_msg_
     * @see pullErrorMsg pullInfoMsg hasErrorMsg hasInfoMsg
     **/
    int save_controls(const std::string &filename);

    /**
     * loads control entries to a file
     * @return error >= 1, ok = 0 details are written into info_msg_ and entry.error_msg_
     * @see pullErrorMsg pullInfoMsg hasErrorMsg hasInfoMsg
     **/
    int load_controls(const std::string &filename);


    static char removeNonAlNum(char in);
    /**
     * @returns error messages
     **/
    std::string pullErrorMsg() ;
    /**
     * @returns info messages
     **/
    std::string pullInfoMsg() ;
    /**
     * @returns to if there are whaiting error messages
     **/
    bool hasErrorMsg() const;
    /**
     * @returns to if there are whaiting info messages
     **/
    bool hasInfoMsg() const;
protected:
    std::stringstream info_msg_;
    std::stringstream error_msg_;
};

#endif // V4R_CAM_H


