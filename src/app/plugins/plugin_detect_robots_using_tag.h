//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    plugin_detect_robots_using_tag.h
  \brief   C++ Interface: plugin_detect_robots_using_tag
  \author  Mark ZJUNlict, 2022
*/
//========================================================================
#ifndef PLUGIN_DETECT_ROBOTS_USING_TAG_H
#define PLUGIN_DETECT_ROBOTS_USING_TAG_H

#include <visionplugin.h>
#include "cmvision_region.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "camera_calibration.h"
#include "field_filter.h"
#include "cmvision_histogram.h"
#include "cmpattern_teamdetector.h"
#include "cmpattern_team.h"
#include "vis_util.h"
#include "lut3d.h"
#include "VarNotifier.h"
#include <opencv2/opencv.hpp>

#include <aruco/aruco.h>

struct TagResults{
    std::vector<int> markerIds;
    std::vector<vector2d> markerCenters;
    std::vector<vector2d> markerFaceCenters;
};

class PluginDetectRobotsArUco : public VisionPlugin
{
protected:
    VarNotifier _notifier;
    VarList * _settings;
        VarDouble * _shrink_ratio;
        VarDouble * _min_marker_ratio;
        VarStringEnum * _detection_mode;

    const CameraParameters& camera_parameters;
    const RoboCupField& field;

    cv::Mat inputImage;
    cv::Mat resizeImage;

    aruco::MarkerDetector MDetector;
    std::vector<aruco::Marker> markers;
    aruco::MarkerDetector::Params& params;

    CMPattern::TeamDetectorSettings * global_team_detector_settings;
    CMPattern::TeamSelector * global_team_selector_blue;
    CMPattern::TeamSelector * global_team_selector_yellow;

    double _blue_robot_height;
    double _yellow_robot_height;
protected slots:
    void teamDataChange();
public:
    PluginDetectRobotsArUco(FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field, CMPattern::TeamSelector * _global_team_selector_blue, CMPattern::TeamSelector * _global_team_selector_yellow, CMPattern::TeamDetectorSettings * global_team_settings);

    ~PluginDetectRobotsArUco();

    virtual ProcessResult process(FrameData * data, RenderOptions * options);
    virtual VarList * getSettings();
    virtual string getName();
private:
    static std::string toString(const aruco::DetectionMode& m){
        switch(m){
        case aruco::DM_NORMAL:
            return "NORMAL";
        case aruco::DM_FAST:
            return "FAST";
        case aruco::DM_VIDEO_FAST:
            return "VIDEO";
        default:
            return "FAST";
        }
    }
    static aruco::DetectionMode toDM(const char* s){
        if(strcmp(s,"FAST") == 0){
            return aruco::DM_FAST;
        }else if(strcmp(s,"NORMAL") == 0){
            return aruco::DM_NORMAL;
        }else if(strcmp(s,"VIDEO") == 0){
            return aruco::DM_VIDEO_FAST;
        }
        return aruco::DM_FAST;
    }
};

#endif // PLUGIN_DETECT_ROBOTS_USING_TAG_H
