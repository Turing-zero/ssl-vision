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
  \file    plugin_detect_robots_using_tag.cpp
  \brief   C++ Interface: plugin_detect_robots_using_tag
  \author  Mark ZJUNlict, 2022
*/
//========================================================================
#include "plugin_detect_robots_using_tag.h"
#include <chrono>
using namespace std::chrono;
PluginDetectRobotsArUco::PluginDetectRobotsArUco(FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field, CMPattern::TeamSelector * _global_team_selector_blue, CMPattern::TeamSelector * _global_team_selector_yellow, CMPattern::TeamDetectorSettings * _global_team_settings)
 : VisionPlugin(_buffer), camera_parameters(camera_params), field(field)
#ifdef USE_CV_ARUCO
#else
 , params(MDetector.getParameters())
#endif
{
#ifdef USE_CV_ARUCO
#else
    MDetector.setDictionary("ARUCO_MIP_16h3");
    MDetector.setDetectionMode(aruco::DM_FAST,0.02);
    params.setCornerRefinementMethod(aruco::CORNER_NONE);
    params.detectEnclosedMarkers(false);
#endif
    global_team_selector_blue=_global_team_selector_blue;
    global_team_selector_yellow=_global_team_selector_yellow;
    global_team_detector_settings=_global_team_settings;

    _settings=new VarList("Robot Tag");
    _settings->addChild(_shrink_ratio = new VarDouble("Shrink Ratio",1.0,1.0,10.0));
    _settings->addChild(_min_marker_ratio = new VarDouble("Marker4Image ratio",0.0,0.0,1.0));
    _settings->addChild(_detection_mode = new VarStringEnum("Marker Detect Mode",toString(aruco::DM_FAST)));
    _detection_mode->addItem(toString(aruco::DM_FAST));
    _notifier.addRecursive(_settings);
    connect(_global_team_selector_blue,SIGNAL(signalTeamDataChanged()),&_notifier,SLOT(changeSlotOtherChange()));
    connect(_global_team_selector_yellow,SIGNAL(signalTeamDataChanged()),&_notifier,SLOT(changeSlotOtherChange()));
    connect(_global_team_settings,SIGNAL(signalTeamDataChanged()),&_notifier,SLOT(changeSlotOtherChange()));
}

PluginDetectRobotsArUco::~PluginDetectRobotsArUco()
{
}


VarList * PluginDetectRobotsArUco::getSettings() {
  return _settings;
}

string PluginDetectRobotsArUco::getName() {
  return "DetectRobots";
}

ProcessResult PluginDetectRobotsArUco::process(FrameData * data, RenderOptions * options)
{
    auto start = high_resolution_clock::now();
    (void)options;
    if (data==0) return ProcessingFailed;

    SSL_DetectionFrame * detection_frame = 0;

    detection_frame=(SSL_DetectionFrame *)data->map.get("ssl_detection_frame");
    if (detection_frame == 0) detection_frame=(SSL_DetectionFrame *)data->map.insert("ssl_detection_frame",new SSL_DetectionFrame());

    detection_frame->clear_robots_blue();
    detection_frame->clear_robots_yellow();

    double shrink_ratio = _shrink_ratio->getDouble();
    int width = int(data->video.getWidth()/shrink_ratio);
    int height = int(data->video.getHeight()/shrink_ratio);

    if(data->video.getColorFormat() == COLOR_RGB8){
        inputImage = cv::Mat(cv::Size(data->video.getWidth(),data->video.getHeight()),CV_8UC3,(void*)(data->video.getData()));
        cv::cvtColor(inputImage,resizeImage,cv::COLOR_BGR2GRAY);
        cv::resize(resizeImage,resizeImage,cv::Size(width,height),cv::INTER_LINEAR);
    }else{
        printf("RawData format not support %d \n",data->video.getColorFormat());
        return ProcessingFailed;
    }

    TagResults * res = (TagResults *) data->map.get("tag_result");
    if (res == nullptr) {
        res = (TagResults *) data->map.insert("tag_result", new TagResults());
    }
    auto markers = MDetector.detect(resizeImage);
    bool need_reinit=_notifier.hasChanged();
    if(need_reinit){
        if(global_team_selector_blue->getSelectedTeam() ==0 || global_team_selector_yellow->getSelectedTeam() ==0){
            _notifier.changeSlotOtherChange();
        }else{
            _blue_robot_height = global_team_selector_blue->getSelectedTeam()->_robot_height->getDouble();
            _yellow_robot_height = global_team_selector_yellow->getSelectedTeam()->_robot_height->getDouble();
        }
    }
    res->markerCenters.clear();
    res->markerFaceCenters.clear();
    res->markerIds.clear();
    for(auto& marker : markers){
        if(marker.size()!=4){
            std::cerr << "marker corner size not correct" << std::endl;
        }
        int tag_id = marker.id;
        res->markerIds.push_back(tag_id);
        int robot_id = tag_id/2;
        //team_id: 0==blue, 1==yellow
        int team_id = tag_id%2;
        auto _robot_height = team_id==0?_blue_robot_height:_yellow_robot_height;
        auto ps = marker;
        vector3d p_3d;
        vector2d p_img;
        vector2d cen(0,0);
        vector2d img_cen(0,0);
        vector2d img_face_cen(0,0);
        vector2d face_cen(0,0);
        for(auto corner_i=0u;corner_i<4;corner_i++){
            p_img.set(shrink_ratio*ps[corner_i].x,shrink_ratio*ps[corner_i].y);
            camera_parameters.image2field(p_3d,p_img,_robot_height);
            img_cen.x += p_img.x/4.0;
            img_cen.y += p_img.y/4.0;
            cen.x += p_3d.x/4.0;
            cen.y += p_3d.y/4.0;
            // get face center for center of top two corners
            if(corner_i == 1) {
                face_cen = cen*2;
                img_face_cen = img_cen*2;
            }
        }
        res->markerCenters.push_back(img_cen);
        res->markerFaceCenters.push_back(img_face_cen);
        auto robot = team_id == 0 ? detection_frame->add_robots_blue() : detection_frame->add_robots_yellow();
        robot->set_robot_id(robot_id);
        robot->set_x(cen.x);
        robot->set_y(cen.y);
        robot->set_height(_robot_height);
        robot->set_orientation((face_cen-cen).angle());
        robot->set_pixel_x(img_cen.x);
        robot->set_pixel_y(img_cen.y);
        robot->set_confidence(1.0);
    }
    auto stop = high_resolution_clock::now();
//    std::cout << "duration1 : " << duration_cast<milliseconds>(stop - start).count() << "ms" << std::endl;
    return ProcessingOk;
}
