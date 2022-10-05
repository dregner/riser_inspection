//
// Created by vant3d on 04/10/22.
//

#ifndef RISER_INSPECTION_DJI_CAMERA_SETTING_H
#define RISER_INSPECTION_DJI_CAMERA_SETTING_H

#include <ros/service_server.h>
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <riser_inspection/CameraSetting.h>


class DJIcameraSetting {
private:
    ros::NodeHandle nh_;
    ros::ServiceClient camera_set_EV_client;
    ros::ServiceClient camera_set_shutter_speed_client;
    ros::ServiceClient camera_set_aperture_client;
    ros::ServiceClient camera_set_iso_client;
    ros::ServiceClient camera_set_focus_point_client;

    ros::ServiceServer camera_setting_service;
public:
    DJIcameraSetting();

    ~DJIcameraSetting();

    void subscriber(ros::NodeHandle nh);

    bool camera_setting_service_server(riser_inspection::CameraSetting::Request &req, riser_inspection::CameraSetting::Response &res);
};


#endif //RISER_INSPECTION_DJI_CAMERA_SETTING_H
