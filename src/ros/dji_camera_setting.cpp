//
// Created by vant3d on 04/10/22.
//

#include "dji_camera_setting.h"

DJIcameraSetting::DJIcameraSetting() {
    DJIcameraSetting::subscriber(nh_);
}
DJIcameraSetting::~DJIcameraSetting() {

}

void DJIcameraSetting::subscriber(ros::NodeHandle nh) {

    camera_set_EV_client = nh.serviceClient<dji_osdk_ros::CameraEV>("camera_task_set_EV");
    camera_set_shutter_speed_client = nh.serviceClient<dji_osdk_ros::CameraShutterSpeed>("camera_task_set_shutter_speed");
    camera_set_aperture_client = nh.serviceClient<dji_osdk_ros::CameraAperture>("camera_task_set_aperture");
    camera_set_iso_client = nh.serviceClient<dji_osdk_ros::CameraISO>("camera_task_set_ISO");
    camera_set_focus_point_client = nh.serviceClient<dji_osdk_ros::CameraFocusPoint>("camera_task_set_focus_point");
}