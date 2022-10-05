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

    camera_set_shutter_speed_client = nh.serviceClient<dji_osdk_ros::CameraShutterSpeed>(
            "camera_task_set_shutter_speed");
    camera_set_focus_point_client = nh.serviceClient<dji_osdk_ros::CameraFocusPoint>("camera_task_set_focus_point");

    camera_setting_service = nh.advertiseService("/riser_inspection/camera_setting",
                                                 &DJIcameraSetting::camera_setting_service_server, this);
}

bool DJIcameraSetting::camera_setting_service_server(riser_inspection::CameraSetting::Request &req,
                                                     riser_inspection::CameraSetting::Response &res) {
    ROS_INFO("Receive Parameters to set DJI Gimbal's camera");
    dji_osdk_ros::CameraShutterSpeed cameraShutterSpeed;
    dji_osdk_ros::CameraAperture cameraAperture;
    dji_osdk_ros::CameraEV cameraEv;
    dji_osdk_ros::CameraISO cameraIso;
    dji_osdk_ros::CameraFocusPoint cameraFocusPoint;

    cameraShutterSpeed.request.payload_index = req.payload_index;
    cameraShutterSpeed.request.exposure_mode = static_cast<uint8_t>(dji_osdk_ros::ExposureMode::PROGRAM_AUTO);
    cameraShutterSpeed.request.shutter_speed = req.shutter_speed;


    cameraFocusPoint.request.payload_index = req.payload_index;
    cameraFocusPoint.request.x = 0.5;
    cameraFocusPoint.request.y = 0.5;

    camera_set_focus_point_client.call(cameraFocusPoint);
    camera_set_shutter_speed_client.call(cameraShutterSpeed);
    if (cameraFocusPoint.response.result && cameraShutterSpeed.response.result) {
        res.result = true;
    } else { return res.result = false; }

    return res.result;


}