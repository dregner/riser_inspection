//
// Created by vant3d on 16/12/2021.
//

#include <local_position_control.h>
#include <dji_control.hpp>

LocalController::LocalController() {
    subscribing(nh_);
}

LocalController::~LocalController() = default;

void LocalController::subscribing(ros::NodeHandle &nh) {
    std::string gps_topic, rtk_topic, attitude_topic, height_topic, local_pos_topic;

    //! Topic parameters
    nh.param("/riser_inspection/gps_topic", gps_topic, std::string("/dji_osdk_sdk/gps_position"));
    nh.param("/riser_inspection/rtk_topic", rtk_topic, std::string("/dji_osdk_sdk/rtk_position"));
    nh.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_osdk_sdk/attitude"));
    nh.param("/riser_inspection/height_takeoff", height_topic, std::string("/dji_osdk_sdk/height_above_takeoff"));
    nh.param("/riser_inspection/local_position_topic", local_pos_topic, std::string("/dji_osdk_sdk/local_position"));
    //! Subscribe topics
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &LocalController::gps_callback, this);
    rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic, 1, &LocalController::rtk_callback, this);
    attitude_sub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                  &LocalController::attitude_callback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(local_pos_topic, 1,
                                                              &LocalController::local_position_callback, this);
    height_sub = nh.subscribe<std_msgs::Float32>(height_topic, 1, &LocalController::height_callback, this);


    //! Service topics
    obtain_control_sdk = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
    set_local_pos_reference = nh.serviceClient<dji_osdk_ros::SetLocalPosRef>("/set_local_pos_ref");
    task_control_client = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("/flight_task_control");
    gimbal_control_client = nh.serviceClient<dji_osdk_ros::GimbalAction>("/gimbal_task_control");
    //! Camera services
    camera_action_service = nh.serviceClient<dji_osdk_ros::CameraAction>("camera_action");
    stereo_v3d_service = nh.serviceClient<stereo_vant::PointGray>("/stereo_point_grey/take_picture");
    ROS_INFO("Subscribed Complet");

    local_position_service = nh.advertiseService("/local_position/set_position",
                                                 &LocalController::local_pos_service_cb, this);

    start_mission_service = nh.advertiseService("/local_position/start_mission",
                                                &LocalController::start_mission_service_cb, this);
    horizontal_point_service = nh.advertiseService("/local_position/horizontal_point",
                                                   &LocalController::horizontal_pt_service_cb, this);


}

bool LocalController::obtain_control(bool ask) {
    dji_osdk_ros::ObtainControlAuthority authority;
    authority.request.enable_obtain = ask;
    obtain_control_sdk.call(authority);

    if (!authority.response.result) {
        ROS_ERROR("Do not understood");
        return authority.response.result;
    }
    ROS_INFO("SDK Controlling: %s", ask ? "True" : "False");
    return authority.response.result;
}


bool LocalController::local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                                           riser_inspection::LocalPosition::Response &res) {
    try {
        if (LocalController::obtain_control(true)) {
            ROS_INFO("Received points x: %f, y: %f, z: %f, yaw: %f", req.x, req.y, req.z, req.yaw);
            ROS_INFO("Threshold: %f m - %f deg", req.position_thresh, req.yaw_thresh);
            return res.result = local_position_ctrl(req.x, req.y, req.z, req.yaw, req.position_thresh, req.yaw_thresh);
        } else {
            ROS_ERROR("Did not get Control Authority");
            return res.result = false;
        }
    }
    catch (ros::Exception &e) {
        ROS_ERROR("ROS error %s", e.what());
        return res.result = false;
    }
}

bool LocalController::start_mission_service_cb(riser_inspection::StartMission::Request &req,
                                               riser_inspection::StartMission::Response &res) {

    doing_mission = true;
    use_gimbal = req.use_gimbal;
    h_error = req.position_thresh;
    yaw_error = req.yaw_thresh;
    use_stereo = req.use_stereo;

    if (use_stereo) {
        stereo_vant::PointGray stereoAction;
        stereo_voo++;
        stereoAction.request.reset_counter = true;
        stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_voo);
        stereoAction.request.file_name = "stereo_vant";
        stereo_v3d_service.call(stereoAction);
    } else {
        LocalController::set_gimbal_angles(0, 0, 0);
    }
    generate_WP(req.control_type);
    if (req.control_type == 4) {
        return res.result = LocalController::obtain_control(true);
    } else { return res.result = true; }
}

bool LocalController::horizontal_pt_service_cb(riser_inspection::hPoint::Request &req,
                                               riser_inspection::hPoint::Response &res) {
    ROS_INFO("Received new horizontal point number %i", req.number);
    int wp_number = req.number - 1;
    try {

        if (wp_number <= (int) waypoint_l.size() - 1) {
            if (LocalController::obtain_control(true)) {

                ROS_INFO("Set new H point [%f, %f, %f] @ %f", waypoint_l[wp_number][1], waypoint_l[wp_number][2],
                         rpa_height, waypoint_l[wp_number][3]);
                ROS_INFO("Threshold: %f m - %f deg", h_error, yaw_error);
                return res.result = local_position_ctrl(waypoint_l[wp_number][1], waypoint_l[wp_number][2],
                                                        rpa_height, waypoint_l[wp_number][3],
                                                        h_error, yaw_error);
            } else {
                ROS_ERROR("Did not get Control Authority");
                return res.result = false;
            }
        } else { doing_mission = false; }
    }
    catch (ros::Exception &e) {
        ROS_ERROR("ROS error %s", e.what());
        return res.result = false;
    }

}

void LocalController::height_callback(const std_msgs::Float32::ConstPtr &msg) {
    rpa_height = msg->data;
}

void LocalController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
}

void LocalController::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk = *msg;
}


void LocalController::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti = *msg;
    current_atti_euler.Set(current_atti.quaternion.w, current_atti.quaternion.x, current_atti.quaternion.y,
                           current_atti.quaternion.z);

}

void LocalController::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    current_local_pos = *msg;
    if (doing_mission) {
        local_position_ctrl_mission(wp_n);
    }
}


bool LocalController::set_local_position() {
    dji_osdk_ros::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

    return (bool) localPosReferenceSetter.response.result;
}

void LocalController::set_gimbal_angles(float roll, float pitch, float yaw) {
    dji_osdk_ros::GimbalAction gimbalAction;
    gimbalAction.request.is_reset = false;
    gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
    gimbalAction.request.rotationMode = 0;
    gimbalAction.request.pitch = pitch;
    gimbalAction.request.roll = roll;
    gimbalAction.request.yaw = yaw;
    gimbalAction.request.time = 0.5;
    gimbal_control_client.call(gimbalAction);

}

bool LocalController::local_position_ctrl(float xCmd, float yCmd, float zCmd, float yawCmd, float pos_thresh,
                                          float yaw_thresh) {
    dji_osdk_ros::FlightTaskControl control_task_point;
    control_task_point.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    control_task_point.request.joystickCommand.x = xCmd;
    control_task_point.request.joystickCommand.y = yCmd;
    control_task_point.request.joystickCommand.z = zCmd;
    control_task_point.request.joystickCommand.yaw = yawCmd;
    control_task_point.request.posThresholdInM = pos_thresh;
    control_task_point.request.yawThresholdInDeg = yaw_thresh;

    task_control_client.call(control_task_point);

    if (control_task_point.response.result) {
        ROS_INFO("(%f, %f, %f) m @ %f deg target complete",
                 current_local_pos.point.x, current_local_pos.point.y, rpa_height, RAD2DEG(current_atti_euler.Yaw()));
        LocalController::obtain_control(false);
        return control_task_point.response.result;
    } else { return control_task_point.response.result; }
}

void
LocalController::local_position_ctrl_mission(int waypoint_n) {
    dji_osdk_ros::FlightTaskControl control_task_mission;
    control_task_mission.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    control_task_mission.request.joystickCommand.x = waypoint_l[waypoint_n][1];
    control_task_mission.request.joystickCommand.y = waypoint_l[waypoint_n][2];
    control_task_mission.request.joystickCommand.z = waypoint_l[waypoint_n][3];
    control_task_mission.request.joystickCommand.yaw = waypoint_l[waypoint_n][4];
    control_task_mission.request.posThresholdInM = h_error;
    control_task_mission.request.yawThresholdInDeg = yaw_error;

    task_control_client.call(control_task_mission);

    if (control_task_mission.response.result) {
        ROS_INFO("WP %i - (%f, %f, %f) m @ %f deg target complete", wp_n,
                 current_local_pos.point.x, current_local_pos.point.y, rpa_height, RAD2DEG(current_atti_euler.Yaw()));
        if (waypoint_n >= (int) waypoint_l.size() - 1) {
            LocalController::obtain_control(false);
            doing_mission = false;
        } else {
            LocalController::acquire_photo(use_stereo, use_gimbal) ? ros::Duration(4).sleep() : ros::Duration(
                    1).sleep();
            wp_n++;
        }
    }
}

bool LocalController::acquire_photo(bool stereo, bool gimbal) {
    dji_osdk_ros::CameraAction cameraAction;
    stereo_vant::PointGray stereoAction;

    if (stereo) {
        stereoAction.request.reset_counter = false;
        stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_voo);
        stereoAction.request.file_name = "stereo_vant";
        stereo_v3d_service.call(stereoAction);
        if (stereoAction.response.result) {
            ROS_INFO("%Stereo image %i - %b", camera_count, stereoAction.response.result);
            camera_count++;
        }
    }
    if (gimbal) {
        cameraAction.request.camera_action = 0;
        camera_action_service.call(cameraAction);
        if (cameraAction.response.result) {
            ROS_INFO("Picture %i - %b", camera_count, cameraAction.response.result);
            camera_count++;
        }
    }
    if (cameraAction.response.result || stereoAction.response.result) {
        return true;
    } else { return false; }

}

bool LocalController::generate_WP(int csv_type) {
    /** Initial setting and parameters to generate trajectory*/
    pathGenerator.reset(); // clear pathGen
    waypoint_l.clear(); // clear waypoint list]
    wp_n = 0;
    int riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v;
    std::string root_directory;
    nh_.param("/riser_inspection/riser_distance", riser_distance, 10);
    nh_.param("/riser_inspection/riser_diameter", riser_diameter, 300);
    nh_.param("/riser_inspection/horizontal_points", h_points, 5);
    nh_.param("/riser_inspection/vertical_points", v_points, 8);
    nh_.param("/riser_inspection/delta_H", delta_h, 15);
    nh_.param("/riser_inspection/delta_V", delta_v, 300);
    nh_.param("/riser_inspection/root_directory", root_directory, std::string("/home/vant3d/Documents"));

    /** Setting intial parameters to create waypoints */
    pathGenerator.setFolderName(root_directory);
    pathGenerator.setInspectionParam(riser_distance, (float) riser_diameter, h_points, v_points, delta_h,
                                     (float) delta_v);
    /** Define start positions to create waypoints */
    pathGenerator.setInitCoord_XY(current_local_pos.point.x, current_local_pos.point.y, rpa_height,
                                  (int) RAD2DEG(current_atti_euler.Yaw()));
    try {
        pathGenerator.createInspectionPoints(csv_type); // type 4 refers to XYZ YAW waypoints
        ROS_WARN("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());

        std::vector<std::vector<std::string>> csv_file = pathGenerator.read_csv(
                pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
        //!Vector of XYZ and Yaw positions to generate a fully automated control
        if (csv_type == 4) {
            for (int k = 1; k < (int) csv_file.size(); k++) {
                waypoint_l.push_back(
                        {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                         2 * std::stof(csv_file[k][3]), 2 * std::stof(csv_file[k][4])});
            }
            waypoint_l.push_back(
                    {(float) csv_file.size(), (float) current_local_pos.point.x, (float) current_local_pos.point.y,
                     2 * rpa_height, 2 * (float) RAD2DEG(current_atti_euler.Yaw())});
            return true;

        }
        //! Vector of XY and Yaw positions to generate a semi-automated control, sending RPA to horizontal position and
        //! the SDK controller release to pilot change manually its vertical position
        if (csv_type == 3) {
            for (int k = 1; k < (int) csv_file.size(); k++) {
                waypoint_l.push_back(
                        {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                         2 * std::stof(csv_file[k][3])});
            }
            waypoint_l.push_back(
                    {(float) csv_file.size(), (float) current_local_pos.point.x, (float) current_local_pos.point.y,
                     2 * (float) RAD2DEG(current_atti_euler.Yaw())});
            ROS_INFO("Created %i horizontal points", h_points);
            ROS_INFO("To send RPA to the points use service /riser_inspection/horizontal_point");
            return true;
        }
    } catch (ros::Exception &e) {
        ROS_WARN("ROS error %s", e.what());
        return false;
    }
}

