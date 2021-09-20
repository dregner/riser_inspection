//
// Created by regner on 20/09/2021.
//

#ifndef RISER_INSPECTION_MISSION_NODE_HH
#define RISER_INSPECTION_MISSION_NODE_HH

// System includes
#include "unistd.h"
#include <iostream>
#include <fstream>

// DJI SDK includes
#include <dji_osdk_ros/MissionWpAction.h>
#include <dji_osdk_ros/MissionHpAction.h>
#include <dji_osdk_ros/MissionWpUpload.h>
#include <dji_osdk_ros/MissionHpUpload.h>
#include <dji_osdk_ros/MissionHpUpdateRadius.h>
#include <dji_osdk_ros/MissionHpUpdateYawRate.h>

#include <dji_osdk_ros/dji_vehicle_node.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

typedef struct ServiceAck {
    bool result;
    int cmd_set;
    int cmd_id;
    unsigned int ack_data;

    ServiceAck(bool res, int set, int id, unsigned int ack)
            : result(res), cmd_set(set), cmd_id(id), ack_data(ack) {
    }

    ServiceAck() {
    }
} ServiceAck;

bool runWaypointMission(uint8_t numWaypoints, int responseTimeout);

void setWaypointDefaults(DJI::OSDK::WayPointSettings *wp);

void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask &waypointTask);

std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(int numWaypoints, DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> importWaypoints(DJI::OSDK::WayPointSettings *start_data, int num_wp);

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                     int responseTimeout,
                     dji_osdk_ros::MissionWaypointTask &waypointTask);

ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask &waypointTask);

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION action);

ServiceAck activate();

ServiceAck obtainCtrlAuthority();

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

#endif //RISER_INSPECTION_MISSION_NODE_HH
