//
// Created by regner on 06/09/2021.
//
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <riser_inspection/WPgenerate.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <vector>


#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

class RiserInspection {
private:
    /// Arrays used to store waypoints and initial values
    float _waypointTaskDJI[8]; // Store DJI waypoint Task parameters
    double _cart_array[6]; // [x y z dz dy dz]' [6x1]
    double _coord_array[5]; // [x y z dz dy dz]' [6x1]

    /// Initial position to waypoint creates
    double _lat0 = -27.605299; // Starting latitude
    double _lon0 = -48.520547; // Starting longitude
    int _alt0 = 3; // Starting altitude
    int _head0 = 30; // Starting heading
    std::ofstream _saved_wp;
public:
    RiserInspection();

    ~RiserInspection();

    void setInitCoord(double lon, double lat, int alt, int head);

    void setDJIwaypointTask(float velocity_range, float idle_velocity, int action_on_finish,
                            int mission_exec_times, int yaw_mode, int trace_mode,
                            int action_on_rc_lost, int gimbal_pitch_mode);

    void get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps, const sensor_msgs::NavSatFixConstPtr &msg_rtk);

    /** @param wp_array Array to be ploted
        @param size Size of array to be ploted (normaly 5 or 6)
        @return printed waypoint at terminal*/
    void print_wp(double *wp_array, int size, int n);

    void csv_save_wp(double *wp_array, int row);

    /** @param cart_wp Array of 6 elements provided from cartesian [x y z dz dy dz]'
        @return coord_wp - Lat, lon, alt, roll (north heading) and pitch (gimbal) [Nx5] */

    void pointCartToCord(double cart_wp[6], int nCount);

    /** @param phi  Riser diameter
        @param d    Distance to riser wall
        @param da   Angle delta. Degrees
        @param nh   Number of acquisitions on same height
        @param dv   Delta height (altitude or Z)
        @param nv   Number of acquisitions levels
        @return Format: [x y z dz dy dz]' [6x1]  */

    bool createInspectionPoints(const double phi, const float d, const float da,
                                const float nh, const float dv, const float nv);

};