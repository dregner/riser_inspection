//
// Created by regner on 14/09/2021.
//

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <riser_inspection/PathGenAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <math.h>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))


class PathGenerator {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<riser_inspection::PathGenAction> as_;
    std::string action_name_;
    riser_inspection::PathGenFeedback feedback_;
    riser_inspection::PathGenResult result_;
    ros::Subscriber sub_GPS_;
    ros::Subscriber sub_obj_distance_;

    float lat0_, lon0_, alt0_, distance;
    float lat_, lon_, alt_;
    bool firstTime = true;


public:
    PathGenerator(std::string name) :
            as_(nh_, name, false),
            action_name_(name) {
        as_.registerGoalCallback(boost::bind(&PathGenerator::executeCB, this));
        as_.registerPreemptCallback(boost::bind(&PathGenerator::preemptCB, this));
        sub_GPS_ = nh_.subscribe("/dji_osdk_ros/GPS", 1, &PathGenerator::gps_position, this);
        sub_obj_distance_ = nh_.subscribe("/vant3d/object_distance", 1, &PathGenerator::object_distance, this);
        as_.start();
    }

    ~PathGenerator(void) {

    }

    void gps_position(sensor_msgs::NavSatFixConstPtr &msg) {
        if (firstTime) {
            lat0_ = msg->latitude;
            lon0_ = msg->longitude;
            alt0_ = msg->altitude;
            firstTime = false;
        } else {
            lat_ = msg->latitude;
            lon_ = msg->longitude;
            alt_ = msg->altitude;
        };
    }

    void object_distance(std_msgs::Float32ConstPtr &msg) {
        distance = msg->data;
    }

    void preemptCB() {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void executeCB(const riser_inspection::PathGenGoalConstPtr &goal) {
        ros::Rate(10);
        bool success = true;

        feedback_.latitude.clear();
        feedback_.longitude.clear();
        feedback_.altitude.clear();
        for (int i = 0; i < (goal->vertical_number * goal->horizontal_number); i++) {

            createInspectionPoints(goal->riser_diameter, distance, goal->delta_angle, goal->horizontal_number,
                                   goal->delta_height, goal->vertical_number);

            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
        }
        if (success) {
            result_.latitude = feedback_.latitude;
            result_.longitude = feedback_.longitude;
            result_.altitude = feedback_.altitude;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }

    }

    void pointCartToCord(double cart_wp[6], int nCount) {

        //[6371 km]. the approximate radius of earth
        long R = 6371 * 1000;



        // Convert starting coordinate (lat lon alt) to cartesian (XYZ)
        double x0 = R * DEG2RAD(lon0_) * cos(DEG2RAD(lon0_));
        double y0 = R * DEG2RAD(lat0_);
        double z0 = (_cart_array[2] * -1) + alt0_;


        double firstHeading = 0;

        double x = _cart_array[0] + x0;
        double y = _cart_array[1] + y0;
        double alt = _cart_array[2] + z0;


        double lon = RAD2DEG(x / (R * cos(DEG2RAD(lon0_))));
        double lat = RAD2DEG(y / R);

        _coord_array[0] = lat;
        _coord_array[1] = lon;
        _coord_array[2] = alt;

        double roll = DEG2RAD(atan2(_cart_array[4], _cart_array[3]));

        double heading = roll - 180 - 90;

        if (nCount == 0) {
            firstHeading = heading;
        }

        double pitch = DEG2RAD(asin(_cart_array[5]));

        heading = (heading - firstHeading + _head0);

        if (heading < -180) {
            heading = heading + 360;
        } else if (heading > 180) {
            heading = heading - 360;
        }
//
        _coord_array[3] = heading;
        _coord_array[4] = pitch;
    }

    void createInspectionPoints(const double phi, const float d, const float da, const float nh,
                                const float dv, const float nv) {
        // Inspection radius.
        double r = d + phi / 2;

        int nCount = 0;
        int n = (int) (nh * nv);
        float CartP[6][n]; // Cartesian matrix points
        float CordP[5][n]; // Coordenate matrix points
        float p[3] = {0, 0, 0};

        // Height change
        float hc = ((nv - 1) * dv);

        //Path control
        bool isVertical = true;

        // Position [XY] - Horizontal levels

        for (int i = 0; i <= nh - 1; i++) {
            // Update Alpha
            float alpha = i * da;
            float hs;
            float hl;

            //Controls vertical path
            if (isVertical) {
                hl = 0; // Initial height
                hs = dv; // Height step
            } else {
                hl = hc; // Initial height
                hs = dv * (-1); // Initial step
            }

            for (int j = 0; j <= nv - 1; j++) {
                float z = j * hs + hl;

                // Lines below set position
                float x = r * cos(DEG2RAD(alpha));
                float y = r * sin(DEG2RAD(alpha));
                _cart_array[0] = x;
                _cart_array[1] = y;
//            _cart_array[2] = z;


                // Orientation
                float endPoint[3] = {0, 0, z};

                float dr[3] = {endPoint[0] - x, endPoint[1] - y, endPoint[2] - z};

                // Calculation of the absolute value of the array
                float absolute = sqrt((dr[0] * dr[0]) + (dr[1] * dr[1]) + (dr[2] * dr[2]));

                _cart_array[3] = dr[0] / absolute;
                _cart_array[4] = dr[1] / absolute;
                _cart_array[5] = dr[2] / absolute;
                _cart_array[2] = z + alt0_;

                RiserInspection::pointCartToCord(_cart_array, nCount);

                for (int i = 0; i < (sizeof(CartP) / sizeof(CartP[0])); i++) {
                    if (i < (sizeof(_cart_array) / sizeof(_cart_array[0]))) {
                        CartP[i][nCount] = _cart_array[i];
                    }
                    if (i < (sizeof(_coord_array) / sizeof(_coord_array[0]))) {
                        CordP[i][nCount] = _coord_array[i];
                    }
                }
//            print_wp(_cart_array, sizeof(_cart_array) / sizeof(_cart_array[0]), nCount);
//            print_wp(_coord_array, sizeof(_coord_array) / sizeof(_coord_array[0]), nCount);
//            csv_wp(_coord_array, sizeof(_coord_array) / sizeof(_coord_array[0]));
                csv_save_wp(_cart_array, sizeof(_cart_array) / sizeof(_cart_array[0]));
                nCount++;
            }
            isVertical = !isVertical;
        }
    }
};