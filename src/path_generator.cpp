#include <path_generator.hh>


RiserInspection::RiserInspection() {
    _saved_wp.open("/home/regner/Documents/wp_generator.csv");
//    initServices(nh_);
    initSubscriber(nh_);
}

RiserInspection::~RiserInspection() {
    _saved_wp.close();
}

void RiserInspection::setInitCoord(double lon, double lat, int alt, int head) {
    _lon0 = lon;
    _lat0 = lat;
    _alt0 = alt;
    _head0 = head;
}

void RiserInspection::setDJIwaypointTask(float velocity_range, float idle_velocity, int action_on_finish,
                                         int mission_exec_times, int yaw_mode, int trace_mode,
                                         int action_on_rc_lost, int gimbal_pitch_mode) {
    _waypointTaskDJI[0] = velocity_range;
    _waypointTaskDJI[1] = idle_velocity;
    _waypointTaskDJI[2] = action_on_finish;
    _waypointTaskDJI[3] = mission_exec_times;
    _waypointTaskDJI[4] = yaw_mode;
    _waypointTaskDJI[5] = trace_mode;
    _waypointTaskDJI[6] = action_on_rc_lost;
    _waypointTaskDJI[7] = gimbal_pitch_mode;

}

//void RiserInspection::initServices(ros::NodeHandle &nh) {
//    try {
//        generate_pathway_srv = nh_.advertiseService("riser_inspection/waypoint_generator", &RiserInspection::createInspectionPoints, this);
//        ROS_INFO("Service riser_inspection/waypoint_generator" initialize");
//    } catch (ros::Exception &e) {
//        ROS_ERROR("Subscribe topics exception: %s", e.what());
//    }
//}


void RiserInspection::initSubscriber(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string left_topic_, right_topic_, gps_topic_, rtk_topic_, imu_topic_;
        nh_private.param("gps_topic", gps_topic_, std::string("/dji_sdk/gps_position"));
        nh_private.param("rtk_topic", rtk_topic_, std::string("/dji_sdk/rtk_position"));
        nh_private.param("imu_topic", imu_topic_, std::string("/dji_sdk/imu"));

        ROS_INFO("Subscriber in Camera Right Topic: %s", right_topic_.c_str());
        gps_position_sub_.subscribe(nh, gps_topic_, 1);
        ROS_INFO("Subscriber in Camera GPS Topic: %s", gps_topic_.c_str());
        rtk_position_sub_.subscribe(nh, rtk_topic_, 1);
        ROS_INFO("Subscriber in Camera RTK Topic: %s", rtk_topic_.c_str());

        sync_.reset(new Sync(RiserInspectionPolicy(10), gps_position_sub_, rtk_position_sub_));
        sync_->registerCallback(boost::bind(&RiserInspection::get_gps_position, this, _1, _2));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void RiserInspection::get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps,
                                       const sensor_msgs::NavSatFixConstPtr &msg_rtk) {
    ptr_gps_position_ = msg_gps;
    ptr_rtk_position_ = msg_rtk;
}


void RiserInspection::print_wp(double *wp_array, int size, int n) {
    char const *cartesian[6] = {"x", "y", "z", "dz", "dy", "dz"};
    char const *gns_cord[5] = {"lat", "lon", "alt", "roll", "pitch"};
    std::cout << "Waypoint - " << n << std::endl;
    for (int i = 0; i < size; i++) {
        if (size > 5) { std::cout << cartesian[i] << "\t" << wp_array[i] << std::endl; }
        else { std::cout << gns_cord[i] << "\t" << wp_array[i] << std::endl; }
    }
}

void RiserInspection::csv_save_wp(double *wp_array, int row) {
    for (int i = 0; i < row; i++) {
        if (_saved_wp.is_open()) {
            if (i != row - 1) { _saved_wp << wp_array[i] << ", "; }
            else { _saved_wp << wp_array[i] << "\n"; }
        }
    }
}

void RiserInspection::pointCartToCord(double cart_wp[6], int nCount) {

    //[6371 km]. the approximate radius of earth
    long R = 6371 * 1000;



    // Convert starting coordinate (lat lon alt) to cartesian (XYZ)
    double x0 = R * DEG2RAD(_lon0) * cos(DEG2RAD(_lon0));
    double y0 = R * DEG2RAD(_lat0);
    double z0 = (_cart_array[2] * -1) + _alt0;


    double firstHeading = 0;

    double x = _cart_array[0] + x0;
    double y = _cart_array[1] + y0;
    double alt = _cart_array[2] + z0;


    double lon = RAD2DEG(x / (R * cos(DEG2RAD(_lon0))));
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

void RiserInspection::createInspectionPoints(const double phi, const float d, const float da, const float nh,
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

            //TODO: Verificar com o Pedro a sutração dos arrays (matrizes)
            float dr[3] = {endPoint[0] - x, endPoint[1] - y, endPoint[2] - z};

            // Calculation of the absolute value of the array
            float absolute = sqrt((dr[0] * dr[0]) + (dr[1] * dr[1]) + (dr[2] * dr[2]));

            _cart_array[3] = dr[0] / absolute;
            _cart_array[4] = dr[1] / absolute;
            _cart_array[5] = dr[2] / absolute;
            _cart_array[2] = z + _alt0;

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
//            csv_wp(_coord_array, sizeof(_coord_array) / sizeof(_coord_array[0]), nCount);
            csv_save_wp(_cart_array, sizeof(_cart_array) / sizeof(_cart_array[0]), nCount);
            nCount++;
        }
        isVertical = !isVertical;
    }
}
bool RiserInspection::serviceCreatePointsCB(int &req, int &res) {}


