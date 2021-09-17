#include <path_generator.hh>


RiserInspection::RiserInspection() {
    _saved_wp.open("/home/regner/Documents/wp_generator.csv");
}

RiserInspection::~RiserInspection() {
    _saved_wp.close();
}



void RiserInspection::setInitCoord(double lon, double lat, int alt, int head) {
    lon0_ = lon;
    lat0_ = lat;
    alt0_ = alt;
    head0_ = head;
}

void RiserInspection::setDJIwaypointTask(float velocity_range, float idle_velocity, int action_on_finish,
                                         int mission_exec_times, int yaw_mode, int trace_mode,
                                         int action_on_rc_lost, int gimbal_pitch_mode) {
    waypointTaskDJI_[0] = velocity_range;
    waypointTaskDJI_[1] = idle_velocity;
    waypointTaskDJI_[2] = action_on_finish;
    waypointTaskDJI_[3] = mission_exec_times;
    waypointTaskDJI_[4] = yaw_mode;
    waypointTaskDJI_[5] = trace_mode;
    waypointTaskDJI_[6] = action_on_rc_lost;
    waypointTaskDJI_[7] = gimbal_pitch_mode;

}

void RiserInspection::rotz_cartP(int yaw) {

    rotz[0][0] = (int) cos(DEG2RAD(yaw));
    rotz[0][1] = (int) (sin(DEG2RAD(yaw)) * -1);
    rotz[0][2] = 0;

    rotz[1][0] = (int) sin(DEG2RAD(yaw));
    rotz[1][1] = (int) cos(DEG2RAD(yaw));
    rotz[1][2] = 0;

    rotz[2][0] = 0;
    rotz[2][1] = 0;
    rotz[2][2] = 1;
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
            if (i != row - 1) { _saved_wp << std::setprecision(10) << wp_array[i] << ", "; }
            else { _saved_wp << std::setprecision(10) << wp_array[i] << "\n"; }
        }
    }
}

void RiserInspection::pointCartToCord(double cart_wp[6], int nCount) {

    //[6371 km]. the approximate radius of earth
    long R = 6371 * 1000;



    // Convert starting coordinate (lat lon alt) to cartesian (XYZ)
    double x0 = R * DEG2RAD(lon0_) * cos(DEG2RAD(lon0_));
    double y0 = R * DEG2RAD(lat0_);
    double z0 = (cart_array_[2] * -1) + alt0_;


    double firstHeading = 0;

    double x = cart_array_[0] + x0;
    double y = cart_array_[1] + y0;
    double alt = cart_array_[2] + z0;


    double lon = RAD2DEG(x / (R * cos(DEG2RAD(lon0_))));
    double lat = RAD2DEG(y / R);

    coord_array_[0] = lat;
    coord_array_[1] = lon;
    coord_array_[2] = alt;


    double roll = RAD2DEG(atan2(cart_wp[4], cart_wp[3]));

    double heading = roll - 180 - 90;

    if (nCount == 0) {
        firstHeading = heading;
    }

    double pitch = DEG2RAD(asin(cart_array_[5]));

    heading = (heading - firstHeading + head0_);

    if (heading < -180) {
        heading = heading + 360;
    } else if (heading > 180) {
        heading = heading - 360;
    }
//
    coord_array_[3] = heading;
    coord_array_[4] = pitch;
}

void RiserInspection::createInspectionPoints(const double phi, const float d, const float da, const float nh,
                                             const float dv, const float nv) {
    // Inspection radius.
    double r = d + phi / 2;

    int nCount = 0;
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
            p[0] = x;
            p[1] = y;
            p[2] = z;

            // Orientation
            float endPoint[3] = {0, 0, z};

            //TODO: Verificar com o Pedro a sutração dos arrays (matrizes)
            double dr[3] = {endPoint[0] - x, endPoint[1] - y, endPoint[2] - z};

            // Calculation of the absolute value of the array
            float absolute = sqrt((dr[0] * dr[0]) + (dr[1] * dr[1]) + (dr[2] * dr[2]));

            dr[0] = dr[0] / absolute;
            dr[1] = dr[1] / absolute;
            dr[2] = dr[2] / absolute;

            RiserInspection::rotz_cartP(180);

            for (int j = 0; j < sizeof(rotz) / sizeof(rotz[0]); j++) { // j from 0 to 2
                double var_p = 0, var_dr =0;
                for (int k = 0; k < sizeof(rotz[0]) / sizeof(int); k++) { // k from 0 to 2
                    var_p += (rotz[j][k] * p[j]);
                    var_dr += (rotz[j][k] * dr[j]);
                    if(k == 2){
                        p[j] = var_p;
                        dr[j] = var_dr;}
                }
            }

            cart_array_[0] = p[0];
            cart_array_[1] = p[1];
            cart_array_[2] = p[2]+alt0_;
            cart_array_[3] = dr[0];
            cart_array_[4] = dr[1];
            cart_array_[5] = dr[2];
            RiserInspection::pointCartToCord(cart_array_, nCount);

//            print_wp(cart_array_, sizeof(cart_array_) / sizeof(cart_array_[0]), nCount);
//            print_wp(coord_array_, sizeof(coord_array_) / sizeof(coord_array_[0]), nCount);
            csv_save_wp(coord_array_, sizeof(coord_array_) / sizeof(coord_array_[0]));
//            csv_save_wp(cart_array_, sizeof(cart_array_) / sizeof(cart_array_[0]));
            nCount++;
        }
        isVertical = !isVertical;
    }
}


int main() {
    RiserInspection riser;
    std::cout << "Create waypoint pathway" << std::endl;

    riser.setInitCoord(-48.520547, -27.605299, 10, 30);
    riser.createInspectionPoints(0.3, 5, 15, 5, -0.3, 10);

    return 0;
}
