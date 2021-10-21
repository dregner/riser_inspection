#include <path_generator.hh>


RiserInspection::RiserInspection() {
    saved_wp_.open("/home/regner/Documents/wp_generator.csv");
    saved_wp_ << "Latitude,Longitude,AltitudeAMSL,Speed,Picture,WP,CameraTilt,UavYaw,WaitTime" << std::endl;

}

RiserInspection::~RiserInspection() {
    saved_wp_.close();
}

void RiserInspection::setInitCoord(double dist, float d_cyl, double lon, double lat, float alt, float head) {
    d_cyl_ = d_cyl;
    dist_ = dist;
    lon0_ = lon;
    lat0_ = lat;
    alt0_ = alt;
    head0_ = head;
}

void RiserInspection::inspectionAngletoHeading(float polar_angle) {

    float heading = -polar_angle + head0_;
    /// Conditions to keep range - 180 to 180
    if (heading < -180) { heading = heading - 360; }
    if (heading > 180) { heading = heading + 360; }
    waypoint_[3] = heading;
}

void RiserInspection::polar2cart(double r, double alpha) {
    xy_array_[0] = r * cos(DEG2RAD(alpha));
    xy_array_[1] = r * sin(DEG2RAD(alpha));

}

void RiserInspection::cart2gcs(double altitude) {
    int R = 6371000; // Earth radius
    double x0 = R * DEG2RAD(lon0_) * cos(DEG2RAD(lon0_)); // Initial Position X
    double y0 = R * DEG2RAD(lat0_); // Initial Position Y
    double x = xy_array_[0] + x0;
    double y = xy_array_[1] + y0;
    waypoint_[0] = RAD2DEG(y / R);
    waypoint_[1] = RAD2DEG(x / (R * cos(DEG2RAD(lon0_))));
    waypoint_[2] = altitude;
}

void RiserInspection::csv_save_ugcs(double *wp_array, int row, int wp_number) {

    if (saved_wp_.is_open()) {
        saved_wp_ << std::setprecision(10) << wp_array[0] << ", "<< std::setprecision(10) << wp_array[1] << ", ";
        saved_wp_ << std::setprecision(10) << wp_array[2] << ", "<< 1 << ", TRUE, " << wp_number  << ", " << 0 << ", ";
        saved_wp_ << std::setprecision(10) << wp_array[3] << ", " << 2 << "\n";
    }
}

void RiserInspection::csv_save_wp(double *wp_array, int row) {
    for (int i = 0; i < row; i++) {
        if (saved_wp_.is_open()) {
            if (i != row - 1) { saved_wp_ << std::setprecision(10) << wp_array[i] << ", "; }
            else { saved_wp_ << std::setprecision(10) << wp_array[i] << "\n"; }
        }
    }
}


void RiserInspection::findCenterHeading(int deltaAngle, int angleCount) {
    if (angleCount % 2 == 1) { start_angle_ = (-deltaAngle * angleCount / 2) + deltaAngle / 2; }
    else { start_angle_ = (int) (-angleCount * round(angleCount / 2)); }
}

void RiserInspection::createInspectionPoints() {
    findCenterHeading(deltaAngle_, angleCount_);
    int count_wp = 1;
    float initial = alt0_; // initiate altitude value
    float vertical; // Set if will move drone down or up INITIALLY DOWN.
    for (int i = 0; i < angleCount_; i++) {
        if (i % 2 == 1) { vertical = -1; } else { vertical = 1; }
        for (int k = 0; k < altitudeCount_; k++) {
            /// Set altitude of this waypoint
            float altitude = initial + (float) k * vertical * deltaAltitude_;
            /// Set Polar values
            polar_array_[0] = dist_ + d_cyl_ / 2; // distance riser and drone
            polar_array_[1] = start_angle_ + i * deltaAngle_; // angle of inspection r^angle (Polar)
            polar_array_[1] += head0_ - 90; // Compense heading orientation and -90 to transform N to 0 deg
            /// Convert Polar to Cartesian
            polar2cart(polar_array_[0], polar_array_[1]);
            /// Introduce values to waypoint array to be printed
            cart2gcs(altitude);
            inspectionAngletoHeading((float) polar_array_[1]);
            /// Export to CSV file
//            csv_save_wp(waypoint_, sizeof(waypoint_) / sizeof(waypoint_[0]));
            csv_save_ugcs(waypoint_, sizeof(waypoint_) / sizeof(waypoint_[0]), count_wp);
            if (k == altitudeCount_ -
                     1) { initial = altitude; } // set initial altitude value when finished de vertical movement.
            count_wp += 1;
        }
    }
}

int main() {
    RiserInspection riser;
    std::cout << "Creating waypoint pathway" << std::endl;

    riser.setInitCoord(5, 0.3, -48.520547, -27.605299, 10, 30);
    riser.createInspectionPoints();
    std::cout << "Finished" << std::endl;

    return 0;
}
