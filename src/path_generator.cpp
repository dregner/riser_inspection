#include <path_generator.hh>


RiserInspection::RiserInspection() {}

RiserInspection::~RiserInspection() {}

void RiserInspection::setInitCoord(double lon, double lat, int alt, int head) {
    lon0 = lon;
    lat0 = lat;
    alt0 = alt;
    head0 = head;
}

int RiserInspection::get_alt() {
    return alt0;
}

int RiserInspection::get_head() {
    return head0;
}

double RiserInspection::get_lon() {
    return lon0;
}

double RiserInspection::get_lat() {
    return lat0;
}

void RiserInspection::pointCartToCord(double cart_wp[6], int nCount) {

    //[6371 km]. the approximate radius of earth
    long R = 6371 * 1000;



    // Convert starting coordinate (lat lon alt) to cartesian (XYZ)
    double x0 = R * DEG2RAD(lon0) * cos(DEG2RAD(lon0));
    double y0 = R * DEG2RAD(lat0);
    double z0 = (cart_array[2] * -1) + alt0;

//    matrix = RiserInspection::transposeXyZ(matrix, cord);

    double firstHeading = 0;

    double x = cart_array[0] + x0;
    double y = cart_array[1] + y0;
    double alt = cart_array[2] + z0;


    double lon = RAD2DEG(x / (R * cos(DEG2RAD(lon0))));
    double lat = RAD2DEG(y / R);

    coord_array[0] = lat;
    coord_array[1] = lon;
    coord_array[2] = alt;

    double roll = DEG2RAD(atan2(cart_array[4], cart_array[3]));

    double heading = roll - 180 - 90;

    if (nCount == 0) {
        firstHeading = heading;
    }

    double pitch = DEG2RAD(asin(cart_array[5]));

    heading = (heading - firstHeading + head0);

    if (heading < -180) {
        heading = heading + 360;
    } else if (heading > 180) {
        heading = heading - 360;
    }
//
    coord_array[3] = heading;
    coord_array[4] = pitch;
}


void RiserInspection::createInspectionPoints(double phi, float d, float da, float nh, float dv, float nv) {
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
            cart_array[0] = x;
            cart_array[1] = y;
//            cart_array[2] = z;


            // Orientation
            float endPoint[3] = {0, 0, z};

            //TODO: Verificar com o Pedro a sutração dos arrays (matrizes)
            float dr[3] = {endPoint[0] - x, endPoint[1] - y, endPoint[2] - z};

            // Calculation of the absolute value of the array
            float absolute = sqrt((dr[0] * dr[0]) + (dr[1] * dr[1]) + (dr[2] * dr[2]));

            cart_array[3] = dr[0] / absolute;
            cart_array[4] = dr[1] / absolute;
            cart_array[5] = dr[2] / absolute;
            cart_array[2] = z + alt0;

            RiserInspection::pointCartToCord(cart_array, nCount);

            for (int i = 0; i < (sizeof(CartP) / sizeof(CartP[0])); i++) {
                if (i < (sizeof(cart_array) / sizeof(cart_array[0]))) {
                    CartP[i][nCount] = cart_array[i];
                }
                if (i < (sizeof(coord_array) / sizeof(coord_array[0]))) {
                    CordP[i][nCount] = coord_array[i];
                }
            }
            print_wp(cart_array, sizeof(cart_array) / sizeof(cart_array[0]), nCount);
            print_wp(coord_array, sizeof(coord_array) / sizeof(coord_array[0]), nCount);

            nCount++;
        }
        isVertical = !isVertical;
    }
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

