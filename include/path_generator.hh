//
// Created by regner on 06/09/2021.
//

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

    double cart_array[6]; /// Format: [x y z dz dy dz]' [6x1]
    double coord_array[5]; /// /// Format: [x y z dz dy dz]' [6x1]
    double lat0 = -27.605299; /// @param lat0 Starting latitude
    double lon0 = -48.520547; /// @param lon0 Starting longitude
    int alt0 = 3; /// @param alt0 Starting altitude
    int head0 = 30; /// @param head0 Starting heading
public:
    RiserInspection();

    ~RiserInspection();

    void setInitCoord(double lon, double lat, int alt, int head);

    double get_lon();

    double get_lat();

    int get_alt();

    int get_head();

    void print_wp(double *wp_array, int size, int n);

    /** @param cart_wp Array of 6 elements provided from cartesian [x y z dz dy dz]'
      * @return coord_wp - Lat, lon, alt, roll (north heading) and pitch (gimbal) [Nx5] */

    void pointCartToCord(double cart_wp[6], int nCount);

    /** @param phi  Riser diameter
      * @param d    Distance to riser wall
      * @param da   Angle delta. Degrees
      * @param nh   Number of acquisitions on same height
      * @param dv   Delta height (altitude or Z)
      * @param nv   Number of acquisitions levels
      * @return Format: [x y z dz dy dz]' [6x1]  */

    void createInspectionPoints(double phi, float d, float da, float nh, float dv, float nv);

};
