#include <iostream>
#include <path_generator.hh>

int main() {
    RiserInspection riser;
    std::cout << "Create waypoint pathway" << std::endl;

    riser.setInitCoord(-48.520547, -27.605299, 10, 30);
    riser.createInspectionPoints(0.3, 5, 15, 5, -0.3, 10);

    return 0;
}
