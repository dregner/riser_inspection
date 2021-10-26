//
// Created by regner on 24/10/2021.
//
#include <path_generator.hh>

int main() {
    PathGenerate riser;
    std::cout << "Creating waypoint pathway" << std::endl;
    riser.setFolderName("/home/vant3d/Documents");
    riser.setInitCoord(5, 0.3, 46.775450, 8.345125, 1839.8 + 10, 74.2);
    riser.setInspectionParam(5, 4, 15, -0.3);
    riser.createInspectionPoints();

    std::cout << "Directory " << riser.getFolderName() << "/" << riser.getFileName() << std::endl;
    std::cout << "Finished" << std::endl;

    return 0;
}
