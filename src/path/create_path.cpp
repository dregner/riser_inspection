//
// Created by regner on 24/10/2021.
//
#include <path_generator.hh>

int main() {
    PathGenerate riser;
    std::cout << "Creating waypoint pathway" << std::endl;
    riser.setFolderName("/home/vant3d/Documents");
    riser.setInspectionParam(5, 300, 5, 4, 15, 300);
    riser.setInitCoord( 40, 8, 20, 0);

    riser.createInspectionPoints(1);

    std::cout << "Directory " << riser.getFolderName() << "/" << riser.getFileName() << std::endl;
    std::cout << "Finished" << std::endl;

    return 0;
}
