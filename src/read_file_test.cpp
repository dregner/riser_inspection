//
// Created by vant3d on 25/10/2021.
//
#include <path_generator.hh>

int main() {
    PathGenerate riser;
    std::cout << "Creating waypoint pathway" << std::endl;
    riser.setFolderName("/home/vant3d/Documents");
    riser.setFileName("teste.csv");
    riser.setInitCoord(40, 10, 10, -180);
    riser.setInspectionParam(5, 300, 5, 4, 15, -300);
    riser.createInspectionPoints(1);

    std::cout << "Directory " << riser.getFolderName() << riser.getFileName() << std::endl;
    std::cout << "Finished" << std::endl;
    std::vector<std::vector<std::string>> dataList = riser.read_csv(
            riser.getFolderName() + "/" + riser.getFileName(), ",");
    // Print the content of row by row on screen
    for (int i = 0; i < dataList.size(); i++) {
        for (int j = 0; j < dataList[0].size(); j++) {
            if (j != dataList[0].size() - 1) { std::cout << dataList[i][j] << " ,\t "; }
            else { std::cout << dataList[i][j] << std::endl; }
            if (dataList[i][j] == "TRUE") {
                dataList[i][j] = std::to_string(1);
            } else {
                dataList[i][j] = std::to_string(0);
            }
        }
    }

std::cout << "PASSED" <<
std::endl;

return 0;
}