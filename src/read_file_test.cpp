//
// Created by vant3d on 25/10/2021.
//
#include <path_generator.hh>

int main() {
    PathGenerate riser;
    std::cout << "Creating waypoint pathway" << std::endl;
    riser.setFolderName("/home/vant3d/Documents");
    riser.setFileName("teste.csv");
    riser.setInitCoord( 46.775450, 8.345125, 1839.8 + 10, 74.2);
    riser.setInspectionParam(5, 300,5, 4, 15, -300);
    riser.createInspectionPoints(1);

    std::cout << "Directory " << riser.getFolderName() << riser.getFileName() << std::endl;
    std::cout << "Finished" << std::endl;
    std::vector<std::vector<std::string>> dataList = riser.read_csv(
            riser.getFolderName() + riser.getFileName(), ",");
    // Print the content of row by row on screen
    for (int i = 0; i < dataList.size(); i++) {
        for (int j = 0; j < dataList[0].size(); j++) {
            if (j != dataList[0].size() - 1) { std::cout << dataList[i][j] << " , "; }
            else { std::cout << dataList[i][j] << std::endl; }
            if (i > 0) {
                if (j == 4) {
                    if (dataList[i][j] == "TRUE") { dataList[i][j] = std::to_string(1); } else { dataList[i][j] = std::to_string(0); }
                }
                double value = std::stod(dataList[i][j]);
                if (j != dataList[0].size() - 1) { std::cout << "T - " << std::setprecision(11) << value << " , ";}
                else{std::cout << "T - " << std::setprecision(11) << value <<std::endl;}

            }
        }
    }

    return 0;
}