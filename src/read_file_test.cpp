//
// Created by vant3d on 25/10/2021.
//
#include <path_generator.hh>
#include <read_file.h>

int main() {
    PathGenerate riser;
    std::cout << "Creating waypoint pathway" << std::endl;
    riser.setFolderName("/home/vant3d/Documents");
    riser.setFileName("teste.csv");
    riser.setInitCoord(5, 0.3, 46.775450, 8.345125, 1839.8 + 10, 74.2);
    riser.setInspectionParam(5, 4, 15, -0.3);
    riser.createInspectionPoints();

    std::cout << "Directory " << riser.getFolderName() << riser.getFileName() << std::endl;
    std::cout << "Finished" << std::endl;
    std::vector<std::pair<std::string, std::vector<double>>> read_csv = riser.read_csv(
            riser.getFolderName() + riser.getFileName());
    // Creating an object of CSVWriter
    CSVReader reader(riser.getFolderName() + riser.getFileName());
    // Get the data from CSV File
    std::vector<std::vector<std::string> > dataList = reader.getData();
    // Print the content of row by row on screen
    for (int i = 0; i < dataList.size(); i++) {
        for (int j = 0; j < dataList[0].size(); j++) {
            if (j != dataList[0].size() - 1) { std::cout << dataList[i][j] << " , "; }
            else { std::cout << dataList[i][j] << std::endl; }
            if(i > 0)
            {
                double value = std::stod(dataList[i][j]);
                std::cout << "TESTE: " << std::setprecision(11) << value << std::endl;

            }
        }
    }

    return 0;
}