#include <iostream>
#include "Calibrator.h"

int main(int argc, char* argv[]) {

    // Check if there are at least two arguments (program name + user argument)
    int number_of_waypoints = -1;
    int start_index = -1;
    std::string data_folder;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <data_folder>" << std::endl;
        return 1;
    }

    if (argc == 2) {
        // Get the user-specified argument from command line
        data_folder = argv[1];
        std::cout << "Folder: " << data_folder << std::endl;
    }

    // Start the calibration
    Calibrator calibrator(data_folder);
    calibrator.calibration();

    return 0;
}

