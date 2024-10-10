//
// Created by davide on 12/09/23.
//
#include "Reader.h"

namespace fs = std::filesystem;

Reader::Reader(const std::string& folder_path){
    folder_path_ = folder_path;
    // Check if the folder path exists
    if (isFolderNotEmpty(folder_path)) {
        std::cout << "The path " << folder_path << " exists and is not empty." << std::endl;
    }

}

std::string Reader::getFolderPath() const{
    return folder_path_;
}


// Function to read calibration pattern information from a YAML file
bool Reader::readCalibrationInfo(CalibrationInfo& calib_info) {
    try {
        YAML::Node config = YAML::LoadFile(this->getFolderPath() + "/CalibrationInfo.yaml");

        calib_info.setNumberOfCams(config["number_of_cameras"].as<int>());
        calib_info.setCamFolderPref(config["camera_folder_prefix"].as<std::string>());
        calib_info.setPatternType(config["pattern_type"].as<std::string>());
        calib_info.setNumRow(config["number_of_rows"].as<int>());
        calib_info.setNumCol(config["number_of_columns"].as<int>());
        calib_info.setSize(config["size"].as<double>());
        calib_info.setResizeFactor(config["resize_factor"].as<double>());
        calib_info.setVisualError(config["visual_error"].as<int>());
        calib_info.setGt(config["gt"].as<int>());
        calib_info.setMetric(config["metric"].as<int>());
        calib_info.setCalibSetup(config["calibration_setup"].as<int>());

        calib_info.printCalibInfo();

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

// Function to read pattern images from a provided folder
std::vector<std::vector<cv::Mat>> Reader::readImages(const int camera_num, const double resize_factor, int start_index, int end_index) {
    std::vector<std::vector<cv::Mat>> images(camera_num);
    std::cout << "Reading images.." << std::endl;

    for (int i = 0; i < camera_num; i++){

        std::vector<std::vector<fs::directory_entry>> entries(camera_num);
        for (const auto& entry : fs::directory_iterator(folder_path_ + "/camera" + std::to_string(i+1) + "/image")) {
            if (fs::is_regular_file(entry)) {

                // Check if the file is an image (you can add more extensions as needed)
                std::string file_extension = entry.path().extension();
                if (file_extension == ".jpg" || file_extension == ".jpeg" || file_extension == ".png" || file_extension == ".tiff") {
                    entries[i].push_back(entry);
                }
                else{
                    std::cout << "Image extension " << file_extension << "is not accepted" << std::endl;
                }
            }
        }


        // Sort the filenames in ascending order
        std::sort(entries[i].begin(), entries[i].end(), compareFilenames);

        // Save images
        int counter_progress_bar = 0;
        /*for (const auto& entry : entries[i]) {
            cv::Mat image = cv::imread(entry.path().string());
            showProgressBar(counter_progress_bar, entries[i].size());
            if (!image.empty()) {
                if (resize_factor!=1)
                    cv::resize(image, image, cv::Size(image.cols*resize_factor, image.rows*resize_factor), cv::INTER_CUBIC);
                images[i].push_back(image);
            } else {
                std::cerr << "Error reading image: " << entry.path() << std::endl;
            }

            counter_progress_bar ++;
        }*/
        for (int j = start_index; j < end_index; ++j) {
            const auto& entry = entries[i][j];
            cv::Mat image = cv::imread(entry.path().string());
            // showProgressBar logic might need to be adjusted based on the new iteration range
            showProgressBar(counter_progress_bar - start_index, end_index - start_index); // Adjust progress bar display
            if (!image.empty()) {
                if (resize_factor != 1)
                    cv::resize(image, image, cv::Size(image.cols * resize_factor, image.rows * resize_factor), cv::INTER_CUBIC);
                images[i].push_back(image);
            } else {
                std::cerr << "Error reading image: " << entry.path() << std::endl;
            }

            counter_progress_bar++;
        }
        std::cout << "Camera " << std::to_string(i+1) << ": process completed, " << std::to_string(counter_progress_bar) << " images!" << std::endl;
    }
    return images;
}


// Function to read robot poses from the provided folder
std::vector<std::vector<cv::Mat>> Reader::readRobotPoses(const int camera_num, std::vector<std::vector<cv::Mat>> &original_poses, int start_index, int end_index){
    std::vector<std::vector<cv::Mat>> poses(camera_num);
    std::cout << "Reading poses.." << std::endl;
    for (int i = 0; i < camera_num; i++){

        std::vector<std::vector<fs::directory_entry>> entries(camera_num);
        for (const auto& entry : fs::directory_iterator(folder_path_ + "/camera" + std::to_string(i+1) + "/pose")) {
            if (fs::is_regular_file(entry)) {
                // Check if the file is an image (you can add more extensions as needed)
                std::string fileExtension = entry.path().extension();
                if (fileExtension == ".csv") {
                    entries[i].push_back(entry);
                }
            }
        }

        // Sort the filenames in ascending order
        std::sort(entries[i].begin(), entries[i].end(), compareFilenames);
        int counter_progress_bar = 0;

        /*for (const auto& entry : entries[i]) {
            cv::Mat pose, pose_check_delim;
            std::string path_pose = entry.path();
            readPoseFromCSV(path_pose, pose_check_delim, ',');
            if (pose_check_delim.cols == pose_check_delim.rows){
                readPoseFromCSV(path_pose, pose, ',');
            }
            else{
                readPoseFromCSV(path_pose, pose, ' ');
            }


            showProgressBar(counter_progress_bar, entries[i].size());
            if (!pose.empty()) {

                cv::Mat noisy_pose = addGaussianNoise<float>(pose, 0, 0, 0, 0, 0, 0);
                poses[i].push_back(noisy_pose);
                original_poses[i].push_back(pose);
            } else {
                std::cerr << "Error reading pose: " << entry.path() << std::endl;
                return poses;
            }

            counter_progress_bar ++;
        }*/
        for (int j = start_index; j < end_index; ++j) {
            const auto& entry = entries[i][j];
            cv::Mat pose, pose_check_delim;
            std::string path_pose = entry.path();
            readPoseFromCSV(path_pose, pose_check_delim, ',');
            if (pose_check_delim.cols == pose_check_delim.rows){
                readPoseFromCSV(path_pose, pose, ',');
            }
            else{
                readPoseFromCSV(path_pose, pose, ' ');
            }

            // showProgressBar logic might need to be adjusted based on the new iteration range
            showProgressBar(counter_progress_bar - start_index, end_index-start_index); // Adjust progress bar display
            if (!pose.empty()) {
                //cv::Mat noisy_pose = addGaussianNoise<float>(pose, 0, 0, 0, 0, 0, 0);
                poses[i].push_back(pose);
                original_poses[i].push_back(pose);
            } else {
                std::cerr << "Error reading pose: " << path_pose << std::endl;
                return poses; // Assuming you can return from your function here. Adjust as necessary.
            }

            counter_progress_bar++;
        }

        std::cout << "Camera " << std::to_string(i+1) << ": process completed, " << std::to_string(counter_progress_bar) << " poses!" << std::endl;
    }

    return poses;
}


int Reader::countImagesInFolder(const std::string& folder_path) {
    int image_count = 0;
    const std::vector<std::string> image_extensions = { ".jpg", ".jpeg", ".png", ".gif", ".tiff" };

    try{
        fs::path folder(folder_path);

        if (fs::exists(folder) && fs::is_directory(folder)) {
            for (const auto& entry : fs::directory_iterator(folder)) {
                if (fs::is_regular_file(entry)) {
                    for (const std::string& extension : image_extensions) {
                        if (entry.path().extension() == extension) {
                            image_count++;
                            break;
                        }
                    }
                }
            }
        }
    }
    catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "An error occurred while accessing the folder: " << ex.what() << std::endl;
    }

    return image_count;
}



