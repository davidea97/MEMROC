//
// Created by davide on 12/09/23.
//

#ifndef METRIC_CALIBRATOR_READER_H
#define METRIC_CALIBRATOR_READER_H

#include <string>
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <ctime>   // For time()

#include "utils.h"
#include "CalibrationInfo.h"
#include "CameraInfo.h"
#include "display_utils.h"

class Reader{
private:

    std::string folder_path_;

public:
    Reader(const std::string& folder_path);
    std::string getFolderPath() const;
    bool readCalibrationInfo(CalibrationInfo& calib_info);
    std::vector<std::vector<cv::Mat>> readImages(const int camera_num, const double resize_factor, int start_index, int end_index);
    std::vector<std::vector<cv::Mat>> readRobotPoses(const int camera_num, std::vector<std::vector<cv::Mat>> &original_poses, int start_index, int end_index);
    int countImagesInFolder(const std::string& folder_path);
};

#endif //METRIC_CALIBRATOR_READER_H
