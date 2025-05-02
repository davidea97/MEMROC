//
// Created by Davide on 27/09/24.
//

#ifndef MEMROC_DETECTOR_H
#define MEMROC_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>

#include "CalibrationInfo.h"
#include "CameraInfo.h"
#include "utils.h"
#include "display_utils.h"


class Detector{
private:
    CalibrationInfo calib_info_;
    int number_of_waypoints_;
    cv::Size pattern_size_;

public:
    Detector(const CalibrationInfo calib_info, const std::vector<CameraInfo>& camera_network_info, const int number_of_waypoints);
    void patternDetection(std::vector<std::vector<cv::Mat>> images, std::vector<std::vector<cv::Mat>> poses, std::vector<std::vector<cv::Mat>> &correct_images, std::vector<std::vector<cv::Mat>> &correct_poses, std::vector<std::vector<std::vector<cv::Point2f>>> &correct_corners, std::vector<std::vector<int>> &cross_observation_matrix, std::vector<std::vector<cv::Mat>> &rvec_all, std::vector<std::vector<cv::Mat>> &tvec_all);
    void checkerboardDetection(std::vector<std::vector<cv::Mat>> images, std::vector<std::vector<cv::Mat>> poses, std::vector<std::vector<cv::Mat>> &correct_images, std::vector<std::vector<cv::Mat>> &correct_poses, std::vector<std::vector<std::vector<cv::Point2f>>> &correct_corners, std::vector<std::vector<int>> &cross_observation_matrix, std::vector<std::vector<cv::Mat>> &rvec_all, std::vector<std::vector<cv::Mat>> &tvec_all);
    void getObjectPoints(std::vector<cv::Point3f> &objectPoints);
    CalibrationInfo getCalibInfo();
    int getNumberOfWaypoints();

    std::vector<CameraInfo> camera_network_info_;
};

#endif //MEMROC_DETECTOR_H
