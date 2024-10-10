//
// Created by davide on 11/10/23.
//

#ifndef METRIC_CALIBRATOR_METRICS_H
#define METRIC_CALIBRATOR_METRICS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "utils.h"
#include "PinholeCameraModel.h"
#include "CalibrationInfo.h"


class Metrics{
private:
    std::vector<cv::Point3f> object_points_;
    std::vector<cv::Mat> h2e_optimal_;
    std::vector<cv::Mat> optimal_b2ee_;
    std::vector<CameraInfo> camera_info_;
    CalibrationInfo calibration_info_;
    std::string output_folder_;
    std::string calibration_method_;
    std::string camera_folder_;
    std::vector<std::vector<int>> cross_observation_matrix_;

public:
    Metrics(const std::vector<cv::Point3f> object_points, const std::vector<cv::Mat> h2e_optimal, const std::vector<cv::Mat> optimal_b2ee, const std::vector<CameraInfo> camera_info, const CalibrationInfo calibration_info, const std::string output_folder, const std::string calibration_method, const std::vector<std::vector<int>> cross_observation_matrix);
    void projectCorners(const std::vector<std::vector<std::vector<cv::Point2f>>> corners, std::vector<cv::Mat> poses, std::vector<std::vector<cv::Mat>> images, std::vector<std::vector<std::vector<cv::Point2f>>> &corner_points_reprojected);
    void reprojectionError(const std::vector<std::vector<std::vector<cv::Point2f>>> corners, const std::vector<std::vector<std::vector<cv::Point2f>>> corner_points_reprojected);
};


#endif //METRIC_CALIBRATOR_METRICS_H
