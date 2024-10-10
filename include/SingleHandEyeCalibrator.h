//
// Created by davide on 06/10/23.
//

#ifndef METRIC_CALIBRATOR_SINGLEHANDEYECALIBRATOR_H
#define METRIC_CALIBRATOR_SINGLEHANDEYECALIBRATOR_H

#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "utils.h"
#include "handeye_calibrator.h"
#include "PinholeCameraModel.h"

class SingleHandEyeCalibrator{
private:
    std::vector<Eigen::Vector3d> robot_t_vecs_;
    std::vector<Eigen::Vector3d> robot_r_vecs_;
    std::vector< Eigen::Vector3d > pattern_pts_;
    double board2ee_[6];
    double h2e_[6];

public:
    SingleHandEyeCalibrator(const std::vector<cv::Mat> correct_poses, const std::vector<cv::Point3f> object_points, const cv::Mat h2e_initial_guess_vec, const cv::Mat b2ee_initial_guess);
    void Calibration(CameraInfo camera_info, std::vector<std::vector<cv::Point2f>> corners, cv::Mat &optimal_h2e, cv::Mat &optimal_b2ee);
};

#endif //METRIC_CALIBRATOR_SINGLEHANDEYECALIBRATOR_H
