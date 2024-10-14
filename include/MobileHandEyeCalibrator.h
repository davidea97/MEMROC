//
// Created by davide on 11/12/23.
//

#ifndef MEMROC_CALIBRATOR_MOBILEHANDEYECALIBRATOR_H
#define MEMROC_CALIBRATOR_MOBILEHANDEYECALIBRATOR_H

#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "utils.h"
#include "display_utils.h"
#include "handeye_calibrator.h"

class MobileHandEyeCalibrator{
private:
    std::vector<std::vector<Eigen::Vector3d>> pnp_t_vecs_;
    std::vector<std::vector<Eigen::Vector3d>> pnp_r_vecs_;
    int number_of_cameras_;
    int number_of_waypoints_;
    std::vector<std::vector<int>> cross_observation_matrix_;
    double** h2e_;
    std::vector<std::vector<double>> cam_z_;

    std::vector<std::vector<Eigen::Vector3d>> robot_t_rel_;
    std::vector<std::vector<Eigen::Vector3d>> robot_r_rel_;

    std::vector<std::vector<Eigen::Vector3d>> pnp_t_rel_;
    std::vector<std::vector<Eigen::Vector3d>> pnp_r_rel_;

public:
    MobileHandEyeCalibrator(const int number_of_waypoints, const int sens_quantity, const std::vector<cv::Mat> h2e_initial_guess_vec, const std::vector<std::vector<int>> cross_observation_matrix,
                            const std::vector<std::vector<double>> cam_z, const std::vector<std::vector<cv::Mat>> relative_robot_poses, const std::vector<std::vector<cv::Mat>> relative_cam_poses);
    ~MobileHandEyeCalibrator() {
        delete[] h2e_;
    }
    void mobileCalibration(std::vector<cv::Mat> &optimal_h2e);
    void mobileJointCalibration(std::vector<cv::Mat> &optimal_h2e);

};

#endif //MEMROC_CALIBRATOR_MOBILEHANDEYECALIBRATOR_H
