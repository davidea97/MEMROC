//
// Calibrator class for MEMROC project
// Created by Davide on 27/09/24.
//

#ifndef MEMROC_CALIBRATOR_H
#define MEMROC_CALIBRATOR_H

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <filesystem>

#include "Reader.h"
#include "Detector.h"
#include "SingleHandEyeCalibrator.h"
#include "MultiHandEyeCalibrator.h"
#include "MobileHandEyeCalibrator.h"
#include "utils.h"
#include "Metrics.h"

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class Calibrator {
private:
    std::string data_;

    // Helper functions for calibration process
    void setupCalibration(CalibrationInfo& calib_info, Reader& reader, std::vector<CameraInfo>& camera_network_info, int number_of_cameras);
    void readAndProcessData(Reader& reader, CalibrationInfo& calib_info, std::vector<std::vector<cv::Mat>>& images_collected, std::vector<std::vector<cv::Mat>>& poses_collected, int& number_of_waypoints);
    void detectCalibrationPattern(Detector detector, CalibrationInfo calib_info, std::vector<std::vector<cv::Mat>>& images_collected, std::vector<std::vector<cv::Mat>>& poses_collected, std::vector<std::vector<cv::Mat>>& rvec_all, std::vector<std::vector<cv::Mat>>& tvec_all, std::vector<std::vector<cv::Mat>>& rvec_used, std::vector<std::vector<cv::Mat>>& tvec_used, std::vector<std::vector<cv::Mat>>& rototras_vec, std::vector<std::vector<cv::Mat>>& rototras_all, std::vector<std::vector<cv::Mat>>& correct_poses, std::vector<std::vector<int>>& cross_observation_matrix);
    void computePoses(std::vector<std::vector<cv::Mat>>& relative_robot_poses, std::vector<std::vector<cv::Mat>>& relative_cam_poses, std::vector<std::vector<cv::Mat>>& rvec_all, std::vector<std::vector<cv::Mat>>& tvec_all, std::vector<std::vector<cv::Mat>> poses_collected, std::vector<std::vector<cv::Mat>> rototras_vec, std::vector<std::vector<cv::Mat>> rototras_all, std::vector<std::vector<int>> cross_observation_matrix);
    void svdElaboration(const std::vector<std::vector<cv::Mat>>& poses_collected, const std::vector<std::vector<cv::Mat>>& rvec_all, const std::vector<std::vector<cv::Mat>>& relative_robot_poses, const std::vector<std::vector<cv::Mat>>& relative_cam_poses, std::vector<std::vector<cv::Mat>> rototras_vec, std::vector<std::vector<cv::Mat>> rototras_all, std::vector<std::vector<int>> cross_observation_matrix, std::vector<cv::Mat>& svd_mat_vec, std::vector<cv::Mat>& svd_mat_inv_vec);
    void processPointClouds(const std::vector<std::vector<cv::Mat>>& poses_collected, std::vector<std::vector<int>> cross_observation_matrix, std::vector<cv::Mat> svd_mat_vec, std::vector <cv::Mat> svd_mat_inv_vec, std::vector<std::vector<cv::Mat>> rototras_vec, std::vector<std::vector<double>>& d_camera_whole, std::vector<std::vector<double>>& ang_camera_whole);
    void groundPlaneDetection(int cam, std::vector<cv::Mat>& pointcloud_vec, int number_of_cameras, int end_index);
    void processPointCloudData(int cam, std::vector<cv::Mat>& pointcloud_vec, std::vector<double>& d_camera_vec, std::vector<double>& ang_camera_vec, std::vector<std::vector<int>> cross_observation_matrix, std::vector<cv::Mat>& svd_mat_vec, std::vector <cv::Mat>& svd_mat_inv_vec, std::vector<std::vector<cv::Mat>> rototras_vec);
    void processPointCloudFeatures(const pcl::ModelCoefficients::Ptr& coefficients, double& d_board, double& ang_board);
    void calibrationProcess(Detector detector,std::vector<std::vector<cv::Mat>> correct_poses, std::vector<std::vector<cv::Mat>> rototras_all, std::vector<std::vector<int>> cross_observation_matrix, std::vector<std::vector<cv::Mat>>& relative_robot_poses, std::vector<std::vector<cv::Mat>>& relative_cam_poses, std::vector<std::vector<double>> d_camera_whole);

public:
    // Constructor
    explicit Calibrator(const std::string& data_folder) : data_(data_folder) {}

    // Main function to run the calibration process
    void calibration();
};

#endif //MEMROC_CALIBRATOR_H
