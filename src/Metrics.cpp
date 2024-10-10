//
// Created by davide on 11/10/23.
//

#include <fstream>
#include "Metrics.h"


Metrics::Metrics(const std::vector<cv::Point3f> object_points, const std::vector<cv::Mat> h2e_optimal, const std::vector<cv::Mat> optimal_b2ee, const std::vector<CameraInfo> camera_info, const CalibrationInfo calibration_info, const std::string output_folder, const std::string calibration_method, const std::vector<std::vector<int>> cross_observation_matrix){
    object_points_ = object_points;
    h2e_optimal_ = h2e_optimal;
    optimal_b2ee_ = optimal_b2ee;
    camera_info_ = camera_info;
    calibration_info_ = calibration_info;
    output_folder_ = output_folder;
    calibration_method_ = calibration_method;
    cross_observation_matrix_ = cross_observation_matrix;
    camera_folder_ = output_folder_ +  "/results_" + calibration_method_ + "/" + calibration_info_.getCamFolderPref();
}

void Metrics::projectCorners(const std::vector<std::vector<std::vector<cv::Point2f>>> corners, std::vector<cv::Mat> poses, std::vector<std::vector<cv::Mat>> images, std::vector<std::vector<std::vector<cv::Point2f>>> &corner_points_reprojected) {

    // Iterate the projection of the corners for each camera
    for (int j = 0; j < calibration_info_.getNumberOfCams(); j++) {

        createFolder(camera_folder_ + std::to_string(j + 1) + "/reprojection");
        createFolder(camera_folder_ + std::to_string(j + 1) + "/detection");

        // Reproject for each robot pose the 3d object points onto the image plane of the camera
        for (int i = 0; i < images[j].size(); i++) {
            if (cross_observation_matrix_[i][j]) {
                // Transformation chain from end effector reference frame to the camera reference frame
                cv::Mat transformation_chain;

                // Transformation matrix depends on the calibration setup (0: eye-in-hand, 1: eye-on-base)
                if (calibration_info_.getCalibSetup() == 1 || calibration_info_.getCalibSetup() == 2) {
                    // Distinguish when the optimization provide a single b2ee mat for each camera or a single one for all of them
                    if (optimal_b2ee_.size() == h2e_optimal_.size()) {
                        transformation_chain = h2e_optimal_[j] * poses[i] * optimal_b2ee_[j];
                    } else {
                        transformation_chain = h2e_optimal_[j] * poses[i] * optimal_b2ee_[0];
                    }
                } else {
                    // Distinguish when the optimization provide a single b2ee mat for each camera or a single one for all of them
                    if (optimal_b2ee_.size() == h2e_optimal_.size()) {
                        transformation_chain = h2e_optimal_[j] * poses[i].inv() * optimal_b2ee_[j];
                    } else {
                        transformation_chain = h2e_optimal_[j] * poses[i].inv() * optimal_b2ee_[0];
                    }
                }

                std::vector<cv::Point2f> corner_reprojected;

                cv::Mat total_transf_r_vec, total_transf_t_vec;
                transfMat2Exp<double>(transformation_chain, total_transf_r_vec, total_transf_t_vec);
                projectPoints(cv::Mat(object_points_), total_transf_r_vec, total_transf_t_vec,
                              camera_info_[j].getCameraMatrix(),
                              camera_info_[j].getDistCoeff(), corner_reprojected);
                if (calibration_info_.getVisualError()) {
                    cv::Mat image_reprojected_corner = images[j][i].clone();
                    cv::Mat image_detected_corner = images[j][i].clone();
                    drawChessboardCorners(image_reprojected_corner,
                                          cv::Size(calibration_info_.getNumRow(),
                                                   calibration_info_.getNumCol()),
                                          corner_reprojected, true);
                    imwrite(camera_folder_ + std::to_string(j + 1) + "/reprojection/img" + std::to_string(i) + ".png",
                            image_reprojected_corner);
                    drawChessboardCorners(image_detected_corner,
                                          cv::Size(calibration_info_.getNumRow(),
                                                   calibration_info_.getNumCol()),
                                          corners[i][j], true);
                    imwrite(camera_folder_ + std::to_string(j + 1) + "/detection/img" + std::to_string(i) + ".png",
                            image_detected_corner);
                }
                corner_points_reprojected[j][i] = corner_reprojected;
            }
        }
    }
}


void Metrics::reprojectionError(const std::vector<std::vector<std::vector<cv::Point2f>>> corners, const std::vector<std::vector<std::vector<cv::Point2f>>> corner_points_reprojected){

    for (int j = 0; j < calibration_info_.getNumberOfCams(); j++) {
        double final_pixel_error = 0.0;
        double tmp_error;
        std::vector<double> single_img_pixel(corners.size());

        // Save the reprojection error in a txt file
        std::ofstream output_file_camera;
        output_file_camera.open(
                camera_folder_ + std::to_string(j + 1) + "/reproj_results_camera" + std::to_string(j + 1) + ".txt");

        if (!output_file_camera.is_open()) {
            std::cerr << "Unable to open the file." << std::endl;
        }

        int counter = 0;
        for (int i = 0; i < corners.size(); i++) {
            if (cross_observation_matrix_[i][j]) {
                double pixel_error = 0.0;

                for (int k = 0; k < corners[i][j].size(); k++) {
                    std::vector<cv::Point2f> temp_points = corners[i][j];
                    tmp_error = norm(temp_points[k] - corner_points_reprojected[j][i][k]);
                    pixel_error = pixel_error + tmp_error;
                }
                pixel_error = pixel_error / corners[i][j].size();
                output_file_camera << "Image n. " << std::to_string(i) << ": " << pixel_error << " pix" << std::endl;
                single_img_pixel[i] = pixel_error;
                final_pixel_error = final_pixel_error + pixel_error;
                counter++;
            }
        }

        output_file_camera.close();

        int lines_number = linesNumber(output_folder_ + "/results_" + calibration_method_ + "/reprojection_error.txt");

        std::ofstream output_file;
        if (lines_number < calibration_info_.getNumberOfCams()) {
            output_file.open(output_folder_ + "/results_" + calibration_method_ + "/reprojection_error.txt",
                             std::ios::app);  // Use std::ios::app to append data.
        } else {
            output_file.open(output_folder_ + "/results_" + calibration_method_ + "/reprojection_error.txt");
        }

        if (!output_file.is_open()) {
            std::cerr << "Unable to open the file." << std::endl;
        }
        final_pixel_error = final_pixel_error / counter;

        output_file << "Final pixel reprojection error of camera " << std::to_string(j + 1) << " is: "
                    << final_pixel_error << " pix" << std::endl;
        output_file.close();
    }
}
