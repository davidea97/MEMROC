//
// Created by Davide on 27/09/24.
//

#include "Calibrator.h"

int MAX_IMAGES = 300;

void Calibrator::calibration() {
    std::cout << "##################################################" << std::endl;
    std::cout << "Start the calibration!" << std::endl;

    CalibrationInfo calib_info;
    Reader reader(data_);

    // Read calibration data and setup cameras
    reader.readCalibrationInfo(calib_info);
    const int number_of_cameras = calib_info.getNumberOfCams();
    std::vector<CameraInfo> camera_network_info(number_of_cameras);
    setupCalibration(calib_info, reader, camera_network_info, number_of_cameras);

    // Read and process image and pose data
    int number_of_waypoints;
    std::vector<std::vector<cv::Mat>> images_collected, poses_collected;
    std::vector<std::vector<cv::Mat>> rototras_vec(number_of_cameras), rototras_all(number_of_cameras);
    
    readAndProcessData(reader, calib_info, images_collected, poses_collected, number_of_waypoints);

    // Pattern detection and pose estimation
    std::vector<std::vector<cv::Mat>> rvec_all(number_of_waypoints, std::vector<cv::Mat>(number_of_cameras)), tvec_all(number_of_waypoints, std::vector<cv::Mat>(number_of_cameras));
    std::vector<std::vector<cv::Mat>> rvec_used(number_of_cameras, std::vector<cv::Mat>(number_of_waypoints)), tvec_used(number_of_cameras, std::vector<cv::Mat>(number_of_waypoints));
    std::vector<std::vector<int>> cross_observation_matrix(number_of_waypoints, std::vector<int>(number_of_cameras, 0));
    
    Detector detector(calib_info, camera_network_info, number_of_waypoints);
    std::vector<std::vector<cv::Mat>> correct_poses(number_of_cameras);
    detectCalibrationPattern(detector,
                            calib_info,
                            images_collected,
                            poses_collected,
                            rvec_all,
                            tvec_all,
                            rvec_used,
                            tvec_used,
                            rototras_vec,
                            rototras_all,
                            correct_poses,
                            cross_observation_matrix);

    // Compute camera and robot poses
    std::vector<std::vector<cv::Mat>> relative_robot_poses, relative_cam_poses;
    computePoses(relative_robot_poses, relative_cam_poses, rvec_all, tvec_all, poses_collected, rototras_vec, rototras_all, cross_observation_matrix);

    // SVD Elaboration
    std::vector<cv::Mat> svd_mat_vec(number_of_cameras);
    std::vector <cv::Mat> svd_mat_inv_vec(number_of_cameras);
    svdElaboration(poses_collected, rvec_all, relative_robot_poses, relative_cam_poses, rototras_vec, rototras_all, cross_observation_matrix, svd_mat_vec, svd_mat_inv_vec);

    std::vector<std::vector<double>> d_camera_whole(number_of_cameras), ang_camera_whole(number_of_cameras);
    // Point cloud processing and ground plane detection
    processPointClouds(poses_collected, cross_observation_matrix, svd_mat_vec, svd_mat_inv_vec, rototras_vec, d_camera_whole, ang_camera_whole);

    // Set initial guess and start calibration process
    calibrationProcess(detector, correct_poses, rototras_all, cross_observation_matrix, relative_robot_poses, relative_cam_poses, d_camera_whole);
}

void Calibrator::setupCalibration(CalibrationInfo& calib_info, Reader& reader, std::vector<CameraInfo>& camera_network_info, int number_of_cameras) {
    
    int total_images = countImagesInFolder(data_ + "/" + calib_info.getCamFolderPref() + "1/image");

    int start_index = 0;
    int end_index = total_images;

    // Setup camera network info
    
    for (int i = 0; i < number_of_cameras; i++) {
        camera_network_info[i].setParameters(data_ + "/" + calib_info.getCamFolderPref() + std::to_string(i + 1), calib_info.getResizeFactor());
    }
}

void Calibrator::readAndProcessData(Reader& reader,
                                    CalibrationInfo& calib_info,
                                    std::vector<std::vector<cv::Mat>>& images_collected,
                                    std::vector<std::vector<cv::Mat>>& poses_collected,
                                    int& number_of_waypoints) {
    const int number_of_cameras = calib_info.getNumberOfCams();
    int total_images = countImagesInFolder(data_ + "/" + calib_info.getCamFolderPref() + "1/image");

    int start_index = 0;
    int end_index = total_images;
    number_of_waypoints = end_index - start_index;

    std::vector<std::vector<cv::Mat>> original_poses(number_of_cameras);
    images_collected = reader.readImages(number_of_cameras, calib_info.getResizeFactor(), start_index, end_index);
    poses_collected = reader.readRobotPoses(number_of_cameras, original_poses, start_index, end_index);
}

void Calibrator::detectCalibrationPattern(Detector detector,
                                        CalibrationInfo calib_info,
                                        std::vector<std::vector<cv::Mat>>& images_collected,
                                        std::vector<std::vector<cv::Mat>>& poses_collected,
                                        std::vector<std::vector<cv::Mat>>& rvec_all,
                                        std::vector<std::vector<cv::Mat>>& tvec_all,
                                        std::vector<std::vector<cv::Mat>>& rvec_used,
                                        std::vector<std::vector<cv::Mat>>& tvec_used,
                                        std::vector<std::vector<cv::Mat>>& rototras_vec,
                                        std::vector<std::vector<cv::Mat>>& rototras_all,
                                        std::vector<std::vector<cv::Mat>>& correct_poses,
                                        std::vector<std::vector<int>>& cross_observation_matrix) {
    const int number_of_cameras = calib_info.getNumberOfCams();
    int number_of_waypoints = images_collected[0].size();

    // Prepare structures for pattern detection
    std::vector<std::vector<cv::Mat>> correct_images(number_of_cameras);
    std::vector<std::vector<std::vector<cv::Point2f>>> correct_corners(number_of_waypoints, std::vector<std::vector<cv::Point2f>>(number_of_cameras));
    detector.patternDetection(images_collected, poses_collected, correct_images, correct_poses, correct_corners, cross_observation_matrix, rvec_all, tvec_all);
    for (int j = 0; j < number_of_cameras; j++) {
        for (int i = 0; i < poses_collected[0].size(); i++) {
            poses_collected[j][i].convertTo(poses_collected[j][i], CV_64F);
        }
    }
    for (int i = 0; i < number_of_cameras; i++) {
        std::vector<cv::Mat> rototras_camera(number_of_waypoints);
        std::vector<cv::Mat> rototras_camera_push;
        for (int j = 0; j < number_of_waypoints; j++) {
            if (cross_observation_matrix[j][i]) {
                cv::Mat rototras;
                cv::Mat rotation;
                cv::Rodrigues(rvec_all[j][i], rotation);
                getRotoTras<double>(rotation, tvec_all[j][i], rototras);

                getTras<double>(rototras, tvec_used[i][j]);
                cv::Mat rotation_temp;
                getRoto<double>(rototras, rotation_temp);
                cv::Rodrigues(rotation_temp, rvec_used[i][j]);

                rototras_camera[j] = rototras;
                rototras_camera_push.push_back(rototras);
            }
        }
        rototras_vec[i] = rototras_camera;
        rototras_all[i] = rototras_camera_push;
    }
}

void Calibrator::computePoses(std::vector<std::vector<cv::Mat>>& relative_robot_poses,
                            std::vector<std::vector<cv::Mat>>& relative_cam_poses,
                            std::vector<std::vector<cv::Mat>>& rvec_all,
                            std::vector<std::vector<cv::Mat>>& tvec_all,
                            std::vector<std::vector<cv::Mat>> poses_collected,
                            std::vector<std::vector<cv::Mat>> rototras_vec,
                            std::vector<std::vector<cv::Mat>> rototras_all,
                            std::vector<std::vector<int>> cross_observation_matrix) {
    const int number_of_cameras = rvec_all[0].size();
    int number_of_waypoints = poses_collected[0].size();
    // Compute relative robot and camera poses
    relative_robot_poses.resize(number_of_cameras, std::vector<cv::Mat>(number_of_waypoints - 1));
    relative_cam_poses.resize(number_of_cameras, std::vector<cv::Mat>(number_of_waypoints - 1));
    for (int i = 0; i < number_of_cameras; i++) {
        for (int j = 0; j < number_of_waypoints - 1; j++) {
            relative_robot_poses[i][j] = poses_collected[i][j].inv() * poses_collected[i][j + 1];
            if (cross_observation_matrix[j][i] && cross_observation_matrix[j + 1][i]){
                relative_cam_poses[i][j] = rototras_vec[i][j] * rototras_vec[i][j + 1].inv();
            }
        }
    }
}

void Calibrator::svdElaboration(const std::vector<std::vector<cv::Mat>>& poses_collected,
                                const std::vector<std::vector<cv::Mat>>& rvec_all,
                                const std::vector<std::vector<cv::Mat>>& relative_robot_poses,
                                const std::vector<std::vector<cv::Mat>>& relative_cam_poses,
                                std::vector<std::vector<cv::Mat>> rototras_vec,
                                std::vector<std::vector<cv::Mat>> rototras_all,
                                std::vector<std::vector<int>> cross_observation_matrix,
                                std::vector<cv::Mat>& svd_mat_vec,
                                std::vector<cv::Mat>& svd_mat_inv_vec) {
    const int number_of_cameras = rvec_all[0].size();
    int number_of_waypoints = poses_collected[0].size();

    // SVD elaboration logic
    for (int i = 0; i < number_of_cameras; i++) {
        cv::Mat data_pnp(rototras_all[i].size(), 3, CV_64F);
        cv::Mat data_robot(rototras_all[i].size(), 3, CV_64F);

        // Extract translation matrices
        transVec2mat(rototras_vec[i], cross_observation_matrix, data_pnp, true, i);
        transVec2mat(poses_collected[i], cross_observation_matrix, data_robot, false, i);

        // Perform SVD
        cv::Mat svd_mat = getSVD(data_robot, data_pnp);
        svd_mat_vec[i] = svd_mat;
        

        cv::Mat svd_temp = (cv::Mat_<double>(4, 4) << svd_mat_vec[i].at<double>(0, 0), svd_mat_vec[i].at<double>(0, 1), svd_mat_vec[i].at<double>(0, 2), 0,
                            svd_mat_vec[i].at<double>(1, 0), svd_mat_vec[i].at<double>(1, 1), svd_mat_vec[i].at<double>(1, 2), 0,
                            svd_mat_vec[i].at<double>(2, 0), svd_mat_vec[i].at<double>(2, 1), svd_mat_vec[i].at<double>(2, 2), 0, 0, 0, 0, 1);

        for (int j = 0; j < number_of_waypoints; j++) {
            std::cout << "Svd iter calculation: " << i << " " << j << std::endl;
            if (cross_observation_matrix[j][i]) {
                svd_mat_inv_vec[i] = poses_collected[i][j].inv() * svd_temp * rototras_vec[i][j].inv();
                std::cout << "Esimated svd inv: " << svd_mat_inv_vec[i] << std::endl;
                break;
            }
        }
    }
}

void Calibrator::processPointClouds(const std::vector<std::vector<cv::Mat>>& poses_collected,
                                    std::vector<std::vector<int>> cross_observation_matrix,
                                    std::vector<cv::Mat> svd_mat_vec,
                                    std::vector<cv::Mat> svd_mat_inv_vec,
                                    std::vector<std::vector<cv::Mat>> rototras_vec,
                                    std::vector<std::vector<double>>& d_camera_whole,
                                    std::vector<std::vector<double>>& ang_camera_whole) {
    const int number_of_cameras = poses_collected.size();
    const int number_of_waypoints = poses_collected[0].size();

    // Process the pcl provided by each camera
    for (int cam = 0; cam < number_of_cameras; cam++) {
        std::vector<cv::Mat> pointcloud_vec(number_of_waypoints);

        // Prepare data structures
        std::vector<double> d_camera_vec(number_of_waypoints), ang_camera_vec(number_of_waypoints);

        std::cout << "#################################### CAMERA " << std::to_string(cam + 1) << " ####################################" << std::endl;

        // Point cloud processing
        preparePlaneDetection(cam, pointcloud_vec, number_of_cameras, number_of_waypoints);

        // Process pointcloud data and extract features
        groundPlaneDetection(cam, pointcloud_vec, d_camera_vec, ang_camera_vec, cross_observation_matrix, svd_mat_vec, svd_mat_inv_vec, rototras_vec);
        
        // double threshold = 0.995;
        double threshold = 0.990;
        for (int i = 0; i < number_of_waypoints; i++) {
            if (ang_camera_vec[i] < threshold) {
                d_camera_vec[i] = 0;
            }
            std::cout << "Camera " << std::to_string(cam + 1) << " - Pointcloud "
                        << std::to_string(i) << ": d_camera = " << d_camera_vec[i]
                        << ", ang_camera = " << ang_camera_vec[i] << std::endl;
        }

        d_camera_whole[cam] = d_camera_vec;
        ang_camera_whole[cam] = ang_camera_vec;
    }
}

void Calibrator::preparePlaneDetection(int cam, std::vector<cv::Mat>& pointcloud_vec, int number_of_cameras, int end_index) {
    namespace fs = std::filesystem;
    std::vector<std::vector<fs::directory_entry>> entries(number_of_cameras);

    for (const auto& entry : fs::directory_iterator(data_ + "/camera" + std::to_string(cam + 1) + "/clouds/")) {
        if (fs::is_regular_file(entry) && entry.path().extension() == ".txt") {
            entries[cam].push_back(entry);
        }
    }

    std::sort(entries[cam].begin(), entries[cam].end(), compareFilenames);
    int min = std::min(end_index, static_cast<int>(entries[cam].size()));

    for (int i = 0; i < min; i++) {
        const auto& entry = entries[cam][i];
        cv::Mat temp_pcl = readPointsFromFile(entry.path().string());
        if (!temp_pcl.empty()) {
            cv::Mat newMat; 
            cv::Mat rowToAdd = cv::Mat::ones(1, temp_pcl.rows, temp_pcl.type());
            cv::vconcat(temp_pcl.t(), rowToAdd, newMat);
            pointcloud_vec[i] = newMat;
        }
    }
}

void Calibrator::groundPlaneDetection(int cam, std::vector<cv::Mat>& pointcloud_vec, std::vector<double>& d_camera_vec, std::vector<double>& ang_camera_vec, std::vector<std::vector<int>> cross_observation_matrix, std::vector<cv::Mat>& svd_mat_vec, std::vector <cv::Mat>& svd_mat_inv_vec, std::vector<std::vector<cv::Mat>> rototras_vec) {
    
    // Process the pointcloud data and compute RANSAC models
    // double ground_plane_confidence = 0.9999;
    double ground_plane_confidence = 0.98;
    for (int i = 0; i < pointcloud_vec.size(); i++) {
        if (cross_observation_matrix[i][cam] && !pointcloud_vec[i].empty()) {
            cv::Mat null_tras = (cv::Mat_<double>(3, 1) << 0, 0, 0);
            cv::Mat svd;
            getRotoTras<double>(svd_mat_inv_vec[cam], null_tras, svd);
            cv::Mat pointcloud_world = svd * pointcloud_vec[i];

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);

            // Fill the cloud with some points
            for (int k = 0; k < pointcloud_world.cols; k++) {
                cloud->points.push_back(
                        pcl::PointXYZ(pointcloud_world.at<double>(0, k), pointcloud_world.at<double>(1, k),
                                        pointcloud_world.at<double>(2, k)));
                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->is_dense = false;
            }

            // RANSAC
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(1 - ground_plane_confidence);
            seg.setAxis(Eigen::Vector3f(0, 0, 1));
            seg.setEpsAngle(10.0f * (M_PI / 180.0f));
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
            }

            std::ofstream outfile("inliers_" + std::to_string(i) + ".txt");
            if (outfile.is_open()) {
                for (const auto& idx : inliers->indices) {
                    outfile << idx << "\n";
                }
                outfile.close();
                std::cout << "Inliers saved to inliers.txt" << std::endl;
            } else {
                std::cerr << "Unable to open file for writing." << std::endl;
            }

            // save the point cloud
            std::ofstream outfile2("pointcloud_" + std::to_string(i) + ".txt");
            if (outfile2.is_open()) {
                for (const auto& point : cloud->points) {
                    outfile2 << point.x << " " << point.y << " " << point.z << "\n";
                }
                outfile2.close();
                std::cout << "Point cloud saved to pointcloud.txt" << std::endl;
            } else {
                std::cerr << "Unable to open file for writing." << std::endl;
            }

            // Process pointcloud features
            if (coefficients->values.empty()) {
                std::cerr << "No coefficients found." << std::endl;
                ang_camera_vec[i] = 0;
                d_camera_vec[i] = 0;
                continue;
            }
            computeCameraHeight(coefficients, d_camera_vec[i], ang_camera_vec[i]);

            std::cout << "Camera " << std::to_string(cam + 1) << " - Pointcloud "
                    << std::to_string(i) << ": d_camera = " << d_camera_vec[i]
                    << ", ang_camera = " << ang_camera_vec[i] << std::endl;
            // // Number of inliers
            // std::cout << "Number of inliers: " << inliers->indices.size() << std::endl;
        }
    }
}

void Calibrator::computeCameraHeight(const pcl::ModelCoefficients::Ptr& coefficients, double& d_camera, double& ang_camera) {
    
    // Process and extract pointcloud features
    cv::Mat normal_wrt_world = (cv::Mat_<double>(3, 1) << coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    cv::Point3d normal_vec_wrt_world(
        normal_wrt_world.at<double>(0),
        normal_wrt_world.at<double>(1),
        normal_wrt_world.at<double>(2));

    cv::Point3d normal_world(0, 0, 1);
    double scalar = normal_world.dot(normal_vec_wrt_world);

    d_camera = coefficients->values[3];
    ang_camera = scalar;
}

void Calibrator::calibrationProcess(Detector detector, std::vector<std::vector<cv::Mat>> correct_poses, std::vector<std::vector<cv::Mat>> rototras_all, std::vector<std::vector<int>> cross_observation_matrix, std::vector<std::vector<cv::Mat>>& relative_robot_poses, std::vector<std::vector<cv::Mat>>& relative_cam_poses, std::vector<std::vector<double>> d_camera_whole) {
    std::vector<cv::Point3f> object_points;
    detector.getObjectPoints(object_points);
    
    int number_of_cameras = detector.camera_network_info_.size();
    std::vector<cv::Mat> multi_h2e_optimal_vec(number_of_cameras);
    // Set initial guess
    std::vector<cv::Mat> h2e_initial_guess_vec(number_of_cameras);
    cv::Mat b2ee_initial_guess;
    setInitialGuess(h2e_initial_guess_vec, rototras_all, correct_poses);

    std::cout << "Initial guess:" << std::endl;
    for (int i = 0; i < number_of_cameras; i++) {
        std::cout << "H2e: " << h2e_initial_guess_vec[i] << std::endl;
    }

    // Start the calibration process
    MobileHandEyeCalibrator mobile_calibrator(detector.getNumberOfWaypoints(), number_of_cameras, h2e_initial_guess_vec, cross_observation_matrix, d_camera_whole, relative_robot_poses, relative_cam_poses);
    mobile_calibrator.mobileCalibration(multi_h2e_optimal_vec);

    std::vector<cv::Mat> best_h2e_wor(number_of_cameras);
    for (int i = 0; i < number_of_cameras; i++) {
        best_h2e_wor[i] = multi_h2e_optimal_vec[i];
        std::cout << "Best H2e MEMROC: " << std::to_string(i + 1) << best_h2e_wor[i] << std::endl;
    }

    mobile_calibrator.mobileJointCalibration(multi_h2e_optimal_vec);
    std::vector<cv::Mat> best_h2e_wor_joint(number_of_cameras);
    for (int i = 0; i < number_of_cameras; i++) {
        best_h2e_wor_joint[i] = multi_h2e_optimal_vec[i];
        std::cout << "Best H2e MEMROC joint: " << std::to_string(i + 1) << best_h2e_wor_joint[i] << std::endl;
    }
}
