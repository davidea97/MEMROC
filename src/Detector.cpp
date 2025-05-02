//
// Created by davide on 03/10/23.
//

#include "Detector.h"


Detector::Detector(const CalibrationInfo calib_info, const std::vector<CameraInfo>& camera_network_info, const int number_of_waypoints){

    calib_info_ = calib_info;
    camera_network_info_ = camera_network_info;
    number_of_waypoints_ = number_of_waypoints;
    pattern_size_ = cv::Size(calib_info_.getNumRow(), calib_info_.getNumCol());
}

void Detector::getObjectPoints(std::vector<cv::Point3f> &objectPoints){

    for (int i = 0; i < calib_info_.getNumCol(); ++i) {
        for (int j = 0; j < calib_info_.getNumRow(); ++j) {
            objectPoints.push_back(cv::Point3f(float(j * calib_info_.getSize()), float(i * calib_info_.getSize()), 0));
        }
    }
}

void Detector::patternDetection(std::vector<std::vector<cv::Mat>> images,
                                std::vector<std::vector<cv::Mat>> poses,
                                std::vector<std::vector<cv::Mat>> &correct_images,
                                std::vector<std::vector<cv::Mat>> &correct_poses,
                                std::vector<std::vector<std::vector<cv::Point2f>>> &correct_corners,
                                std::vector<std::vector<int>> &cross_observation_matrix,
                                std::vector<std::vector<cv::Mat>> &rvec_all,
                                std::vector<std::vector<cv::Mat>> &tvec_all){
    checkerboardDetection(images, poses, correct_images, correct_poses, correct_corners, cross_observation_matrix, rvec_all, tvec_all);
}

void Detector::checkerboardDetection(std::vector<std::vector<cv::Mat>> images,
                                    std::vector<std::vector<cv::Mat>> poses,
                                    std::vector<std::vector<cv::Mat>> &correct_images,
                                    std::vector<std::vector<cv::Mat>> &correct_poses,
                                    std::vector<std::vector<std::vector<cv::Point2f>>> &correct_corners,
                                    std::vector<std::vector<int>> &cross_observation_matrix,
                                    std::vector<std::vector<cv::Mat>> &rvec_all,
                                    std::vector<std::vector<cv::Mat>> &tvec_all){

    std::vector<std::vector<std::vector<cv::Point2f>>> correct_corners_only(images.size());
    std::cout << "Detecting corners.." << std::endl;

    // Define the ChArUco board
    int squaresX = 10; // or your board size
    int squaresY = 14;
    float squareLength = 0.072f;  // in meters
    float markerLength = 0.056f;  // in meters

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::aruco::CharucoBoard::create(squaresX,
                                        squaresY,
                                        squareLength,
                                        markerLength,
                                        dictionary);

    for (int cam = 0; cam < calib_info_.getNumberOfCams(); cam++) {

        // Get the number of images for each camera
        int number_of_images = images[cam].size();
        // Start detection loop

        for (int img = 0; img < number_of_images; img++){

            showProgressBar(img, number_of_images);

            // Charuco detection
            cv::Mat image = images[cam][img].clone();

            // Detect ArUco markers
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

            // Interpolate ChArUco corners
            cv::Mat charucoCorners, charucoIds;

            if (!markerIds.empty()) {
                cv::aruco::interpolateCornersCharuco(markerCorners,
                                                    markerIds,
                                                    image,
                                                    board,
                                                    charucoCorners,
                                                    charucoIds,
                                                    camera_network_info_[cam].getCameraMatrix(),
                                                    camera_network_info_[cam].getDistCoeff());
            }

            if (charucoIds.total() > 0) {
                cv::Mat rvec, tvec;
                bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners,
                                                                charucoIds,
                                                                board,
                                                                camera_network_info_[cam].getCameraMatrix(),
                                                                camera_network_info_[cam].getDistCoeff(),
                                                                rvec,
                                                                tvec);
                if (valid) {
                    // Draw and optionally save image
                    cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
                    cv::aruco::drawDetectedCornersCharuco(image, charucoCorners, charucoIds);
                    cv::aruco::drawAxis(image,
                                        camera_network_info_[cam].getCameraMatrix(),
                                        camera_network_info_[cam].getDistCoeff(),
                                        rvec,
                                        tvec,
                                        0.1);
                    cv::imwrite("charuco_image" + std::to_string(img) + ".png", image);
                    
                    std::cout << "Camera " << std::to_string(cam+1) << " - Image " << img << ": charuco corners detected!" << std::endl;
                    std::cout << "Rotation Vector:\n" << rvec << std::endl;
                    std::cout << "Translation Vector:\n" << tvec << std::endl;

                    if (calib_info_.getVisualError())
                        correct_images[cam].push_back(images[cam][img].clone());
                    correct_poses[cam].push_back(poses[cam][img].clone());
                    correct_corners_only[cam].push_back(charucoCorners);
                    cross_observation_matrix[img][cam] = 1;
                    correct_corners[img][cam] = charucoCorners;
                    rvec_all[img][cam] = rvec;
                    tvec_all[img][cam] = tvec;
                } else {
                    std::cerr << "Pose could not be estimated." << std::endl;
                }
            } else {
                std::cout << "Pattern not found in image " << img << " of camera " << cam + 1 << std::endl;
            }

            // // Convert from BGR to gray image
            // cv::Mat gray_image;
            // cv::cvtColor(images[cam][img], gray_image, cv::COLOR_BGR2GRAY);

            // // Corner detection
            // std::vector<cv::Point2f> corners;
            // bool pattern_found = cv::findChessboardCorners(gray_image, pattern_size_, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

            // // Sub pixel accuracy
            // if(pattern_found) {
            //     cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            //     std::vector<cv::Point3f> object_points;
            //     getObjectPoints(object_points);

            //     // RANSAC filter
            //     cv::Mat rvec, tvec;
            //     bool correct_detection = filterRansac(object_points, corners, camera_network_info_[cam], rvec, tvec);

            //     if (correct_detection){
            //         if (calib_info_.getVisualError())
            //             correct_images[cam].push_back(images[cam][img].clone());
            //         correct_poses[cam].push_back(poses[cam][img].clone());
            //         correct_corners_only[cam].push_back(corners);

            //         cross_observation_matrix[img][cam] = 1;
            //         correct_corners[img][cam] = corners;
            //         rvec_all[img][cam] = rvec;
            //         tvec_all[img][cam] = tvec;

            //     }
            // }
        }

        std::cout << "Camera " << std::to_string(cam+1) << ": process completed, " << std::to_string(correct_corners_only[cam].size()) << " filtered images!" << std::endl;
    }
}


CalibrationInfo Detector::getCalibInfo(){
    return this->calib_info_;
}

int Detector::getNumberOfWaypoints(){
    return this->number_of_waypoints_;
}