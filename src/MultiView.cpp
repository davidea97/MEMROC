//
// Created by davide on 07/11/23.
//

#include "MultiView.h"

MultiView::MultiView(std::vector<cv::Mat> c2c_vec, std::vector<CameraInfo> camera_network_info) {
    c2c_vec_ = c2c_vec;
    camera_network_info_ = camera_network_info;
}

void MultiView::triangulation(std::vector<std::vector<cv::Point2f>> image_points){

    // Check if each camera can detect the whole set of 2d points
    if (checkInnerVecSize(image_points)) {

        // Iterate for each camera which is detecting the target
        for (int i = 0; i < image_points.size(); i++) {
            std::vector<cv::Point2f> pattern = image_points[i];
        }
    }
}