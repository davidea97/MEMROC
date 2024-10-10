//
// Created by davide on 07/11/23.
//

#ifndef METRIC_CALIBRATOR_MULTIVIEW_H
#define METRIC_CALIBRATOR_MULTIVIEW_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "CameraInfo.h"
#include "utils.h"

class MultiView{
private:
    std::vector<cv::Mat> c2c_vec_;
    std::vector<CameraInfo> camera_network_info_;

public:
    MultiView(std::vector<cv::Mat> c2c_vec, std::vector<CameraInfo> camera_network_info);
    void triangulation(std::vector<std::vector<cv::Point2f>> image_points);

};

#endif //METRIC_CALIBRATOR_MULTIVIEW_H
