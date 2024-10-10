//
// Created by davide on 12/09/23.
//

#ifndef METRIC_CALIBRATOR_CAMERAINFO_H
#define METRIC_CALIBRATOR_CAMERAINFO_H

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

// Define the object CameraInfo which includes all the intrinsic parameters of the camera
class CameraInfo{
private:
    float fx_, fy_, cx_, cy_;
    int has_dist_coeff_;
    float dist_k0_;
    float dist_k1_;
    float dist_px_;
    float dist_py_;
    float dist_k2_;
    float dist_k3_;
    float dist_k4_;
    float dist_k5_;
    int img_width_;
    int img_height_;

public:
    explicit CameraInfo();

    // Set the intrinsic parameters of a Camera Info objects by reading from a yaml file
    bool setParameters(const std::string& intrinsic_path, const float resize_factor);

    // Return fx
    [[nodiscard]] float getFx() const;
    void setFx(float new_fx);
    // Return fy
    [[nodiscard]] float getFy() const;
    void setFy(float new_fy);
    // Return Cx
    [[nodiscard]] float getCx() const;
    void setCx(float new_cx);
    // Return Cy
    [[nodiscard]] float getCy() const;
    void setCy(float new_cy);

    // Return distortion coeffient k0
    [[nodiscard]] float getDistK0() const;
    // Return distortion coeffient k1
    [[nodiscard]] float getDistK1() const;
    // Return distortion coeffient Px
    [[nodiscard]] float getDistPx() const;
    // Return distortion coeffient Py
    [[nodiscard]] float getDistPy() const;
    // Return distortion coeffient k2
    [[nodiscard]] float getDistK2() const;
    // Return distortion coeffient k3
    [[nodiscard]] float getDistK3() const;
    // Return distortion coeffient k4
    [[nodiscard]] float getDistK4() const;
    // Return distortion coeffient k5
    [[nodiscard]] float getDistK5() const;
    // Return image width
    [[nodiscard]] int getImageWidth() const;
    // Return image height
    [[nodiscard]] int getImageHeight() const;
    // Return the whole camera matrix
    [[nodiscard]] cv::Mat getCameraMatrix() const;
    void setCameraMatrix(cv::Mat new_camera_matrix);
    // Return the distortion coefficients
    [[nodiscard]] cv::Mat getDistCoeff() const;
    void setDistCoeff(cv::Mat new_dist_coeff);


};

#endif //METRIC_CALIBRATOR_CAMERAINFO_H
