//
// Created by davide on 28/09/23.
//
#include "CameraInfo.h"
#include "utils.h"

CameraInfo::CameraInfo(){
}

float CameraInfo::getFx() const{
    return this->fx_;
}

void CameraInfo::setFx(float new_fx){
    this->fx_ = new_fx;
}

float CameraInfo::getFy() const{
    return this->fy_;
}

void CameraInfo::setFy(float new_fy){
    this->fy_ = new_fy;
}

float CameraInfo::getCx() const {
    return this->cx_;
}

void CameraInfo::setCx(float new_cx) {
    this->cx_ = new_cx;
}

float CameraInfo::getCy() const {
    return this->cy_;
}

void CameraInfo::setCy(float new_cy){
    this->cy_ = new_cy;
}

float CameraInfo::getDistK0() const {
    return this->dist_k0_;
}

float CameraInfo::getDistK1() const {
    return this->dist_k1_;
}

float CameraInfo::getDistPx() const {
    return this->dist_px_;
}

float CameraInfo::getDistPy() const {
    return this->dist_py_;
}

float CameraInfo::getDistK2() const {
    return this->dist_k2_;
}

float CameraInfo::getDistK3() const {
    return this->dist_k3_;
}

float CameraInfo::getDistK4() const {
    return this->dist_k4_;
}

float CameraInfo::getDistK5() const {
    return this->dist_k5_;
}

int CameraInfo::getImageWidth() const {
    return this->img_width_;
}

int CameraInfo::getImageHeight() const {
    return this->img_height_;
}

cv::Mat CameraInfo::getCameraMatrix() const {
    cv::Mat camera_matrix = (cv::Mat_<float>(3,3) << this->fx_, 0, this->cx_, 0, this->fy_, this->cy_, 0, 0, 1);
    return camera_matrix;
}

void CameraInfo::setCameraMatrix(cv::Mat new_camera_matrix) {
    this->fx_ = new_camera_matrix.at<float>(0,0);
    this->fy_ = new_camera_matrix.at<float>(1,1);
    this->cx_ = new_camera_matrix.at<float>(0,2);
    this->cy_ = new_camera_matrix.at<float>(1,2);
}

cv::Mat CameraInfo::getDistCoeff() const {
    cv::Mat distortion_coeff = (cv::Mat_<float>(1,8) << this->dist_k0_, this->dist_k1_, this->dist_px_, this->dist_py_,
                                                        this->dist_k2_, this->dist_k3_, this->dist_k4_, this->dist_k5_);
    return distortion_coeff;
}

void CameraInfo::setDistCoeff(cv::Mat new_dist_coeff){
    this->dist_k0_ = new_dist_coeff.at<float>(0,0);
    this->dist_k1_ = new_dist_coeff.at<float>(0,1);
    this->dist_k2_ = new_dist_coeff.at<float>(0,2);
    this->dist_k3_ = new_dist_coeff.at<float>(0,3);
    this->dist_k4_ = new_dist_coeff.at<float>(0,4);
    this->dist_k5_ = new_dist_coeff.at<float>(0,5);
    this->dist_px_ = new_dist_coeff.at<float>(0,6);
    this->dist_py_ = new_dist_coeff.at<float>(0,7);
}

bool CameraInfo::setParameters(const std::string& intrinsic_path, const float resize_factor){
    // Check if the folder path exists
    if (isFolderNotEmpty(intrinsic_path)) {
        std::cout << "The file " << intrinsic_path << " exists and is not empty." << std::endl;

        try {
            YAML::Node intrinsic_config_file = YAML::LoadFile(intrinsic_path + "/intrinsic_pars_file.yaml");

            this->fx_ = intrinsic_config_file["fx"].as<float>()*resize_factor;
            this->fy_ = intrinsic_config_file["fy"].as<float>()*resize_factor;
            this->cx_ = intrinsic_config_file["cx"].as<float>()*resize_factor;
            this->cy_ = intrinsic_config_file["cy"].as<float>()*resize_factor;

            this->has_dist_coeff_ = intrinsic_config_file["has_dist_coeff"].as<int>();

            this->dist_k0_ = intrinsic_config_file["dist_k0"].as<float>();
            this->dist_k1_ = intrinsic_config_file["dist_k1"].as<float>();
            this->dist_px_ = intrinsic_config_file["dist_px"].as<float>();
            this->dist_py_ = intrinsic_config_file["dist_py"].as<float>();
            this->dist_k2_ = intrinsic_config_file["dist_k2"].as<float>();
            this->dist_k3_ = intrinsic_config_file["dist_k3"].as<float>();
            this->dist_k4_ = intrinsic_config_file["dist_k4"].as<float>();
            this->dist_k5_ = intrinsic_config_file["dist_k5"].as<float>();

            this->img_width_ = intrinsic_config_file["img_width"].as<int>()*resize_factor;
            this->img_height_ = intrinsic_config_file["img_height"].as<int>()*resize_factor;

            return true;
        } catch (const YAML::Exception &e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }
    else{
        return false;
    }
}

