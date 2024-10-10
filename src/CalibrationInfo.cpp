//
// Created by davide on 27/12/23.
//
#include <iostream>

#include "CalibrationInfo.h"

CalibrationInfo::CalibrationInfo() {}

void CalibrationInfo::setNumberOfCams(const int n){
    this->number_of_cameras_ = n;
}

void CalibrationInfo::setCamFolderPref(const std::string pref){
    this->camera_folder_prefix_ = pref;
}

void CalibrationInfo::setPatternType(const std::string pattern_type){
    this->pattern_type_ = pattern_type;
}

void CalibrationInfo::setNumRow(const int n_row){
    this->number_of_rows_ = n_row;
}

void CalibrationInfo::setNumCol(const int n_col){
    this->number_of_columns_ = n_col;
}

void CalibrationInfo::setSize(const double size){
    this->size_ = size;
}

void CalibrationInfo::setResizeFactor(const double resize_factor){
    this->resize_factor_ = resize_factor;
}

void CalibrationInfo::setVisualError(const int visual_error){
    this->visual_error_ = visual_error;
}

void CalibrationInfo::setGt(const int gt){
    this->gt_ = gt;
}

void CalibrationInfo::setMetric(const int metric){
    this->metric_ = metric;
}

void CalibrationInfo::setCalibSetup(const int setup){
    this->calibration_setup_ = setup;
}

const int CalibrationInfo::getNumberOfCams() const {
    return this->number_of_cameras_;
}

const std::string CalibrationInfo::getCamFolderPref() const {
    return this->camera_folder_prefix_;
}

const std::string CalibrationInfo::getPatternType() const {
    return this->pattern_type_;
}

const int CalibrationInfo::getNumRow() const {
    return this->number_of_rows_;
}

const int CalibrationInfo::getNumCol() const {
    return this->number_of_columns_;
}

const double CalibrationInfo::getSize() const {
    return this->size_;
}

const double CalibrationInfo::getResizeFactor() const {
    return this->resize_factor_;
}

const int CalibrationInfo::getVisualError() const {
    return this->visual_error_;
}

const int CalibrationInfo::getGt() const {
    return this->gt_;
}

const int CalibrationInfo::getMetric() const {
    return this->metric_;
}

const int CalibrationInfo::getCalibSetup() const {
    return this->calibration_setup_;
}



void CalibrationInfo::printCalibInfo() {
    std::cout << "------------- Calibration parameters -------------" << std::endl;
    std::cout << "Calibration type: " ;
    switch (this->calibration_setup_){
        case 0:
            std::cout << "Eye-in-hand" << std::endl;
            break;
        case 1:
            std::cout << "Eye-on-base" << std::endl;
            break;
        case 2:
            std::cout << "Stereo mobile" << std::endl;
            break;
    }
    std::cout << "Number of cameras: " << this->number_of_cameras_ << std::endl;
    std::cout << "Patterns specifics: " << std::endl;
    std::cout << "  - Type: " << this->pattern_type_ << std::endl;
    std::cout << "  - Rows: " << this->number_of_rows_ << std::endl;
    std::cout << "  - Cols: " << this->number_of_columns_ << std::endl;
    std::cout << "  - Size: " << this->size_ << std::endl;
}