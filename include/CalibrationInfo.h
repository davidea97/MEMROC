//
// Created by davide on 12/09/23.
//

#ifndef METRIC_CALIBRATOR_CALIBRATIONINFO_H
#define METRIC_CALIBRATOR_CALIBRATIONINFO_H

#include <string>

// Define a structure to store the data from the YAML file
class CalibrationInfo {
private:
    int number_of_cameras_;
    std::string camera_folder_prefix_;
    std::string pattern_type_;
    int number_of_rows_;
    int number_of_columns_;
    double size_;
    double resize_factor_;
    int visual_error_;
    int gt_;
    int metric_;
    int calibration_setup_;

public:
    CalibrationInfo();
    void setNumberOfCams(const int n);
    void setCamFolderPref(const std::string pref);
    void setPatternType(const std::string pattern_type);
    void setNumRow(const int n_row);
    void setNumCol(const int n_col);
    void setSize(const double size);
    void setResizeFactor(const double resize_factor);
    void setVisualError(const int visual_error);
    void setGt(const int gt);
    void setMetric(const int metric);
    void setCalibSetup(const int setup);
    const int getNumberOfCams() const;
    const std::string getCamFolderPref() const;
    const std::string getPatternType() const;
    const int getNumRow() const;
    const int getNumCol() const;
    const double getSize() const;
    const double getResizeFactor() const;
    const int getVisualError() const;
    const int getGt() const;
    const int getMetric() const;
    const int getCalibSetup() const;

    void printCalibInfo();
};

#endif //METRIC_CALIBRATOR_CALIBRATIONINFO_H
