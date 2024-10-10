//
// Created by davide on 12/09/23.
//
#ifndef MEMROC_CALIBRATOR_UTILS_H
#define MEMROC_CALIBRATOR_UTILS_H

#include <string>
#include <filesystem>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <random>
#include <opencv2/core/affine.hpp>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <algorithm>
#include <unordered_set>
#include "CameraInfo.h"
#include "CalibrationInfo.h"

struct Transformation {
    double tx, ty, tz; // Traslazione: x, y, z
    double rx, ry, rz; // Rotazione: x, y, z
};

struct LinearRegressionResult {
    double slope;
    double intercept;
};

bool isFolderNotEmpty(const std::string& folder_path);
bool compareFilenames(const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b);
void checkData(std::string data_folder, std::string prefix, int number_of_cameras);
void readPoseFromCSV(const std::string& input_path, cv::Mat& out_mat, char delim);
bool filterRansac(std::vector<cv::Point3f> object_points, std::vector<cv::Point2f> corners, CameraInfo camera_info, cv::Mat& rvec, cv::Mat& tvec);
void setInitialGuess(std::vector<cv::Mat> &h2e_initial_guess_vec, const std::vector<std::vector<cv::Mat>> rototras_vec, const std::vector<std::vector<cv::Mat>> correct_poses);
void createFolder(std::string folder_name);
int linesNumber(const std::string file_path);
Eigen::MatrixXd cvMatToEigen(const cv::Mat& cvMatrix);
void translationError(const cv::Mat& matrix1, const cv::Mat &matrix2, double& translation_error);
void rotationError(const cv::Mat& matrix1, const cv::Mat &matrix2, double& rotation_error);
void readGt(const int number_of_cameras, const std::string data_folder, std::vector<cv::Mat>& gt_vec, std::vector<std::vector<cv::Mat>>& gt_c2c_vec);
std::vector<double> errorMetric(const cv::Mat &gt, const cv::Mat &est);
void mat2Csv(const std::string& filename, const cv::Mat& matrix);
bool checkInnerVecSize(const std::vector<std::vector<cv::Point2f>>& vec);
std::string type2str(int type);
std::vector<std::vector<cv::Mat>> getRelativePoses(const std::vector<std::vector<cv::Mat>>& poses);
void savePose(int i, cv::Mat pose, std::string folder);
Eigen::Matrix4d parametersToMatrix(const std::vector<double>& params);
double handEyeObjective(const std::vector<double> &x, std::vector<double> &grad, void *data);
double handEyeObjectiveAXXB(const std::vector<double> &x, std::vector<double> &grad, void *data);
double dotProduct(const cv::Point3d& a, const cv::Point3d& b);
Eigen::Matrix3d computeRotationMatrix(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B);
Eigen::Matrix3d computeSvd(const Eigen::Matrix3d& H);
cv::Mat eigenToCvMat(const Eigen::MatrixXd& eigenMatrix);
void normalization(cv::Mat& input, cv::Mat& output);
void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale=1);
cv::Mat readPointsFromFile(const std::string& filename);
void reOrthogonalize(cv::Mat& R);
void data_augmentation(std::vector<cv::Mat> &robot_poses, std::vector<cv::Mat> pnp, std::vector<cv::Mat> &pnp_tras, std::vector<cv::Mat> &pnp_rot, cv::Mat svd,  const double std_dev, const double std_dev_t, std::vector<std::vector<int>> cross_obs);
void data_perturbation(std::vector<cv::Mat> &robot_poses, const double std_dev_tras, const double std_dev_rot, std::normal_distribution<> d_tras, std::normal_distribution<> d_rot, std::mt19937 gen);
void extractAndMatchSIFT(const std::vector<cv::Mat>& collected_images, std::vector<std::vector<cv::KeyPoint>>& keypoints1_vec, std::vector<std::vector<cv::KeyPoint>>& keypoints2_vec);

void triangulatePoints(const cv::Mat& rvec1, const cv::Mat& tvec1, const cv::Mat& rvec2, const cv::Mat& tvec2, const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& cameraMatrix, std::vector<cv::Point3d>& outputPoints);
void convertKeypointsToPoint2f(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points);
void plotWithPCL(const std::vector<cv::Point3d>& inputPoints, const cv::Mat svd);
cv::Mat readPointsFromTriangulation(const std::vector<cv::Point3d> triangulated_points);
double calculateInclination(const cv::Point3d vector);
void transVec2mat(std::vector<cv::Mat> trans_vec, std::vector<std::vector<int>> cross_observation_matrix, cv::Mat& trans_matrix, const int inversion, const int num_camera);
cv::Mat getSVD(cv::Mat data1, cv::Mat data2);
void findDeepMatches(const std::string& matchesFolderPath, std::vector<cv::Mat>& images_collected, std::vector<std::vector<cv::KeyPoint>>& keypoints1_vec, std::vector<std::vector<cv::KeyPoint>>& keypoints2_vec, int start_index, int end_index, double resize);
void metric_AX_XB(const std::vector<cv::Mat>& X, const std::vector<std::vector<cv::Mat>>& Ai, const std::vector<std::vector<cv::Mat>>& Bi, const std::vector<std::vector<int>>& cross_observation_matrix);
std::string getStringAfterLastSlash(const std::string& input);
int countImagesInFolder(const std::string& path);
cv::Mat rpyToRotationMatrix(const cv::Mat& rpy);
cv::Mat rotationMatrixToRpy(const cv::Mat& R);
cv::Mat computeRpyAngleError(const cv::Mat& rpy1, const cv::Mat& rpy2);
double trasError(const cv::Mat& translation_vec1, const cv::Mat &translation_vec2);
void appendTransformationsToFile(const std::string& filename, const std::string& datasetName, const int start_index, const int end_index, const std::vector<Transformation>& h2e, const std::vector<std::vector<Transformation>>& cam2cam);
cv::Vec3f rotationMatrixToRPY(const cv::Mat& R);
double averageRPYRotationError(const cv::Mat& mat1, const cv::Mat& mat2);
void saveTransformations(const std::string& filename,
                         const std::string& datasetName,
                         const std::vector<Transformation>& h2eTransformations,
                         const std::vector<std::vector<Transformation>>& cam2camTransformations);


LinearRegressionResult simpleLinearRegression(const std::vector<double>& x, const std::vector<double>& y);
void plotLinearRegression(const std::vector<double>& x, const std::vector<double>& y, const LinearRegressionResult& lrResult);
std::vector<int> removeOutliersFromLine(std::vector<double>& x, std::vector<double>& y, const LinearRegressionResult& lrResult, double threshold);


void plot3DTrajectories(const std::vector<cv::Mat>& robot2d, const std::vector<cv::Mat>& cam2d);
bool hasNonZero(const std::vector<std::vector<double>>& vectors, const std::vector<std::vector<int>>& cross_observation_matrix);
double computeTransError_AXXB(cv::Mat A, cv::Mat B, cv::Mat X);
std::vector<double> computeTransAverageError_AXXB(std::vector<std::vector<cv::Mat>> A, std::vector<std::vector<cv::Mat>> B, std::vector<cv::Mat> X);
void selectRandomSubsetAXXB_RANSAC(std::vector<std::vector<cv::Mat>>& selected_A, std::vector<std::vector<cv::Mat>>& selected_B, std::vector<std::vector<int>>& cross_observation_matrix_RANSAC, const int min_inliers, const int number_of_cameras, std::default_random_engine generator, std::uniform_int_distribution<size_t> distribution, const std::vector<std::vector<cv::Mat>> relative_robot_poses, const std::vector<std::vector<cv::Mat>> relative_cam_poses);
cv::Vec4d rotationMatrixToQuaternion(const cv::Mat &R);

void saveTranslationVectors(const std::vector<cv::Mat>& h2e_matrices, const std::vector<cv::Mat>& c2c_matrices, const std::string& filename);

void data_perturbation_camera(std::vector<cv::Mat> &camera_poses, const double std_dev_tras, const double std_dev_rot, std::vector<std::vector<int>> cross_observation_matrix, int cam);

void extractTranslationAndRotation(const cv::Mat& transformation_matrix, Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion);
std::vector<int> selectPoses(const std::vector<std::vector<int>>& cross_observation_matrix, int max_images_per_camera);
int countMultiCameraPoses(const std::vector<int>& selected_poses, const std::vector<std::vector<int>>& cross_observation_matrix);

template <typename _T>
void getRotoTras(cv::Mat rotation, cv::Mat translation, cv::Mat& G){
    G = (cv::Mat_<_T>(4,4) << rotation.at<_T>(0,0), rotation.at<_T>(0,1), rotation.at<_T>(0,2), translation.at<_T>(0),
            rotation.at<_T>(1,0), rotation.at<_T>(1,1), rotation.at<_T>(1,2), translation.at<_T>(1),
            rotation.at<_T>(2,0), rotation.at<_T>(2,1), rotation.at<_T>(2,2), translation.at<_T>(2),
            0.0, 0.0, 0.0, 1.0);
}

template <typename _T>
void getRoto(cv::Mat G, cv::Mat& rotation){
    rotation = (cv::Mat_<_T>(3,3) << G.at<_T>(0,0), G.at<_T>(0,1), G.at<_T>(0,2),
            G.at<_T>(1,0), G.at<_T>(1,1), G.at<_T>(1,2),
            G.at<_T>(2,0), G.at<_T>(2,1), G.at<_T>(2,2));
}

template <typename _T>
void getTras(cv::Mat G, cv::Mat& translation){
    translation = (cv::Mat_<_T>(1,3) << G.at<_T>(0,3), G.at<_T>(1,3), G.at<_T>(2,3));
}


template <typename _T>
cv::Mat rotationMatrixToEulerAngles(const cv::Mat &R) {
    assert(R.rows == 3 && R.cols == 3);

    _T sy = sqrt(R.at<_T>(0,0) * R.at<_T>(0,0) +  R.at<_T>(1,0) * R.at<_T>(1,0));

    bool singular = sy < 1e-6; // Se sy è vicino a zero, la direzione dell'asse z è vicino a singolarità

    _T x, y, z;
    if (!singular) {
        x = atan2(R.at<_T>(2,1), R.at<_T>(2,2));
        y = atan2(-R.at<_T>(2,0), sy);
        z = atan2(R.at<_T>(1,0), R.at<_T>(0,0));
    } else {
        x = atan2(-R.at<_T>(1,2), R.at<_T>(1,1));
        y = atan2(-R.at<_T>(2,0), sy);
        z = 0;
    }
    cv::Mat euler_mat = (cv::Mat_<_T>(1,3) << 0,0,0);
    euler_mat.at<_T>(0,0) = x;
    euler_mat.at<_T>(0,1) = y;
    euler_mat.at<_T>(0,2) = z;
    return euler_mat;
}

template <typename _T>
cv::Mat eulerAnglesToRotationMatrix(const cv::Mat& rvec) {

    cv::Mat R_x = (cv::Mat_<_T>(3, 3) <<
                                          1, 0, 0,
            0, cos(rvec.at<_T>(0)), -sin(rvec.at<_T>(0)),
            0, sin(rvec.at<_T>(0)), cos(rvec.at<_T>(0)));

    cv::Mat R_y = (cv::Mat_<_T>(3, 3) <<
                                          cos(rvec.at<_T>(1)), 0, sin(rvec.at<_T>(1)),
            0, 1, 0,
            -sin(rvec.at<_T>(1)), 0, cos(rvec.at<_T>(1)));

    cv::Mat R_z = (cv::Mat_<_T>(3, 3) <<
                                          cos(rvec.at<_T>(2)), -sin(rvec.at<_T>(2)), 0,
            sin(rvec.at<_T>(2)), cos(rvec.at<_T>(2)), 0,
            0, 0, 1);

    cv::Mat R = R_z * R_y * R_x;

    return R;
}

template <typename _T>
cv::Mat eulerAnglesToRotationMatrix(const std::vector<_T>& theta) {
    // Assicurati che il vettore theta abbia tre elementi
    assert(theta.size() == 6);

    // Calcola le matrici di rotazione intorno a ciascun asse
    cv::Mat R_x = (cv::Mat_<_T>(3, 3) <<
                                          1, 0, 0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0]));

    cv::Mat R_y = (cv::Mat_<_T>(3, 3) <<
                                          cos(theta[1]), 0, sin(theta[1]),
            0, 1, 0,
            -sin(theta[1]), 0, cos(theta[1]));

    cv::Mat R_z = (cv::Mat_<_T>(3, 3) <<
                                          cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]), cos(theta[2]), 0,
            0, 0, 1);

    // Combina le rotazioni in una singola matrice di rotazione
    cv::Mat R = R_z * R_y * R_x;
    cv::Mat t = (cv::Mat_<_T>(1,3) << theta[3], theta[4], theta[5]);
    cv::Mat G;
    getRotoTras<_T>(R,t,G);

    return G;
}

template <typename _T>
void addGaussianNoiseToElement(cv::Mat &matrix, int row, int col, _T mean, _T stddev) {
    // Create a single element matrix for the noise
    cv::Mat noise = cv::Mat::zeros(1, 1, matrix.type());
    cv::randn(noise, mean, stddev);

    // Add the noise to the specific element in the matrix
    matrix.at<_T>(row, col) += noise.at<_T>(0, 0);

}

template <typename _T>
cv::Mat addGaussianNoise(cv::Mat original_pose, _T tx, _T ty, _T tz, _T rx, _T ry, _T rz){
    cv::Mat rotation, tvec;
    getRoto<_T>(original_pose, rotation);
    getTras<_T>(original_pose, tvec);

    cv::Mat rvec, rotation_noisy;
    rvec = rotationMatrixToEulerAngles<_T>(rotation);

    // Define mean and standard deviation for each component
    _T means[3] = {0.0, 0.0, 0.0};
    _T stddevs_r[3] = {rx, ry, rz}; // Different stddev for each component
    _T stddevs_t[3] = {tx, ty, tz};

    // Add Gaussian noise to each component of translation and rotation matrices
    for (int k = 0; k < 3; ++k) {
        addGaussianNoiseToElement<_T>(tvec, 0, k, means[k], stddevs_t[k]);
        addGaussianNoiseToElement<_T>(rvec, 0, k, means[k], stddevs_r[k]);
    }

    //cv::Rodrigues(rvec, rotation_noisy);
    rotation_noisy = eulerAnglesToRotationMatrix<_T>(rvec);

    cv::Mat noisy_pose;
    getRotoTras<_T>(rotation_noisy, tvec, noisy_pose);

    return noisy_pose;
}



template <typename _T>
_T getAverage(std::vector<_T> vec){
    _T sum = 0;
    for (int i = 0; i < vec.size(); i++){
        sum += vec[i];
    }
    _T average = sum/vec.size();

    return average;
}


template <typename _T, int _ROWS, int _COLS>
void openCv2Eigen( const cv::Mat_<_T> &cv_mat,
                   Eigen::Matrix<_T, _ROWS, _COLS> &eigen_mat )
{
    for(int r = 0; r < _ROWS; r++)
        for(int c = 0; c < _COLS; c++)
            eigen_mat(r,c) = cv_mat(r,c);
}

template <typename _T, int _ROWS, int _COLS>
void eigen2openCv( const Eigen::Matrix<_T, _ROWS, _COLS> &eigen_mat,
                   cv::Mat_<_T> &cv_mat )
{
    cv_mat = cv::Mat_<_T>(_ROWS,_COLS);

    for(int r = 0; r < _ROWS; r++)
        for(int c = 0; c < _COLS; c++)
            cv_mat(r,c) = eigen_mat(r,c);
}

template <typename _T>
void rotMat2AngleAxis( const cv::Mat &r_mat, cv::Mat &r_vec )
{
    cv::Mat_<_T>tmp_r_mat(r_mat);
    cv::Rodrigues(tmp_r_mat, r_vec );
}

template <typename _T>
void rotMat2AngleAxis( const cv::Mat &r_mat, cv::Vec<_T, 3> &r_vec )
{
    cv::Mat_<_T>tmp_r_mat(r_mat), tmp_r_vec;
    cv::Rodrigues(tmp_r_mat, tmp_r_vec );
    r_vec[0] = tmp_r_vec(0); r_vec[1] = tmp_r_vec(1); r_vec[2] = tmp_r_vec(2);
}

template <typename _T>
void rotMat2AngleAxis( const Eigen::Matrix<_T, 3, 3> &r_mat, cv::Mat &r_vec )
{
    cv::Mat_<_T> tmp_r_mat;
    eigen2openCv<_T, 3, 3>(r_mat, tmp_r_mat);
    cv::Rodrigues(tmp_r_mat, r_vec );
}

template <typename _T>
void rotMat2AngleAxis( const Eigen::Matrix<_T, 3, 3> &r_mat, cv::Vec<_T, 3> &r_vec )
{
    cv::Mat_<_T> tmp_r_vec;
    rotMat2AngleAxis<_T>( r_mat, tmp_r_vec );
    r_vec[0] = tmp_r_vec(0); r_vec[1] = tmp_r_vec(1); r_vec[2] = tmp_r_vec(2);
}

template <typename _T>
void angleAxis2RotMat( const cv::Mat &r_vec, cv::Mat &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec);
    cv::Rodrigues(tmp_r_vec, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Vec<_T, 3> &r_vec, cv::Mat &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec);
    cv::Rodrigues(tmp_r_vec, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Mat &r_vec, Eigen::Matrix<_T, 3, 3> &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec), tmp_r_mat;
    cv::Rodrigues(tmp_r_vec, tmp_r_mat );
    openCv2Eigen<_T, 3, 3>( tmp_r_mat, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Vec<_T, 3> &r_vec, Eigen::Matrix<_T, 3, 3> &r_mat)
{
    cv::Mat_<_T>tmp_r_vec(r_vec), tmp_r_mat;
    cv::Rodrigues(tmp_r_vec, tmp_r_mat );
    openCv2Eigen<_T, 3, 3>( tmp_r_mat, r_mat );
}

template <typename _T>
void exp2TransfMat( const cv::Mat &r_vec, const cv::Mat &t_vec, cv::Mat &g_mat )
{
    cv::Mat_<_T> tmp_t_vec(t_vec), r_mat;
    angleAxis2RotMat<_T>( r_vec, r_mat );

    g_mat = (cv::Mat_< _T >(4, 4)
            << r_mat(0,0), r_mat(0,1), r_mat(0,2), tmp_t_vec(0,0),
            r_mat(1,0), r_mat(1,1), r_mat(1,2), tmp_t_vec(1,0),
            r_mat(2,0), r_mat(2,1), r_mat(2,2), tmp_t_vec(2,0),
            0,          0,          0,          1);

}
template <typename _T>
void transfMat2Exp( const cv::Mat &g_mat, cv::Mat &r_vec, cv::Mat &t_vec )
{
    cv::Mat_<_T> tmp_g_mat(g_mat);
    cv::Mat_< _T > r_mat = (cv::Mat_< _T >(3, 3)
            << tmp_g_mat(0,0), tmp_g_mat(0,1), tmp_g_mat(0,2),
            tmp_g_mat(1,0), tmp_g_mat(1,1), tmp_g_mat(1,2),
            tmp_g_mat(2,0), tmp_g_mat(2,1), tmp_g_mat(2,2));
    rotMat2AngleAxis<_T>(r_mat, r_vec);
    t_vec = (cv::Mat_< _T >(3, 1)<<tmp_g_mat(0,3), tmp_g_mat(1,3), tmp_g_mat(2,3));
}


struct OptimizationData {
    std::vector<std::pair<cv::Mat, cv::Mat>> transformPairs; // Coppie di matrici A e B
    Eigen::Matrix4d X_fixed; // Parte fissa di X, esclusa tz
    Eigen::Matrix4d Z_fixed; // Parte fissa di Z, esclusa tz
};



#endif //METRIC_CALIBRATOR_UTILS_H
