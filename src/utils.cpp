//
// Created by davide on 12/09/23.
//

#include <fstream>
#include "utils.h"

namespace fs = std::filesystem;

bool isFolderNotEmpty(const std::string& folder_path) {
    if (!fs::exists(folder_path)) {
        // The folder does not exist
        std::cerr << "The folder does not exist!" << std::endl;
        return false;
    }

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        // If there is at least one entry in the folder, it's not empty
        return true;
    }

    // The folder is empty
    std::cerr << "The folder is empty!" << std::endl;
    return false;
}

bool compareFilenames(const fs::directory_entry& a, const fs::directory_entry& b) {
    return a.path().filename().string() < b.path().filename().string();
}

void checkData(std::string data_folder, std::string prefix, int number_of_cameras){
    int folder_count = 0;
    for (const auto& entry: fs::directory_iterator(data_folder)){
        if (entry.path().filename().string().find(prefix) == 0){
            folder_count ++;
        }
    }

    // Stop the program if the number of selected cameras is different from the number of provided folders
    if (number_of_cameras != folder_count) {
        std::cerr << "The number of selected cameras does not coincide with the number of provided folders!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void readPoseFromCSV(const std::string& input_path, cv::Mat& out_mat, char delim)
{
    std::ifstream inputfile(input_path);
    std::string current_line;
    std::vector<std::vector<float> > all_data;
    while(getline(inputfile, current_line)){
        std::vector<float> values;
        std::stringstream temp(current_line);
        std::string single_value;
        while(getline(temp,single_value,delim)){
            float f = std::stof(single_value.c_str());
            values.push_back(f);
        }
        all_data.push_back(values);
    }

    out_mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_32FC1);
    for(int rows = 0; rows < (int)all_data.size(); rows++){
        for(int cols= 0; cols< (int)all_data[0].size(); cols++){
            out_mat.at<float>(rows,cols) = all_data[rows][cols];
        }
    }
}

bool filterRansac(std::vector<cv::Point3f> object_points, std::vector<cv::Point2f> corners, CameraInfo camera_info, cv::Mat& rvec, cv::Mat& tvec){

    std::vector<int> inliers;
    cv::solvePnPRansac(object_points, corners, camera_info.getCameraMatrix(),camera_info.getDistCoeff(), rvec, tvec, false, 100, 15, 0.99, inliers);
    if (inliers.size() == object_points.size())
        return true;
    else
        return false;
}

void setInitialGuess(std::vector<cv::Mat> &h2e_initial_guess_vec, const std::vector<std::vector<cv::Mat>> rototras_vec, const std::vector<std::vector<cv::Mat>> correct_poses){

    for (int i = 0; i < h2e_initial_guess_vec.size(); i++) {
        h2e_initial_guess_vec[i] = (cv::Mat_<double>(4, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);

        cv::Mat double_matrix1, double_matrix2;
        rototras_vec[i][0].convertTo(double_matrix1, CV_64F);
        correct_poses[i][0].convertTo(double_matrix2, CV_64F);
        h2e_initial_guess_vec[i] = double_matrix1 * double_matrix2.inv();
    }
};

void createFolder(std::string folder_name){
    if (!fs::is_directory(folder_name)) {
        try {
            fs::create_directories(folder_name);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error creating the folder: " << e.what() << std::endl;
        }
    }
}

int linesNumber(const std::string file_path){
    std::vector<std::string> existing_data;
    std::ifstream input_file(file_path);
    if (input_file.is_open()) {
        std::string line;
        while (std::getline(input_file, line)) {
            existing_data.push_back(line);
        }
        input_file.close();
    }

    return existing_data.size();
}

Eigen::MatrixXd cvMatToEigen(const cv::Mat& cvMatrix) {
    Eigen::MatrixXd eigenMatrix(cvMatrix.rows, cvMatrix.cols);
    for (int i = 0; i < cvMatrix.rows; i++) {
        for (int j = 0; j < cvMatrix.cols; j++) {
            eigenMatrix(i, j) = cvMatrix.at<double>(i, j);
        }
    }
    return eigenMatrix;
}

void translationError(const cv::Mat& matrix1, const cv::Mat &matrix2, double& translation_error){

    // Extract translation vectors from the matrices (using OpenCV)
    cv::Mat translation_vec1 = matrix1(cv::Rect(3, 0, 1, 3));
    cv::Mat translation_vec2 = matrix2(cv::Rect(3, 0, 1, 3));

    // Calculate the translation error (Euclidean distance) using OpenCV
    cv::Mat translation_diff;
    translation_vec1.convertTo(translation_vec1, CV_64F);
    translation_vec2.convertTo(translation_vec2, CV_64F);

    cv::absdiff(translation_vec1, translation_vec2, translation_diff);
    translation_error = cv::norm(translation_diff, cv::NORM_L2);
}

void rotationError(const cv::Mat& matrix1, const cv::Mat& matrix2, double& rotation_error) {
    // Extract the 3x3 rotation components from the 4x4 transformation matrices
    cv::Mat rotation_mat1 = matrix1(cv::Rect(0, 0, 3, 3));
    cv::Mat rotation_mat2 = matrix2(cv::Rect(0, 0, 3, 3));

    rotation_mat1.convertTo(rotation_mat1, CV_64F);
    rotation_mat2.convertTo(rotation_mat2, CV_64F);
    // Compute the relative rotation matrix
    cv::Mat rel_mat = rotation_mat1 * rotation_mat2.t(); // Use transpose instead of inverse for rotation matrices

    // Convert the OpenCV matrix to an Eigen matrix
    Eigen::Matrix3d eigen_rel_mat = cvMatToEigen(rel_mat);
    //std::cout << "Eigen relative rotation matrix: " << eigen_rel_mat << std::endl;

    // Use Eigen to compute the angle of the axis-angle representation
    Eigen::AngleAxisd angleAxis(eigen_rel_mat);
    double angle = angleAxis.angle(); // This is the rotation error in radians

    // Optionally, convert the angle to degrees
    rotation_error = angle * (180.0 / M_PI);
}

cv::Vec3f rotationMatrixToRPY(const cv::Mat& R) {
    cv::Vec3f euler_angles;
    cv::Mat mtxR, mtxQ;
    cv::Vec3d out_angles;
    cv::Rodrigues(R, mtxR);  
    cv::RQDecomp3x3(mtxR, mtxR, mtxQ, out_angles); 
    euler_angles = out_angles;
    return euler_angles;
}
double averageRPYRotationError(const cv::Mat& mat1, const cv::Mat& mat2) {
    CV_Assert(mat1.size() == cv::Size(4, 4) && mat2.size() == cv::Size(4, 4));  

    cv::Mat R1 = mat1(cv::Rect(0, 0, 3, 3));
    cv::Mat R2 = mat2(cv::Rect(0, 0, 3, 3));

    cv::Vec3f rpy1 = rotationMatrixToRPY(R1);
    cv::Vec3f rpy2 = rotationMatrixToRPY(R2);

    cv::Vec3f diff = rpy1 - rpy2;
    double error = cv::norm(diff) / 3.0;

    return error;
}


Eigen::Matrix4d parametersToMatrix(const std::vector<double>& params) {
    Eigen::Matrix3d m;

    m = Eigen::AngleAxisd(params[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(params[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(params[2], Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = m;
    T(0, 3) = params[3];
    T(1, 3) = params[4];
    T(2, 3) = params[5];
    return T;
}

void readGt(const int number_of_cameras, const std::string data_folder, std::vector<cv::Mat>& gt_vec, std::vector<std::vector<cv::Mat>>& gt_c2c_vec){
    for (int i = 0; i < number_of_cameras; i++) {
        std::string gt_path = data_folder + "/GT/gt_cam" + std::to_string(i + 1) + ".csv";
        readPoseFromCSV(gt_path, gt_vec[i], ' ');
    }

    for (int i = 0; i < number_of_cameras; i++){
        for (int j = 0; j < number_of_cameras; j++){
            if (i!=j)
                gt_c2c_vec[i][j] = gt_vec[i].inv()*gt_vec[j];
        }
    }
}

std::vector<double> errorMetric(const cv::Mat &gt, const cv::Mat &est){

    std::vector<double> metric(2);
    double translation_error, rotation_error;
    translationError(gt, est, translation_error);
    rotationError(gt, est, rotation_error);
    metric[0] = translation_error;
    metric[1] = rotation_error;

    return metric;
}

void mat2Csv(const std::string& filename, const cv::Mat& matrix) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Unable to open the file." << std::endl;
        return;
    }

    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            file << matrix.at<double>(i, j);
            if (j < matrix.cols - 1) {
                file << ' '; // Add a comma to separate values
            }
        }
        file << '\n'; // Add a newline character to separate rows
    }

    file.close();
}

void extractTranslationAndRotation(const cv::Mat& transformation_matrix, Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion) {
    // Check if the input is a 4x4 matrix
    if (transformation_matrix.rows != 4 || transformation_matrix.cols != 4) {
        throw std::invalid_argument("Input matrix must be 4x4.");
    }

    // Extract the rotation matrix (3x3 part)
    cv::Mat rotation_matrix = transformation_matrix(cv::Range(0, 3), cv::Range(0, 3));

    // Extract the translation vector (3x1 part)
    cv::Mat translation_vector = transformation_matrix(cv::Range(0, 3), cv::Range(3, 4));

    // Convert OpenCV matrix to Eigen matrix
    Eigen::Matrix3d eigen_rotation;
    eigen_rotation = cvMatToEigen(rotation_matrix);

    // Convert OpenCV vector to Eigen vector
    translation = Eigen::Vector3d(translation_vector.at<double>(0, 0), translation_vector.at<double>(1, 0), translation_vector.at<double>(2, 0));

    // Convert rotation matrix to quaternion
    quaternion = Eigen::Quaterniond(eigen_rotation);

    std::cout << "Translation: " << translation << std::endl;
    std::cout << "Rotation: " << quaternion.coeffs() << std::endl;
}

bool checkInnerVecSize(const std::vector<std::vector<cv::Point2f>>& vec) {
    if (vec.empty()) {
        return true; // An empty vector technically has elements of the same size
    }

    // Get the size of the first inner vector
    size_t size = vec[0].size();

    // Iterate through the rest of the inner vectors and compare their sizes
    for (size_t i = 1; i < vec.size(); i++) {
        if (vec[i].size() != size) {
            return false; // Inner vectors have different sizes
        }
    }

    return true; // All inner vectors have the same size
}


std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}


std::vector<std::vector<cv::Mat>> getRelativePoses(const std::vector<std::vector<cv::Mat>>& poses){
    int num_of_cameras = poses.size();
    std::vector<std::vector<cv::Mat>> relative_poses(num_of_cameras);
    for (int i = 0; i < num_of_cameras; i++){
        int number_of_poses = poses[i].size();
        std::vector<cv::Mat> single_relative_pose;
        for (int j = 1; j < number_of_poses; j++){
            cv::Mat relative_pose = poses[i][j-1].inv()*poses[i][j];
            single_relative_pose.push_back(relative_pose);
        }
        relative_poses[i] = single_relative_pose;
    }

    return relative_poses;
}


void savePose(int i, cv::Mat pose, std::string folder){
    std::stringstream fileName;
    fileName << std::setw(4) << std::setfill('0') << i << ".csv";

    // Open the file
    std::cout <<"File: " << folder + "/optimal_pose/" + fileName.str() << std::endl;
    std::ofstream file(folder + "/optimal_pose/" + fileName.str());
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << fileName.str() << std::endl;
    }
    // Generate and write a 4x4 matrix to the file
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            file << pose.at<double>(row, col); // Example value for the matrix element
            if (col < 3) file << " "; // Comma for separating values
        }
        file << "\n"; // Newline at the end of each row
    }

    // Close the file
    file.close();
}



double dotProduct(const cv::Point3d& a, const cv::Point3d& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

cv::Mat eigenToCvMat(const Eigen::MatrixXd& eigenMatrix) {
    cv::Mat cvMatrix(eigenMatrix.rows(), eigenMatrix.cols(), CV_64F);

    for (int i = 0; i < eigenMatrix.rows(); ++i) {
        for (int j = 0; j < eigenMatrix.cols(); ++j) {
            cvMatrix.at<double>(i, j) = eigenMatrix(i, j);
        }
    }

    return cvMatrix;
}



Eigen::Matrix3d computeRotationMatrix(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    // Compute the cross-covariance matrix
    Eigen::Matrix3d H = A * B.transpose();

    // Perform SVD on the cross-covariance matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Compute the rotation matrix
    Eigen::Matrix3d R = V * U.transpose();

    // Correcting the rotation matrix to ensure a right-handed coordinate system
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    return R;
}

Eigen::Matrix3d computeSvd(const Eigen::Matrix3d& H) {

    // Perform SVD on the cross-covariance matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Compute the rotation matrix
    Eigen::Matrix3d R = V * U.transpose();

    // Correcting the rotation matrix to ensure a right-handed coordinate system
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    return R;
}

void normalization(cv::Mat& input, cv::Mat& output) {
    output = input.clone();
    int rows = input.rows;
    int cols = input.cols;

    for (int i = 0; i < cols; i++) {
        cv::Scalar mean;
        mean = cv::mean(input.col(i));

        for (int j = 0; j < rows; j++) {
            output.at<double>(j, i) = (input.at<double>(j, i) - mean[0]);
        }
    }
}


void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale) {
    double angle = atan2((double)p.y - q.y, (double)p.x - q.x); 
    double hypotenuse = sqrt((double)((p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x)));

    q.x = (int)(p.x - scale * hypotenuse * cos(angle));
    q.y = (int)(p.y - scale * hypotenuse * sin(angle));

    cv::line(img, p, q, colour, 1, cv::LINE_AA);

    p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 1, cv::LINE_AA);

    p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
    cv::line(img, p, q, colour, 1, cv::LINE_AA);
}


cv::Mat readPointsFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::vector<cv::Point3f> points;

    // Check if the file is opened successfully
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double x, y, z;

        if (!(iss >> x >> y >> z)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue; // Skip malformed line
        }

        // Add the point to the vector
        points.push_back(cv::Point3d(x, y, z));
    }

    // Convert vector of points to cv::Mat
    cv::Mat pointsMat(points.size(), 3, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
        pointsMat.at<double>(i, 0) = points[i].x;
        pointsMat.at<double>(i, 1) = points[i].y;
        pointsMat.at<double>(i, 2) = points[i].z;
    }

    return pointsMat;
}

void reOrthogonalize(cv::Mat& R) {
    // Assuming R is 3x3
    cv::Vec3d x(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0));
    cv::Vec3d y(R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1));
    cv::Vec3d z;

    // Gram-Schmidt process
    x = cv::normalize(x); // Normalize the first vector
    y = y - x.dot(y) * x; // Make y orthogonal to x
    y = cv::normalize(y); // Normalize y
    z = x.cross(y); // Compute the third vector as the cross product of x and y

    // Update matrix R
    R.at<double>(0, 0) = x[0]; R.at<double>(1, 0) = x[1]; R.at<double>(2, 0) = x[2];
    R.at<double>(0, 1) = y[0]; R.at<double>(1, 1) = y[1]; R.at<double>(2, 1) = y[2];
    R.at<double>(0, 2) = z[0]; R.at<double>(1, 2) = z[1]; R.at<double>(2, 2) = z[2];
}

void data_augmentation(std::vector<cv::Mat> &robot_poses, std::vector<cv::Mat> pnp, std::vector<cv::Mat> &pnp_tras, std::vector<cv::Mat> &pnp_rot, cv::Mat svd,  const double std_dev, const double std_dev_t, std::vector<std::vector<int>> cross_obs){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, std_dev); // Gaussian distribution with 0 mean and std_dev standard deviation
    //std::normal_distribution<> d_tras(0, std_dev_t);

    int counter = 0;
    for (auto& pose : robot_poses) {
        if (cross_obs[counter][0]) {
            if (pose.rows != 4 || pose.cols != 4 || pose.type() != CV_64F) {
                throw std::runtime_error("Each pose must be a 4x4 matrix of type double.");
            }
            std::cout << "######################################################" << std::endl;
            std::cout << "Original robot pose: " << pose << std::endl;
            // Generate random Gaussian noise for rotation around X and Y axes
            double noiseX = d(gen);
            double noiseY = d(gen);

            cv::Mat A_rotation, B_inv_rotation;
            getRoto<double>(pose, A_rotation);

            // Extract the rotation matrix
            cv::Mat euler_angles = rotationMatrixToEulerAngles<double>(A_rotation);
            euler_angles.at<double>(0) = euler_angles.at<double>(0) + noiseX;
            euler_angles.at<double>(1) = euler_angles.at<double>(1) + noiseY;
            cv::Mat A_rotation_noisy = eulerAnglesToRotationMatrix<double>(euler_angles);

            // Ensure determinant is 1 for a proper rotation matrix
            double det = cv::determinant(A_rotation_noisy);
            if (std::abs(det - 1.0) > 1e-6) { // Check if determinant is not close to 1
                if (det != 0) {                                 // Avoid division by zero
                    A_rotation_noisy /= std::cbrt(det);      // Scale the matrix to make the determinant 1
                }
            }

            // Copy the corrected rotation matrix back to the pose
            A_rotation_noisy.copyTo(pose(cv::Rect(0, 0, 3, 3)));
            std::cout << "Robot pose perturbated: " << pose << std::endl;
        }
        counter ++;
    }
}


void data_perturbation(std::vector<cv::Mat> &robot_poses, const double std_dev_tras, const double std_dev_rot, std::normal_distribution<> d_tras, std::normal_distribution<> d_rot, std::mt19937 gen){



    int counter = 0;
    for (auto& pose : robot_poses) {

        if (pose.rows != 4 || pose.cols != 4 || pose.type() != CV_64F) {
            throw std::runtime_error("Each pose must be a 4x4 matrix of type double.");
        }

        std::cout << "######################################################" << std::endl;
        double noiseX_tras = d_tras(gen);
        double noiseY_tras = d_tras(gen);
        //double noiseZ_tras = d_tras(gen);

        //double noiseX_rot = d_rot(gen);
        //double noiseY_rot = d_rot(gen);
        double noiseZ_rot = d_rot(gen);
        std::cout <<"Z yaw error: " << noiseZ_rot << std::endl;

        cv::Mat A_rotation;
        getRoto<double>(pose, A_rotation);

        // Extract the rotation matrix
        cv::Mat euler_angles = rotationMatrixToEulerAngles<double>(A_rotation);
        euler_angles.at<double>(0) = euler_angles.at<double>(0);
        euler_angles.at<double>(1) = euler_angles.at<double>(1);
        euler_angles.at<double>(2) = euler_angles.at<double>(2) + noiseZ_rot;

        cv::Mat A_rotation_noisy = eulerAnglesToRotationMatrix<double>(euler_angles);

        // Ensure determinant is 1 for a proper rotation matrix
        double det = cv::determinant(A_rotation_noisy);
        if (std::abs(det - 1.0) > 1e-6) { // Check if determinant is not close to 1
            if (det != 0) {                                 // Avoid division by zero
                A_rotation_noisy /= std::cbrt(det);      // Scale the matrix to make the determinant 1
            }
        }


        // Copy the corrected rotation matrix back to the pose
        A_rotation_noisy.copyTo(pose(cv::Rect(0, 0, 3, 3)));

        pose.at<double>(0,3) = pose.at<double>(0,3) + noiseX_tras;
        pose.at<double>(1,3) = pose.at<double>(1,3) + noiseY_tras;
        pose.at<double>(2,3) = pose.at<double>(2,3);

        counter++;
    }
}

void data_perturbation_camera(std::vector<cv::Mat> &camera_poses, const double std_dev_tras, const double std_dev_rot, std::vector<std::vector<int>> cross_observation_matrix, int cam){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_tras(0, std_dev_tras); // Gaussian distribution with 0 mean and std_dev standard deviation
    std::normal_distribution<> d_rot(0, std_dev_rot);

    int counter = 0;
    for (auto& pose : camera_poses) {
        if (cross_observation_matrix[counter][cam] && !pose.empty() ) {
            if (pose.rows != 4 || pose.cols != 4 || pose.type() != CV_64F) {
                throw std::runtime_error("Each pose must be a 4x4 matrix of type double.");
            }

            std::cout << "######################################################" << std::endl;
            std::cout << "Original camera pose: " << pose << std::endl;
            double noiseX_tras = d_tras(gen);
            double noiseY_tras = d_tras(gen);
            double noiseZ_tras = d_tras(gen);

            double noiseX_rot = d_rot(gen);
            double noiseY_rot = d_rot(gen);
            double noiseZ_rot = d_rot(gen);
            std::cout << "Z yaw error: " << noiseZ_rot << std::endl;

            cv::Mat A_rotation;
            getRoto<double>(pose, A_rotation);

            // Extract the rotation matrix
            cv::Mat euler_angles = rotationMatrixToEulerAngles<double>(A_rotation);
            euler_angles.at<double>(0) = euler_angles.at<double>(0);
            euler_angles.at<double>(1) = euler_angles.at<double>(1);
            euler_angles.at<double>(2) = euler_angles.at<double>(2) + noiseZ_rot;

            cv::Mat A_rotation_noisy = eulerAnglesToRotationMatrix<double>(euler_angles);

            // Ensure determinant is 1 for a proper rotation matrix
            double det = cv::determinant(A_rotation_noisy);
            if (std::abs(det - 1.0) > 1e-6) { // Check if determinant is not close to 1
                if (det != 0) {                                 // Avoid division by zero
                    A_rotation_noisy /= std::cbrt(det);      // Scale the matrix to make the determinant 1
                }
            }


            // Copy the corrected rotation matrix back to the pose
            A_rotation_noisy.copyTo(pose(cv::Rect(0, 0, 3, 3)));

            pose.at<double>(0, 3) = pose.at<double>(0, 3) + noiseX_tras;
            pose.at<double>(1, 3) = pose.at<double>(1, 3) + noiseY_tras;
            pose.at<double>(2, 3) = pose.at<double>(2, 3);

            std::cout << "Robot pose perturbated: " << pose << std::endl;


        }
        counter++;
    }
}

// Helper function to convert keypoints to Point2f
void convertKeypointsToPoint2f(const std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points) {
    points.clear();
    for (const auto& kp : keypoints) {
        points.push_back(kp.pt);
    }
}


void triangulatePoints(
        const cv::Mat& rvec1,
        const cv::Mat& tvec1,
        const cv::Mat& rvec2,
        const cv::Mat& tvec2,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        const cv::Mat& cameraMatrix, // Camera intrinsic matrix
        std::vector<cv::Point3d>& outputPoints) // Output 3D points
{

    // Convert keypoints to Point2f
    std::vector<cv::Point2f> points1, points2;
    convertKeypointsToPoint2f(keypoints1, points1);
    convertKeypointsToPoint2f(keypoints2, points2);

    // The rest of the function remains the same as the previous example...
    // Convert the first rotation vector to a rotation matrix
    cv::Mat R1;
    cv::Rodrigues(rvec1, R1);
    cv::Mat P1_temp;
    cv::hconcat(R1, tvec1, P1_temp);

    cv::Mat K;
    cameraMatrix.convertTo(K, CV_64FC1);
    cv::Mat P1 = K * P1_temp; // Projection matrix for the first camera

    // Assuming rvec_all and tvec_all refer to the transformation from the first to the second camera
    cv::Mat R2, P2;

    cv::Rodrigues(rvec2, R2); // Convert the second rotation vector to a rotation matrix
    cv::hconcat(R2, tvec2, P2); // Construct [R|t] for the second camera

    P2 = K * P2; // Projection matrix for the second camera

    cv::Mat points4D; // Homogeneous coordinates of points
    cv::triangulatePoints(P1, P2, points1, points2, points4D);

    // Convert homogeneous coordinates to 3D points
    for (int i = 0; i < points4D.cols; i++) {
        cv::Mat col = points4D.col(i);
        col /= col.at<float>(3); // Normalize by the last element
        outputPoints.push_back(cv::Point3d(col.at<float>(0), col.at<float>(1), col.at<float>(2)));
    }
}

cv::Mat readPointsFromTriangulation(const std::vector<cv::Point3d> triangulated_points) {

    std::vector<cv::Point3d> points;
    for (int i = 0; i<triangulated_points.size(); i++){
        double x, y, z;

        // Add the point to the vector
        points.push_back(cv::Point3d(triangulated_points[i].x, triangulated_points[i].y, triangulated_points[i].z));
    }

    // Convert vector of points to cv::Mat
    cv::Mat pointsMat(points.size(), 3, CV_64F);
    for (size_t i = 0; i < points.size(); ++i) {
        pointsMat.at<double>(i, 0) = points[i].x;
        pointsMat.at<double>(i, 1) = points[i].y;
        pointsMat.at<double>(i, 2) = points[i].z;
    }

    return pointsMat;
}


double calculateInclination(const cv::Point3d vector) {
    // Vettore di riferimento [0, 0, 1]
    cv::Point3d referenceVector(0, 0, 1);

    // Calcola il prodotto scalare tra il vettore dato e il vettore di riferimento
    double dotProduct = vector.x * referenceVector.x + vector.y * referenceVector.y + vector.z * referenceVector.z;

    // Calcola la norma (lunghezza) dei due vettori
    double normVector = std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
    double normReferenceVector = std::sqrt(referenceVector.x * referenceVector.x + referenceVector.y * referenceVector.y + referenceVector.z * referenceVector.z);

    // Calcola l'angolo in radianti usando il prodotto scalare e le norme dei vettori
    double angleRadians = std::acos(dotProduct / (normVector * normReferenceVector));

    // Converti l'angolo in gradi
    double angleDegrees = angleRadians * (180.0 / CV_PI);

    return angleDegrees;
}

void transVec2mat(std::vector<cv::Mat> trans_vec, std::vector<std::vector<int>> cross_observation_matrix, cv::Mat& trans_matrix, const int inversion, const int num_camera){
    int counter = 0;
    for(size_t i = 0; i < trans_vec.size(); ++i) {
        if (cross_observation_matrix[i][num_camera]) {
            cv::Mat pnp_temp;
            if (inversion) {
                pnp_temp = trans_vec[i].inv();
            }
            else{
                pnp_temp = trans_vec[i];
            }
            cv::Mat translation = pnp_temp.col(3).rowRange(0, 3);
            trans_matrix.at<double>(counter, 0) = translation.at<double>(0);
            trans_matrix.at<double>(counter, 1) = translation.at<double>(1);
            trans_matrix.at<double>(counter, 2) = translation.at<double>(2);
            counter += 1;
        }
    }
}

cv::Mat getSVD(cv::Mat data1, cv::Mat data2){
    cv::Mat data1_normalized, data2_normalized;
    normalization(data1, data1_normalized);
    normalization(data2, data2_normalized);


    Eigen::MatrixXd A = cvMatToEigen(data1_normalized);
    Eigen::MatrixXd B = cvMatToEigen(data2_normalized);
    Eigen::Matrix3d H = B.transpose()*A;

    // Compute the svd
    Eigen::Matrix3d R = computeSvd(H);
    cv::Mat svd_mat = eigenToCvMat(R);

    return svd_mat;
}


void findDeepMatches(const std::string& matchesFolderPath, std::vector<cv::Mat>& images_collected, std::vector<std::vector<cv::KeyPoint>>& keypoints1_vec, std::vector<std::vector<cv::KeyPoint>>& keypoints2_vec, int start_index, int end_index, double resize) {
    std::vector<fs::path> sortedEntries;

    // Collect all .txt files
    for (const auto& entry : fs::directory_iterator(matchesFolderPath)) {
        if (entry.path().extension() == ".txt") {
            sortedEntries.push_back(entry.path());
        }
    }

    // Sort the entries based on the filename
    std::sort(sortedEntries.begin(), sortedEntries.end(), [](const fs::path& a, const fs::path& b) {
        return a.filename().string() < b.filename().string();
    });

    int counter1 = 0;
    int counter2 = 1;
    if (end_index > sortedEntries.size()){
        end_index = sortedEntries.size();
    }
    for (int j = start_index; j < end_index; ++j) {
        const auto& entryPath = sortedEntries[j];
        int index1, index2;
        std::sscanf(entryPath.filename().string().c_str(), "%d_%d.txt", &index1, &index2);
        std::ifstream file(entryPath);
        std::vector<cv::Point2f> points1, points2;
        std::string line;

        while (std::getline(file, line)) {
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream iss(line);
            float x1, y1, x2, y2;
            if (iss >> x1 >> y1 >> x2 >> y2) {
                points1.emplace_back(x1*resize, y1*resize);
                points2.emplace_back(x2*resize, y2*resize);
            }
        }

        file.close();

        //std::cout << "Found " << points1.size() << " matches in file " << entryPath.filename() << std::endl;

        if (counter1 < images_collected.size() && counter2 < images_collected.size()) {
            std::vector<cv::KeyPoint> keypoints1, keypoints2;

            for (size_t i = 0; i < points1.size(); ++i) {
                keypoints1.emplace_back(points1[i], 1.f);
                keypoints2.emplace_back(points2[i], 1.f);
            }

            keypoints1_vec[counter1] = keypoints1;
            keypoints2_vec[counter2] = keypoints2;
            counter1 ++;
            counter2 ++;
        } else {
            std::cerr << "Image indices out of bounds: " << index1 << ", " << index2 << std::endl;
        }
    }
}





void saveTransformations(const std::string& filename,
                         const std::string& datasetName,
                         const std::vector<Transformation>& h2eTransformations,
                         const std::vector<std::vector<Transformation>>& cam2camTransformations) {
    // Apri il file in modalità append
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Impossibile aprire il file per la scrittura." << std::endl;
        return;
    }

    // Scrivi nel file come prima, ma con il file aperto in modalità append
    file << "\n" << datasetName << std::endl;
    file << "h2e:" << std::endl;
    for (size_t i = 0; i < h2eTransformations.size(); ++i) {
        file << "camera" << i + 1 << ": "
             << h2eTransformations[i].tx << ", "
             << h2eTransformations[i].ty << ", "
             << h2eTransformations[i].tz << ", "
             << h2eTransformations[i].rx << ", "
             << h2eTransformations[i].ry << ", "
             << h2eTransformations[i].rz << std::endl;
    }

    file << "cam2cam:" << std::endl;
    for (size_t i = 0; i < cam2camTransformations.size(); ++i) {
        for (size_t j = 0; j < cam2camTransformations[i].size(); ++j) {
            file << "cam" << i + 1 << " to cam" << j + 1 << ": "
                 << cam2camTransformations[i][j].tx << ", "
                 << cam2camTransformations[i][j].ty << ", "
                 << cam2camTransformations[i][j].tz << ", "
                 << cam2camTransformations[i][j].rx << ", "
                 << cam2camTransformations[i][j].ry << ", "
                 << cam2camTransformations[i][j].rz << std::endl;
        }
    }

    file.close();
}


int countImagesInFolder(const std::string& path) {
    int count = 0;
    for (const auto& entry : fs::directory_iterator(path)) {
        if (entry.is_regular_file()) {
            auto ext = entry.path().extension().string();
            
            // Convert extension to lowercase to make the comparison case-insensitive
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                count++;
            }
        }
    }
    return count;
}


cv::Mat rpyToRotationMatrix(const cv::Mat& rpy) {
    double roll = rpy.at<double>(0, 0);
    double pitch = rpy.at<double>(1, 0);
    double yaw = rpy.at<double>(2, 0);

    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll));

    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch));

    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1);

    cv::Mat R = Rz * Ry * Rx;
    return R;
}

// Function to convert a rotation matrix to RPY angles
cv::Mat rotationMatrixToRpy(const cv::Mat& R) {
    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +  R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return (cv::Mat_<double>(3, 1) << x, y, z);
}

// Function to compute the RPY angle error
cv::Mat computeRpyAngleError(const cv::Mat& rpy1, const cv::Mat& rpy2) {

    cv::Mat rpy_error = (cv::Mat_<double>(3,1) << rpy1.at<double>(0)-rpy2.at<double>(0), rpy1.at<double>(1)-rpy2.at<double>(1), rpy1.at<double>(2)-rpy2.at<double>(2));

    return rpy_error;
}

double trasError(const cv::Mat& translation_vec1, const cv::Mat &translation_vec2){

    cv::Mat translation_diff;
    cv::absdiff(translation_vec1, translation_vec2, translation_diff);
    double tras_error = cv::norm(translation_diff, cv::NORM_L2);
    return tras_error;
}


void appendTransformationsToFile(const std::string& filename,
                                 const std::string& datasetName,
                                 const int start_index,
                                 const int end_index,
                                 const std::vector<Transformation>& h2e,
                                 const std::vector<std::vector<Transformation>>& cam2cam) {
    // Open the file in append mode
    std::ofstream file(filename, std::ios::app);

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    file << "Dataset: " << datasetName << " (" << end_index-start_index << " images: " << start_index << " - "<< end_index << ")" << std::endl;
    // Append h2e transformations
    int counter_h2e = 1;
    for (const auto& trans : h2e) {
        file << "H2e " << counter_h2e  << ", "
             << trans.tx << ", " << trans.ty << ", " << trans.tz << ", "
             << trans.rx << ", " << trans.ry << ", " << trans.rz << std::endl;
        counter_h2e ++;
    }

    // Append cam2cam transformations
    int counter_c2c_first = 1;

    for (const auto& row : cam2cam) {
        int counter_c2c_second = 1;
        for (const auto& trans : row) {
            file << "C2c " << counter_c2c_first << " to " << counter_c2c_second << ", "
                 << trans.tx << ", " << trans.ty << ", " << trans.tz << ", "
                 << trans.rx << ", " << trans.ry << ", " << trans.rz << std::endl;
            counter_c2c_second++;
        }
        counter_c2c_first++;
    }

    file.close();
}



LinearRegressionResult simpleLinearRegression(const std::vector<double>& x, const std::vector<double>& y) {
    const size_t n = x.size();
    double mean_x = std::accumulate(x.begin(), x.end(), 0.0) / n;
    double mean_y = std::accumulate(y.begin(), y.end(), 0.0) / n;

    double sum_xy = 0.0;
    double sum_xx = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sum_xy += (x[i] - mean_x) * (y[i] - mean_y);
        sum_xx += (x[i] - mean_x) * (x[i] - mean_x);
    }

    LinearRegressionResult result;
    result.slope = sum_xy / sum_xx;
    result.intercept = mean_y - result.slope * mean_x;

    return result;
}


void plotLinearRegression(const std::vector<double>& x, const std::vector<double>& y, const LinearRegressionResult& lrResult) {
    double minX = *std::min_element(x.begin(), x.end());
    double maxX = *std::max_element(x.begin(), x.end());
    double minY = *std::min_element(y.begin(), y.end());
    double maxY = *std::max_element(y.begin(), y.end());

    // Create an image for plotting
    cv::Mat plotImg(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));

    // Normalize the points to fit in the plot
    double scaleX = 500 / (maxX - minX);
    double scaleY = 500 / (maxY - minY);
    double offsetX = 50;
    double offsetY = 550;

    // Draw the original data points
    for (size_t i = 0; i < x.size(); ++i) {
        int px = static_cast<int>(scaleX * (x[i] - minX) + offsetX);
        int py = static_cast<int>(offsetY - scaleY * (y[i] - minY));
        cv::circle(plotImg, cv::Point(px, py), 4, cv::Scalar(255, 0, 0), cv::FILLED);

        // Prepare the text to display
        std::string text = std::to_string(i);

        // Calculate text size to adjust the label position
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

        // Adjust text position to not overlap the point
        cv::Point textOrg(px - textSize.width / 2, py - textSize.height - 10);

        // Put the number near the point
        cv::putText(plotImg, text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    // Draw the linear regression line by calculating start and end points
    cv::Point pt1(offsetX, offsetY - static_cast<int>(scaleY * (lrResult.slope * minX + lrResult.intercept - minY)));
    cv::Point pt2(offsetX + 500, offsetY - static_cast<int>(scaleY * (lrResult.slope * maxX + lrResult.intercept - minY)));
    cv::line(plotImg, pt1, pt2, cv::Scalar(0, 255, 0), 2);

    // Show the plot
    cv::imshow("Linear Regression Plot", plotImg);
    cv::waitKey(0);
}


std::vector<int> removeOutliersFromLine(std::vector<double>& x, std::vector<double>& y, const LinearRegressionResult& lrResult, double threshold) {
    std::vector<int> indicesToKeep;
    std::vector<int> indicesRemoved;

    // Calculate residuals and decide whether to keep each point
    for (size_t i = 0; i < x.size(); ++i) {
        double predictedY = lrResult.slope * x[i] + lrResult.intercept;
        double residual = std::abs(y[i] - predictedY);
        if (residual <= threshold) {
            indicesToKeep.push_back(i);
        } else {
            indicesRemoved.push_back(i); // Save the index of the removed point
        }
    }

    // Create new vectors based on the indices to keep
    std::vector<double> newX, newY;
    for (int idx : indicesToKeep) {
        newX.push_back(x[idx]);
        newY.push_back(y[idx]);
    }

    // Replace old vectors with the new ones
    x.swap(newX);
    y.swap(newY);

    // Return the indices of the removed points
    return indicesRemoved;
}

int countMultiCameraPoses(const std::vector<int>& selected_poses, const std::vector<std::vector<int>>& cross_observation_matrix) {
    int M = cross_observation_matrix.size();
    int N = cross_observation_matrix[0].size();
    int multi_camera_poses = 0;

    for (int i = 0; i < M; ++i) {
        if (selected_poses[i] == 1) {
            int camera_count = 0;
            for (int j = 0; j < N; ++j) {
                if (cross_observation_matrix[i][j] == 1) {
                    camera_count++;
                }
            }
            if (camera_count > 1) {
                multi_camera_poses++;
            }
        }
    }

    return multi_camera_poses;
}

std::vector<int> selectPoses(const std::vector<std::vector<int>>& cross_observation_matrix, int max_images_per_camera) {

    int M = cross_observation_matrix.size();
    int N = cross_observation_matrix[0].size();
    
    // Vector initialization
    std::vector<int> selected_poses(M, 0);
    
    // How many poses are already selected for each camera
    std::vector<int> images_per_camera(N, 0);
    
    // Vector with the indices of the poses
    std::vector<int> pose_indices(M);
    std::iota(pose_indices.begin(), pose_indices.end(), 0);
    
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Shuffle the indices
    std::shuffle(pose_indices.begin(), pose_indices.end(), gen);
    
    for (int pose : pose_indices) {
        bool can_select = true;
        for (int camera = 0; camera < N; ++camera) {
            if (cross_observation_matrix[pose][camera] == 1 && images_per_camera[camera] >= max_images_per_camera) {
                can_select = false;
                break;
            }
        }
        if (can_select) {
            selected_poses[pose] = 1;
            for (int camera = 0; camera < N; ++camera) {
                if (cross_observation_matrix[pose][camera] == 1) {
                    images_per_camera[camera]++;
                }
            }
        }
    }
    
    return selected_poses;
}


// Checks if there is a non-zero element in any of the vectors
bool hasNonZero(const std::vector<std::vector<double>>& vectors, const std::vector<std::vector<int>>& cross_observation_matrix) {
    for (int i = 0; i < vectors.size(); i++) {
        int counter_cam = 0;
        for (int j = 0; j < vectors[i].size(); j++) {
            if (cross_observation_matrix[j][i]) {
                counter_cam++;
            }
        }
        if (counter_cam < std::min(4, static_cast<int>(vectors[i].size()))) {
            return false;
        }
    }

    for (int i = 0; i < vectors.size(); i++){
        for (int j = 0; j < vectors[i].size(); j++) { // Check each element in the current vector
            if (vectors[i][j] != 0.0) {
                return true; // Return true if a non-zero element is found
            }
        }
    }

    return false; // Return false if all elements in all vectors are zero
}


double computeTransError_AXXB(cv::Mat A, cv::Mat B, cv::Mat X){
    cv::Mat chain1 = A*X;
    cv::Mat chain2 = X*B;
    cv::Mat tras1, tras2;
    getTras<double>(chain1, tras1);
    getTras<double>(chain2, tras2);
    double tras_error = trasError(tras1, tras2);
    return tras_error;
}

std::vector<double> computeTransAverageError_AXXB(std::vector<std::vector<cv::Mat>> A, std::vector<std::vector<cv::Mat>> B, std::vector<cv::Mat> X){
    int number_of_cameras = X.size();
    std::vector<double> tras_cam_vec(number_of_cameras);
    double average_error = 0.0;
    for (int i = 0; i < number_of_cameras; i++){
        int counter_cam = 0;
        double tras_cam = 0.0;
        for (int j = 0; j < A[i].size(); j++){
            if (!A[i][j].empty()){
                double tras_error;
                cv::Mat tras1, tras2;
                getTras<double>(A[i][j]*X[i], tras1);
                getTras<double>(X[i]*B[i][j], tras2);
                tras_error = trasError(tras1, tras2);
                tras_cam += tras_error;
                counter_cam++;
            }
        }
        tras_cam = tras_cam/counter_cam;
        tras_cam_vec[i] = tras_cam;
    }

    return tras_cam_vec;
}

void selectRandomSubsetAXXB_RANSAC(std::vector<std::vector<cv::Mat>>& selected_A, std::vector<std::vector<cv::Mat>>& selected_B, std::vector<std::vector<int>>& cross_observation_matrix_RANSAC, const int min_inliers, const int number_of_cameras, std::default_random_engine generator, std::uniform_int_distribution<size_t> distribution, const std::vector<std::vector<cv::Mat>> relative_robot_poses, const std::vector<std::vector<cv::Mat>> relative_cam_poses){
    std::vector<int> non_empty_counter(number_of_cameras, 0);

    bool flag = true;
    while(flag) {
        bool allCamerasReachedTarget = true;
        for (int cam = 0; cam < number_of_cameras; cam++) {
            if (non_empty_counter[cam] < min_inliers) {
                allCamerasReachedTarget = false;
                break;
            }
        }

        if (allCamerasReachedTarget) {
            flag = false;
        }
        int index = distribution(generator);
        for (int cam = 0; cam < number_of_cameras; cam++) {
            if (non_empty_counter[cam] < min_inliers) {
                if (!relative_cam_poses[cam][index].empty()) {
                    selected_A[cam][index] = relative_robot_poses[cam][index];
                    selected_B[cam][index] = relative_cam_poses[cam][index];
                    non_empty_counter[cam]++;
                    cross_observation_matrix_RANSAC[index][cam] = 1;
                    cross_observation_matrix_RANSAC[index+1][cam] = 1;
                }
            }
        }
    }
}



cv::Vec4d rotationMatrixToQuaternion(const cv::Mat &R) {
    double w, x, y, z;
    w = sqrt(std::max(0.0, 1.0 + R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2))) / 2.0;
    x = sqrt(std::max(0.0, 1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2))) / 2.0;
    y = sqrt(std::max(0.0, 1.0 - R.at<double>(0, 0) + R.at<double>(1, 1) - R.at<double>(2, 2))) / 2.0;
    z = sqrt(std::max(0.0, 1.0 - R.at<double>(0, 0) - R.at<double>(1, 1) + R.at<double>(2, 2))) / 2.0;

    x = copysign(x, R.at<double>(2, 1) - R.at<double>(1, 2));
    y = copysign(y, R.at<double>(0, 2) - R.at<double>(2, 0));
    z = copysign(z, R.at<double>(1, 0) - R.at<double>(0, 1));

    return cv::Vec4d(w, x, y, z);
}


void saveTranslationVectors(const std::vector<cv::Mat>& h2e_matrices, const std::vector<cv::Mat>& c2c_matrices, const std::string& filename) {
    // Open the file stream
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    for (const auto& mat : h2e_matrices) {
        if (mat.empty()) {
            // If the matrix is empty, write "0,0,0"
            outFile << "0,0,0,0,0,0" << std::endl;
        } else {
            // Assuming the matrix is 4x4 and contains a translation vector in the last column
            double x = mat.at<double>(0, 3);
            double y = mat.at<double>(1, 3);
            double z = mat.at<double>(2, 3);
            cv::Mat rotation;
            getRoto<double>(mat, rotation);
            cv::Mat euler = rotationMatrixToEulerAngles<double>(rotation);
            // Write the translation vector to the file
            outFile << x << "," << y << "," << z << "," << euler.at<double>(0) << "," << euler.at<double>(1) << "," << euler.at<double>(2) << std::endl;
        }
    }

    for (const auto& mat : c2c_matrices) {
        if (mat.empty()) {
            // If the matrix is empty, write "0,0,0"
            outFile << "0,0,0,0,0,0" << std::endl;
        } else {
            // Assuming the matrix is 4x4 and contains a translation vector in the last column
            double x = mat.at<double>(0, 3);
            double y = mat.at<double>(1, 3);
            double z = mat.at<double>(2, 3);
            cv::Mat rotation;
            getRoto<double>(mat, rotation);
            cv::Mat euler = rotationMatrixToEulerAngles<double>(rotation);
            // Write the translation vector to the file
            outFile << x << "," << y << "," << z << "," << euler.at<double>(0) << "," << euler.at<double>(1) << "," << euler.at<double>(2) << std::endl;
        }
    }

    // Close the file
    outFile.close();
}