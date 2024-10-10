//
// Created by davide on 06/10/23.
//

#include "SingleHandEyeCalibrator.h"

SingleHandEyeCalibrator::SingleHandEyeCalibrator(const std::vector<cv::Mat> correct_poses, const std::vector<cv::Point3f> object_points, const cv::Mat h2e_initial_guess_vec, const cv::Mat b2ee_initial_guess){
    std::cout << "Start single camera hand-eye calibration.." << std::endl;

    pattern_pts_.reserve(object_points.size());
    for( auto &cp : object_points )
        pattern_pts_.emplace_back( cp.x, cp.y, cp.z );

    cv::Mat_<float> r_vec, t_vec;
    robot_r_vecs_.resize(correct_poses.size());
    robot_t_vecs_.resize(correct_poses.size());

    // Save tvec and rvec for each robot path_pose
    for (int i = 0; i < correct_poses.size(); i++){
        cv::Mat robot_pose_temp = correct_poses[i];
        transfMat2Exp<float>(robot_pose_temp, r_vec, t_vec);
        for( int j = 0; j < 3; j++ ){
            robot_r_vecs_[i](j) = r_vec(j);
            robot_t_vecs_[i](j) = t_vec(j);
        }
    }

    cv::Mat_<float> r_vec_b2ee, t_vec_b2ee;
    transfMat2Exp<float>(b2ee_initial_guess, r_vec_b2ee, t_vec_b2ee);
    for( int j = 0; j < 3; j++ ){

        board2ee_[j] = r_vec_b2ee(j);
        board2ee_[j+3] = t_vec_b2ee(j);
    }

    // Save tvec and rvec for the camera2robot initial guess
    transfMat2Exp<float>(h2e_initial_guess_vec, r_vec, t_vec);
    for( int j = 0; j < 3; j++ ){

        h2e_[j] = r_vec(j);
        h2e_[j+3] = t_vec(j);
    }
}

void SingleHandEyeCalibrator::Calibration(CameraInfo camera_info, std::vector<std::vector<cv::Point2f>> corners, cv::Mat &optimal_h2e, cv::Mat &optimal_b2ee){
    PinholeCameraModel camera_model(camera_info);
    double observed_pt_data[2];
    Eigen::Map<Eigen::Vector2d> observed_pt(observed_pt_data);
    // Ceres problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 4;
    options.max_num_iterations = 100;
    options.gradient_tolerance = std::numeric_limits<double>::epsilon();
    options.function_tolerance = std::numeric_limits<double>::epsilon();
    bool enable_huber_loss = true;
    double humer_a_scale = 5.0;
    ceres::LossFunction* loss_function = enable_huber_loss ? new ceres::HuberLoss(humer_a_scale) : NULL;
    ceres::Problem problem;

    //######### EYE-ON-BASE ##########
    for (int i = 0; i < robot_r_vecs_.size(); i++){

        auto images_points = corners[i];
        for( int k = 0; k < pattern_pts_.size(); k++ ) {
            observed_pt_data[0] = images_points[k].x;
            observed_pt_data[1] = images_points[k].y;

            ceres::CostFunction *cost_function = CalibReprojectionError::Create(camera_model, robot_r_vecs_[i], robot_t_vecs_[i],
                                                                      pattern_pts_[k], observed_pt);

            problem.AddResidualBlock(cost_function, loss_function, board2ee_, h2e_);
        }
    }

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // Put result into matrices
    cv::Mat optim_h2e_rvec = (cv::Mat_<float>(3,1) << h2e_[0], h2e_[1], h2e_[2]);
    cv::Mat optim_h2e_tvec = (cv::Mat_<float>(3,1) << h2e_[3], h2e_[4], h2e_[5]);

    exp2TransfMat<float>(optim_h2e_rvec, optim_h2e_tvec, optimal_h2e);

    cv::Mat optim_board2tcp_rvec = (cv::Mat_<float>(3,1) << board2ee_[0], board2ee_[1], board2ee_[2]);
    cv::Mat optim_board2tcp_tvec = (cv::Mat_<float>(3,1) << board2ee_[3], board2ee_[4], board2ee_[5]);

    exp2TransfMat<float>(optim_board2tcp_rvec, optim_board2tcp_tvec, optimal_b2ee);
}