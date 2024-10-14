//
// Created by davide on 11/12/23.
//
#include "MobileHandEyeCalibrator.h"

MobileHandEyeCalibrator::MobileHandEyeCalibrator(const int number_of_waypoints, const int sens_quantity, const std::vector<cv::Mat> h2e_initial_guess_vec, 
                                                const std::vector<std::vector<int>> cross_observation_matrix, 
                                                const std::vector<std::vector<double>> cam_z, const std::vector<std::vector<cv::Mat>> relative_robot_poses, 
                                                const std::vector<std::vector<cv::Mat>> relative_cam_poses) {
    number_of_cameras_ = sens_quantity;
    number_of_waypoints_ = number_of_waypoints;
    cross_observation_matrix_ = cross_observation_matrix;
    cam_z_ = cam_z;

    robot_t_rel_.resize(sens_quantity, std::vector<Eigen::Vector3d>(number_of_waypoints-1));
    robot_r_rel_.resize(sens_quantity, std::vector<Eigen::Vector3d>(number_of_waypoints-1));
    pnp_t_rel_.resize(sens_quantity, std::vector<Eigen::Vector3d>(number_of_waypoints-1));
    pnp_r_rel_.resize(sens_quantity, std::vector<Eigen::Vector3d>(number_of_waypoints-1));


    // Array initialization
    h2e_ = new double*[number_of_cameras_];
    for (int i = 0; i < number_of_cameras_; i++) {
        h2e_[i] = new double[6];
    }
    
    // H2e definition
    for (int i = 0; i < number_of_cameras_; i++){
        cv::Mat_<double> r_vec_h2e, t_vec_h2e;
        transfMat2Exp<double>(h2e_initial_guess_vec[i], r_vec_h2e, t_vec_h2e);
        for( int j = 0; j < 3; j++ )
        {
            h2e_[i][j] = r_vec_h2e(j);
            h2e_[i][j+3] = t_vec_h2e(j);
        }
    }
    // Relative poses
    for (int i = 0; i < number_of_cameras_; i++) {
        for (int wp = 0; wp < number_of_waypoints_ - 1; wp++) {
            cv::Mat_<double> rel_r_vec_w2r, rel_t_vec_w2r;

            cv::Mat robot_rel_pose = relative_robot_poses[i][wp];
            if (!robot_rel_pose.empty()) {
                transfMat2Exp<double>(robot_rel_pose, rel_r_vec_w2r, rel_t_vec_w2r);

                for (int j = 0; j < 3; j++) {
                    robot_r_rel_[i][wp](j) = rel_r_vec_w2r(j);
                    robot_t_rel_[i][wp](j) = rel_t_vec_w2r(j);
                }
            }
        }
    }

    for (int i = 0; i < number_of_cameras_; i++){
        for (int wp = 0; wp < number_of_waypoints_-1; wp++){

            cv::Mat_<double> rel_r_vec_c2b, rel_t_vec_c2b;

            cv::Mat cam_rel_pose = relative_cam_poses[i][wp];
            if (!cam_rel_pose.empty()){
                transfMat2Exp<double>(cam_rel_pose, rel_r_vec_c2b, rel_t_vec_c2b);
                for (int j = 0; j < 3; j++) {
                    pnp_r_rel_[i][wp](j) = rel_r_vec_c2b(j);
                    pnp_t_rel_[i][wp](j) = rel_t_vec_c2b(j);
                }
            }

        }
    }
}

void MobileHandEyeCalibrator::mobileCalibration(std::vector<cv::Mat> &optimal_h2e) {
   
   // Ceres problem
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    
    options.linear_solver_type = ceres::DENSE_SCHUR;  //ITERATIVE_SCHUR
    options.max_num_refinement_iterations = 3;
    options.use_mixed_precision_solves = true;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 20;
    options.max_num_iterations = 2000;
    options.gradient_tolerance = 1e-5;  // 1e-5
    options.function_tolerance = std::numeric_limits<double>::epsilon();
    options.parameter_tolerance = 1e-8;  // 1e-8
    bool enable_cauchy_loss = true;
    double humer_a_scale = 1;
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(humer_a_scale);

    ceres::Problem problem_multi_mobile;

    // Prepare the optimization algorithm
    for (int wp = 0; wp < number_of_waypoints_; wp++){

        showProgressBar(wp, number_of_waypoints_);

        // Check how many cameras are detecting
        int detection_amount = accumulate(cross_observation_matrix_[wp].begin(), cross_observation_matrix_[wp].end(), 0);

        // Single camera optimization problem whenever the checkerboard is detected
        if (detection_amount > 0){

            for (int cam = 0; cam < number_of_cameras_; cam++){

                // Check if the current camera detected the checkerboard in the current pose
                if (cross_observation_matrix_[wp][cam]) {

                    if (wp > 0) {
                        if (cross_observation_matrix_[wp - 1][cam]) {
                            if (cam_z_[cam][wp] == 0) {
                        
                                ceres::CostFunction *cost_function_ax_xb_wo_z_rel = Classic_ax_xb_wo_z_rel::Create(pnp_r_rel_[cam][wp - 1], pnp_t_rel_[cam][wp - 1], robot_r_rel_[cam][wp - 1], robot_t_rel_[cam][wp - 1]);
                                problem_multi_mobile.AddResidualBlock(cost_function_ax_xb_wo_z_rel, loss_function, h2e_[cam]);
                            } else {

                                ceres::CostFunction *cost_function_ax_xb_rel = Classic_ax_xb_rel::Create(pnp_r_rel_[cam][wp - 1], pnp_t_rel_[cam][wp - 1], robot_r_rel_[cam][wp - 1], robot_t_rel_[cam][wp - 1], cam_z_[cam][wp]);
                                problem_multi_mobile.AddResidualBlock(cost_function_ax_xb_rel, loss_function, h2e_[cam]);
                            }
                        }
                    }
                }
            }
        }
    }

    ceres::Solver::Summary summary_cam2cam;
    Solve(options, &problem_multi_mobile, &summary_cam2cam);

    std::cout << summary_cam2cam.FullReport() << "\n";

    for (int cam = 0; cam < number_of_cameras_; cam++){
        cv::Mat optim_h2e_rvec = (cv::Mat_<double>(3,1) << h2e_[cam][0], h2e_[cam][1], h2e_[cam][2]);
        cv::Mat optim_h2e_tvec = (cv::Mat_<double>(3,1) << h2e_[cam][3], h2e_[cam][4], h2e_[cam][5]);
        cv::Mat optim_h2e_mat;

        exp2TransfMat<double>(optim_h2e_rvec, optim_h2e_tvec, optim_h2e_mat);
        optimal_h2e[cam] = optim_h2e_mat;
    }
}



void MobileHandEyeCalibrator::mobileJointCalibration(std::vector<cv::Mat> &optimal_h2e) {
    
    // Ceres problem
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    options.linear_solver_type = ceres::DENSE_SCHUR;  //ITERATIVE_SCHUR
    options.max_num_refinement_iterations = 3;
    options.use_mixed_precision_solves = true;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 20;
    options.max_num_iterations = 2000;
    options.gradient_tolerance = 1e-5;  // 1e-5
    options.function_tolerance = std::numeric_limits<double>::epsilon();
    options.parameter_tolerance = 1e-8;  // 1e-8
    bool enable_cauchy_loss = true;
    double humer_a_scale = 1;
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(humer_a_scale);

    ceres::Problem problem_multi_mobile;

    // Prepare the optimization algorithm
    for (int wp = 0; wp < number_of_waypoints_; wp++){

        showProgressBar(wp, number_of_waypoints_);

        // Check how many cameras are detecting
        int detection_amount = accumulate(cross_observation_matrix_[wp].begin(), cross_observation_matrix_[wp].end(), 0);

        // Single camera optimization problem whenever the checkerboard is detected
        if (detection_amount > 0){

            for (int cam = 0; cam < number_of_cameras_; cam++){
                // Check if the current camera detected the checkerboard in the current pose
                if (cross_observation_matrix_[wp][cam]) {

                    if (wp > 0) {
                        if (cross_observation_matrix_[wp - 1][cam]) {
                            if (cam_z_[cam][wp] == 0) {

                                ceres::CostFunction *cost_function_ax_xb_wo_z_rel = Classic_ax_xb_wo_z_rel::Create(pnp_r_rel_[cam][wp - 1], pnp_t_rel_[cam][wp - 1], robot_r_rel_[cam][wp - 1], robot_t_rel_[cam][wp - 1]);
                                problem_multi_mobile.AddResidualBlock(cost_function_ax_xb_wo_z_rel, loss_function, h2e_[cam]);
                            } else {

                                ceres::CostFunction *cost_function_ax_xb_rel = Classic_ax_xb_rel::Create(pnp_r_rel_[cam][wp - 1], pnp_t_rel_[cam][wp - 1], robot_r_rel_[cam][wp - 1], robot_t_rel_[cam][wp - 1], cam_z_[cam][wp]);
                                problem_multi_mobile.AddResidualBlock(cost_function_ax_xb_rel, loss_function, h2e_[cam]);
                            }
                        }
                    }
                }
            }

            // Multi camera optimization problem whenever the checkerboard is detected simultaneously by more cameras
            if (detection_amount>1){
                for (int cam_1 = 0; cam_1 < number_of_cameras_; cam_1++){
                    for (int cam_2 = 0; cam_2 < number_of_cameras_; cam_2++){

                        // Check that the cameras which are detecting are different
                        if (cam_1 != cam_2 && cross_observation_matrix_[wp][cam_1] && cross_observation_matrix_[wp][cam_2]){

                            if (wp > 0) {
                                if (cross_observation_matrix_[wp - 1][cam_1] && cross_observation_matrix_[wp - 1][cam_1] && cross_observation_matrix_[wp - 1][cam_2] && cross_observation_matrix_[wp - 1][cam_2]) {
                                    ceres::CostFunction *cost_function_ax_xb_rel = Classic_ax_xb_rel_multi::Create(
                                            pnp_r_rel_[cam_1][wp - 1], pnp_t_rel_[cam_1][wp - 1], pnp_r_rel_[cam_2][wp - 1], pnp_t_rel_[cam_2][wp - 1]);
                                    problem_multi_mobile.AddResidualBlock(cost_function_ax_xb_rel, loss_function, h2e_[cam_1], h2e_[cam_2]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    ceres::Solver::Summary summary_cam2cam;
    Solve(options, &problem_multi_mobile, &summary_cam2cam);

    std::cout << summary_cam2cam.FullReport() << "\n";

    for (int cam = 0; cam < number_of_cameras_; cam++){
        cv::Mat optim_h2e_rvec = (cv::Mat_<double>(3,1) << h2e_[cam][0], h2e_[cam][1], h2e_[cam][2]);
        cv::Mat optim_h2e_tvec = (cv::Mat_<double>(3,1) << h2e_[cam][3], h2e_[cam][4], h2e_[cam][5]);
        cv::Mat optim_h2e_mat;

        exp2TransfMat<double>(optim_h2e_rvec, optim_h2e_tvec, optim_h2e_mat);
        optimal_h2e[cam] = optim_h2e_mat;
    }
}



