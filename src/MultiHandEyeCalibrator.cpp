//
// Created by davide on 31/10/23.
//

#include "MultiHandEyeCalibrator.h"

MultiHandEyeCalibrator::MultiHandEyeCalibrator(const int number_of_waypoints, const int sens_quantity, const std::vector<cv::Point3f> object_points, std::vector<cv::Mat> poses, const std::vector<cv::Mat> h2e_initial_guess_vec, const cv::Mat b2ee_initial_guess, const std::vector<std::vector<int>> cross_observation_matrix, std::vector<std::vector<cv::Mat>> rvec_all, std::vector<std::vector<cv::Mat>> tvec_all, std::vector<std::vector<cv::Mat>> pnp) {
    number_of_cameras_ = sens_quantity;
    number_of_waypoints_ = number_of_waypoints;
    cross_observation_matrix_ = cross_observation_matrix;
    robot_r_vecs_.resize(number_of_waypoints);
    robot_t_vecs_.resize(number_of_waypoints);
    robot_r_vecs_inv_.resize(number_of_waypoints);
    robot_t_vecs_inv_.resize(number_of_waypoints);
    pnp_t_vecs_.resize(sens_quantity, std::vector<Eigen::Vector3d>(number_of_waypoints));
    pnp_r_vecs_.resize(sens_quantity, std::vector<Eigen::Vector3d>(number_of_waypoints));

    // Array initialization
    h2e_ = new double*[number_of_cameras_];
    cam2cam_ = new double**[number_of_cameras_];
    for (int i = 0; i < number_of_cameras_; i++) {
        h2e_[i] = new double[6];
        cam2cam_[i] = new double*[number_of_cameras_];
        for (int j = 0; j < number_of_cameras_; j++) {
            cam2cam_[i][j] = new double[6];
        }
    }

    // Initialization of robot poses to be optimized
    robot_opt_ = new double*[number_of_waypoints_];
    for (int j = 0; j < number_of_waypoints_; j++) {
        robot_opt_[j] = new double[6];
    }


    for (int j = 0; j < number_of_waypoints; j++){
        cv::Mat_<double> r_vec_pose, t_vec_pose;
        transfMat2Exp<double>(poses[j].inv(), r_vec_pose, t_vec_pose);
        for (int k = 0; k <3; k++) {
            robot_opt_[j][k] = r_vec_pose(k);
            robot_opt_[j][k + 3] = t_vec_pose(k);
        }
    }


    // H2e and Cam2cam definition
    for (int i = 0; i < number_of_cameras_; i++){
        cv::Mat_<double> r_vec_h2e, t_vec_h2e;
        transfMat2Exp<double>(h2e_initial_guess_vec[i], r_vec_h2e, t_vec_h2e);
        for( int j = 0; j < 3; j++ )
        {
            h2e_[i][j] = r_vec_h2e(j);
            h2e_[i][j+3] = t_vec_h2e(j);
        }


        for (int j = 0; j < number_of_cameras_; j++){
            if (i!=j){
                cv::Mat_<double> r_vec_c2c, t_vec_c2c;
                transfMat2Exp<double>(h2e_initial_guess_vec[i]*h2e_initial_guess_vec[j].inv(), r_vec_c2c, t_vec_c2c); // j-th camera frame in i-th camera frame
                for (int k = 0; k < 3; k++){

                    cam2cam_[i][j][k] = r_vec_c2c(k);
                    cam2cam_[i][j][k+3] = t_vec_c2c(k);
                }
            }
        }
    }

    // Save tvec and rvec for each robot path_pose
    for (int wp = 0; wp < number_of_waypoints_; wp++) {
        cv::Mat_<double> r_vec_w2e, t_vec_w2e;
        cv::Mat robot_pose_temp = poses[wp];
        transfMat2Exp<double>(robot_pose_temp, r_vec_w2e, t_vec_w2e);
        for (int j = 0; j < 3; j++) {
            robot_r_vecs_[wp](j) = r_vec_w2e(j);
            robot_t_vecs_[wp](j) = t_vec_w2e(j);
        }
    }

    for (int i = 0; i < number_of_cameras_; i++){
        for (int wp = 0; wp < number_of_waypoints_; wp++){
            if (cross_observation_matrix_[wp][i]) {
                for (int j = 0; j < 3; j++) {
                    pnp_r_vecs_[i][wp](j) = rvec_all[i][wp].at<double>(j);
                    pnp_t_vecs_[i][wp](j) = tvec_all[i][wp].at<double>(j);
                }
            }
        }
    }

    pnp_ = new double**[number_of_cameras_];
    for (int i = 0; i < number_of_cameras_; i++) {
        pnp_[i] = new double*[number_of_waypoints];
        for (int j = 0; j < number_of_waypoints; j++) {
            pnp_[i][j] = new double[6];
        }
    }

    for (int i = 0; i < number_of_cameras_; i++){
        for (int j = 0; j < number_of_waypoints; j++){
            if (cross_observation_matrix_[j][i]){
                cv::Mat_<double> r_vec_pnp, t_vec_pnp;
                transfMat2Exp<double>(pnp[i][j], r_vec_pnp, t_vec_pnp);
                for (int k = 0; k <3; k++) {
                    pnp_[i][j][k] = r_vec_pnp(k);
                    pnp_[i][j][k + 3] = t_vec_pnp(k);
                }
            }
        }
    }

    // Save inverse tvec and inverse rvec for each robot path_pose
    for (int wp = 0; wp < number_of_waypoints_; wp++) {
        cv::Mat_<double> r_vec_w2e, t_vec_w2e;
        cv::Mat robot_pose_temp = poses[wp];
        transfMat2Exp<double>(robot_pose_temp.inv(), r_vec_w2e, t_vec_w2e);
        for (int j = 0; j < 3; j++) {
            robot_r_vecs_inv_[wp](j) = r_vec_w2e(j);
            robot_t_vecs_inv_[wp](j) = t_vec_w2e(j);
        }
    }

    // Object point initialization
    cv::Mat_<double> r_vec, t_vec;
    pattern_pts_.reserve(object_points.size());

    for( auto &cp : object_points )
        pattern_pts_.emplace_back( cp.x, cp.y, cp.z );

    transfMat2Exp<double>(b2ee_initial_guess, r_vec, t_vec);
    for( int j = 0; j < 3; j++ ){

        board2ee_[j] = r_vec(j);
        board2ee_[j+3] = t_vec(j);
    }
}

void MultiHandEyeCalibrator::eyeOnBaseCalibration(const std::vector<CameraInfo> camera_info, std::vector<std::vector<std::vector<cv::Point2f>>> corners, std::vector<cv::Mat> &optimal_h2e, cv::Mat &optimal_b2ee, std::vector<std::vector<cv::Mat>> &optimal_cam2cam, std::vector<int> selected_poses) {
    double observed_pt_data[2];
    double observed_pt_data_dir[2];
    Eigen::Map<Eigen::Vector2d> observed_pt(observed_pt_data);
    Eigen::Map<Eigen::Vector2d> observed_pt_dir(observed_pt_data_dir);

    // Ceres problem
    ceres::Solver::Options options;

    // Minimizer type (TRUST_REGION or LINE_SEARCH)
    //options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.preconditioner_type = ceres::JACOBI;

    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 18;
    options.max_num_iterations = 3000;
    options.gradient_tolerance = 1e-5;
    options.function_tolerance = std::numeric_limits<double>::epsilon();
    options.parameter_tolerance = 1e-8;
    double humer_a_scale = 1.0;
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(humer_a_scale);

    ceres::Problem problem_multi_cam2cam;

    std::vector<PinholeCameraModel> cam_model_vec;
    for (int i = 0; i < number_of_cameras_; i++){
        PinholeCameraModel camera_model(camera_info[i]);
        cam_model_vec.push_back(camera_model);
    }

    // Prepare the optimization algorithm
    for (int wp = 0; wp < number_of_waypoints_; wp++){
        showProgressBar(wp, number_of_waypoints_);

        // Check how many cameras are detecting
        int detection_amount = accumulate(cross_observation_matrix_[wp].begin(), cross_observation_matrix_[wp].end(), 0);

        // Single camera optimization problem whenever the checkerboard is detected
        if (detection_amount > 0){
            for (int cam = 0; cam < number_of_cameras_; cam++){
                // Check if the current camera detected the checkerboard in the current pose
                if (cross_observation_matrix_[wp][cam] && selected_poses[wp] == 1){
                    auto images_points = corners[wp][cam];
                    for (int k = 0; k < pattern_pts_.size(); k++){
                        observed_pt_data[0] = images_points[k].x;
                        observed_pt_data[1] = images_points[k].y;
                        ceres::CostFunction *cost_function = CalibReprojectionError::Create(cam_model_vec[cam], robot_r_vecs_[wp], robot_t_vecs_[wp], pattern_pts_[k], observed_pt);
                        problem_multi_cam2cam.AddResidualBlock(cost_function, loss_function, board2ee_, h2e_[cam]);
                    }
                }
            }
            // Multi camera optimization problem whenever the checkerboard is detected simultaneously by more cameras
            if (detection_amount>1){
                for (int cam_1 = 0; cam_1 < number_of_cameras_; cam_1++){
                    for (int cam_2 = 0; cam_2 < number_of_cameras_; cam_2++){

                        // Check that the cameras which are detecting are different
                        if (cam_1 != cam_2 && cross_observation_matrix_[wp][cam_1] && cross_observation_matrix_[wp][cam_2] && selected_poses[wp] == 1){

                            auto images_points1 = corners[wp][cam_1];
                            auto images_points2 = corners[wp][cam_2];
                            for (int k = 0; k < pattern_pts_.size(); k++){

                                observed_pt_data[0] = images_points1[k].x;
                                observed_pt_data[1] = images_points1[k].y;

                                observed_pt_data_dir[0] = images_points2[k].x;
                                observed_pt_data_dir[1] = images_points2[k].y;

                                //MULTI
                                ceres::CostFunction *cost_function_multi =
                                        MultiCalibReprojectionError::Create(cam_model_vec[cam_1], robot_r_vecs_[wp], robot_t_vecs_[wp], pattern_pts_[k], observed_pt);
                                problem_multi_cam2cam.AddResidualBlock(cost_function_multi, loss_function, board2ee_, h2e_[cam_2], cam2cam_[cam_1][cam_2]);

                                ceres::CostFunction *cost_function_multi_inv =
                                        MultiCalibReprojectionErrorInverse::Create(cam_model_vec[cam_2], robot_r_vecs_[wp], robot_t_vecs_[wp], pattern_pts_[k], observed_pt_dir);
                                problem_multi_cam2cam.AddResidualBlock(cost_function_multi_inv, loss_function, board2ee_, h2e_[cam_1], cam2cam_[cam_1][cam_2]);
                            }
                        }
                    }
                }
            }
        }
    }

    ceres::Solver::Summary summary_cam2cam;
    Solve(options, &problem_multi_cam2cam, &summary_cam2cam);

    std::cout << summary_cam2cam.FullReport() << "\n";

    for (int cam = 0; cam < number_of_cameras_; cam++){
        cv::Mat optim_h2e_rvec = (cv::Mat_<double>(3,1) << h2e_[cam][0], h2e_[cam][1], h2e_[cam][2]);
        cv::Mat optim_h2e_tvec = (cv::Mat_<double>(3,1) << h2e_[cam][3], h2e_[cam][4], h2e_[cam][5]);
        cv::Mat optim_h2e_mat;
        exp2TransfMat<double>(optim_h2e_rvec, optim_h2e_tvec, optim_h2e_mat);
        optimal_h2e[cam] = optim_h2e_mat;
    }

    for (int i = 0; i < number_of_cameras_; i++){
        for (int j = 0; j < number_of_cameras_; j++){
            if (i!=j){
                optimal_cam2cam[i][j] = optimal_h2e[i]*optimal_h2e[j].inv();
            }
        }
    }

    cv::Mat optim_board2tcp_rvec = (cv::Mat_<double>(3,1) << board2ee_[0], board2ee_[1], board2ee_[2]);
    cv::Mat optim_board2tcp_tvec = (cv::Mat_<double>(3,1) << board2ee_[3], board2ee_[4], board2ee_[5]);
    exp2TransfMat<double>(optim_board2tcp_rvec, optim_board2tcp_tvec, optimal_b2ee);
}


void MultiHandEyeCalibrator::eyeInHandCalibration(const std::vector<CameraInfo> camera_info, std::vector<std::vector<std::vector<cv::Point2f>>> corners, std::vector<cv::Mat> &optimal_h2e, cv::Mat &optimal_b2ee, std::vector<std::vector<cv::Mat>> &optimal_cam2cam, std::vector<std::vector<cv::Mat>> images_collected) {
    double observed_pt_data[2];
    double observed_pt_data_dir[2];
    Eigen::Map<Eigen::Vector2d> observed_pt(observed_pt_data);
    Eigen::Map<Eigen::Vector2d> observed_pt_dir(observed_pt_data_dir);

    // Ceres problem
    ceres::Solver::Options options;

    // Minimizer type (TRUST_REGION or LINE_SEARCH)
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.preconditioner_type = ceres::JACOBI;

    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 18;
    options.max_num_iterations = 5000;
    options.gradient_tolerance = 1e-5;
    options.function_tolerance = std::numeric_limits<double>::epsilon();
    options.parameter_tolerance = 1e-8;
    double humer_a_scale = 1.0;
    ceres::LossFunction* loss_function =  new ceres::CauchyLoss(humer_a_scale);

    ceres::Problem problem_multi_eye_in_hand;

    std::vector<PinholeCameraModel> cam_model_vec;
    for (int i = 0; i < number_of_cameras_; i++){
        PinholeCameraModel camera_model(camera_info[i]);
        cam_model_vec.push_back(camera_model);
    }

    // Prepare the optimization algorithm
    for (int wp = 0; wp < number_of_waypoints_; wp++){
        showProgressBar(wp, number_of_waypoints_);

        // Check how many cameras are detecting
        int detection_amount = accumulate(cross_observation_matrix_[wp].begin(), cross_observation_matrix_[wp].end(), 0);

        // Single camera optimization problem whenever the checkerboard is detected
        if (detection_amount > 0){
            for (int cam = 0; cam < number_of_cameras_; cam++){
                // Check if the current camera detected the checkerboard in the current pose
                if (cross_observation_matrix_[wp][cam]){

                    auto images_points = corners[wp][cam];

                    for (int k = 0; k < pattern_pts_.size(); k++){
                        observed_pt_data[0] = images_points[k].x;
                        observed_pt_data[1] = images_points[k].y;
                        ceres::CostFunction *cost_function = EyeInHandCalibReprojectionError::Create(cam_model_vec[cam], robot_r_vecs_inv_[wp], robot_t_vecs_inv_[wp], pattern_pts_[k], observed_pt);
                        problem_multi_eye_in_hand.AddResidualBlock(cost_function, loss_function, board2ee_, h2e_[cam]);
                    }
                }
            }
            // Multi camera optimization problem whenever the checkerboard is detected simultaneously by more cameras
            if (detection_amount>1){
                for (int cam_1 = 0; cam_1 < number_of_cameras_; cam_1++){
                    for (int cam_2 = 0; cam_2 < number_of_cameras_; cam_2++){

                        // Check that the cameras which are detecting are different
                        if (cam_1 != cam_2 && cross_observation_matrix_[wp][cam_1] && cross_observation_matrix_[wp][cam_2]){

                            auto images_points1 = corners[wp][cam_1];
                            auto images_points2 = corners[wp][cam_2];
                            for (int k = 0; k < pattern_pts_.size(); k++){

                                observed_pt_data[0] = images_points1[k].x;
                                observed_pt_data[1] = images_points1[k].y;

                                observed_pt_data_dir[0] = images_points2[k].x;
                                observed_pt_data_dir[1] = images_points2[k].y;

                                //MULTI
                                ceres::CostFunction *cost_function_multi =
                                        MultiEyeInHandCalibReprojectionError::Create(cam_model_vec[cam_1], robot_r_vecs_inv_[wp], robot_t_vecs_inv_[wp], pattern_pts_[k], observed_pt);
                                problem_multi_eye_in_hand.AddResidualBlock(cost_function_multi, loss_function, board2ee_, h2e_[cam_2], cam2cam_[cam_1][cam_2]);

                                ceres::CostFunction *cost_function_multi_inv =
                                        MultiEyeInHandCalibReprojectionErrorInverse::Create(cam_model_vec[cam_2], robot_r_vecs_inv_[wp], robot_t_vecs_inv_[wp], pattern_pts_[k], observed_pt_dir);
                                problem_multi_eye_in_hand.AddResidualBlock(cost_function_multi_inv, loss_function, board2ee_, h2e_[cam_1], cam2cam_[cam_1][cam_2]);
                            }
                        }
                    }
                }
            }
        }
    }

    ceres::Solver::Summary summary_cam2cam;
    Solve(options, &problem_multi_eye_in_hand, &summary_cam2cam);

    std::cout << summary_cam2cam.FullReport() << "\n";

    for (int cam = 0; cam < number_of_cameras_; cam++){
        cv::Mat optim_h2e_rvec = (cv::Mat_<double>(3,1) << h2e_[cam][0], h2e_[cam][1], h2e_[cam][2]);
        cv::Mat optim_h2e_tvec = (cv::Mat_<double>(3,1) << h2e_[cam][3], h2e_[cam][4], h2e_[cam][5]);
        cv::Mat optim_h2e_mat;

        exp2TransfMat<double>(optim_h2e_rvec, optim_h2e_tvec, optim_h2e_mat);
        optimal_h2e[cam] = optim_h2e_mat;
    }

    cv::Mat optim_board2tcp_rvec = (cv::Mat_<double>(3,1) << board2ee_[0], board2ee_[1], board2ee_[2]);
    cv::Mat optim_board2tcp_tvec = (cv::Mat_<double>(3,1) << board2ee_[3], board2ee_[4], board2ee_[5]);
    exp2TransfMat<double>(optim_board2tcp_rvec, optim_board2tcp_tvec, optimal_b2ee);
}