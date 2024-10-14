//
// Created by Davide on 27/09/24.
//

#ifndef MEMROC_HANDEYE_CALIBRATOR_H
#define MEMROC_HANDEYE_CALIBRATOR_H

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <ceres/rotation.h>


struct Classic_ax_xb_rel
{
    Classic_ax_xb_rel(const Eigen::Vector3d &pnp_r_rel,
                  const Eigen::Vector3d &pnp_t_rel,
                  const Eigen::Vector3d &robot_r_rel,
                  const Eigen::Vector3d &robot_t_rel,
                  const double camz_i) :
            pnp_r_rel(pnp_r_rel),
            pnp_t_rel(pnp_t_rel),
            robot_r_rel(robot_r_rel),
            robot_t_rel(robot_t_rel),
            camz_i(camz_i){}

    bool operator()(const double* const h2e,
                    double* residuals) const
    {
        double rot_mat_inv_1[9], rot_vec_1[3], tras_vec_1[3], rot_mat2[9], rot_mat_inv2[9], rot_vec_2[3], tras_vec_2[3],
                cal_rot_mat[9], h2e_rot_mat[9], rot_mat_inv_2[9], rot_mat_rob[9], tras_vec_3[3];

        ceres::AngleAxisToRotationMatrix(h2e, h2e_rot_mat);
        cv::Mat X = (cv::Mat_<double>(4,4) <<
                h2e_rot_mat[0], h2e_rot_mat[3], h2e_rot_mat[6], h2e[3],
                h2e_rot_mat[1], h2e_rot_mat[4], h2e_rot_mat[7], h2e[4],
                h2e_rot_mat[2], h2e_rot_mat[5], h2e_rot_mat[8], h2e[5],
                0, 0, 0, 1);


        double robot_r[3] = {double(robot_r_rel(0)), double(robot_r_rel(1)), double(robot_r_rel(2))},
                robot_t[3] = {double(robot_t_rel(0)), double(robot_t_rel(1)), double(robot_t_rel(2))};

        //###########################################################################################
        ceres::AngleAxisToRotationMatrix(robot_r, rot_mat_rob);

        cv::Mat A = (cv::Mat_<double>(4,4) <<
                rot_mat_rob[0], rot_mat_rob[3], rot_mat_rob[6], robot_t[0],
                rot_mat_rob[1], rot_mat_rob[4], rot_mat_rob[7], robot_t[1],
                rot_mat_rob[2], rot_mat_rob[5], rot_mat_rob[8], robot_t[2],
                0, 0, 0, 1);

        cv::Mat chain1 = A* X;
        cv::Mat chain3 = X;


        rot_mat_inv_1[0] = chain1.at<double>(0,0);
        rot_mat_inv_1[1] = chain1.at<double>(1,0);
        rot_mat_inv_1[2] = chain1.at<double>(2,0);
        rot_mat_inv_1[3] = chain1.at<double>(0,1);
        rot_mat_inv_1[4] = chain1.at<double>(1,1);
        rot_mat_inv_1[5] = chain1.at<double>(2,1);
        rot_mat_inv_1[6] = chain1.at<double>(0,2);
        rot_mat_inv_1[7] = chain1.at<double>(1,2);
        rot_mat_inv_1[8] = chain1.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv_1, rot_vec_1);

        tras_vec_1[0] = chain1.at<double>(0,3);
        tras_vec_1[1] = chain1.at<double>(1,3);
        tras_vec_1[2] = chain1.at<double>(2,3);

        tras_vec_3[0] = chain3.at<double>(0,3);
        tras_vec_3[1] = chain3.at<double>(1,3);
        tras_vec_3[2] = chain3.at<double>(2,3);

        //###########################################################################################

        // Chain 2
        double pnp_r[3] = {double(pnp_r_rel(0)), double(pnp_r_rel(1)), double(pnp_r_rel(2))};
        double pnp_t[3] = {double(pnp_t_rel(0)), double(pnp_t_rel(1)), double(pnp_t_rel(2))};

        ceres::AngleAxisToRotationMatrix(pnp_r, cal_rot_mat);
        cv::Mat B = (cv::Mat_<double>(4,4) <<
                cal_rot_mat[0], cal_rot_mat[3], cal_rot_mat[6], pnp_t[0],
                cal_rot_mat[1], cal_rot_mat[4], cal_rot_mat[7], pnp_t[1],
                cal_rot_mat[2], cal_rot_mat[5], cal_rot_mat[8], pnp_t[2],
                0, 0, 0, 1);

        cv::Mat chain2 = X * B;

        rot_mat_inv_2[0] = chain2.at<double>(0,0);
        rot_mat_inv_2[1] = chain2.at<double>(1,0);
        rot_mat_inv_2[2] = chain2.at<double>(2,0);
        rot_mat_inv_2[3] = chain2.at<double>(0,1);
        rot_mat_inv_2[4] = chain2.at<double>(1,1);
        rot_mat_inv_2[5] = chain2.at<double>(2,1);
        rot_mat_inv_2[6] = chain2.at<double>(0,2);
        rot_mat_inv_2[7] = chain2.at<double>(1,2);
        rot_mat_inv_2[8] = chain2.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv_2, rot_vec_2);

        tras_vec_2[0] = chain2.at<double>(0,3);
        tras_vec_2[1] = chain2.at<double>(1,3);
        tras_vec_2[2] = chain2.at<double>(2,3);

        //###########################################################################################

        residuals[0] = (tras_vec_1[0] - tras_vec_2[0]);
        residuals[1] = (tras_vec_1[1] - tras_vec_2[1]);
        residuals[2] = (tras_vec_1[2] - tras_vec_2[2]);
        residuals[3] = (rot_vec_1[0] - rot_vec_2[0]);
        residuals[4] = (rot_vec_1[1] - rot_vec_2[1]);
        residuals[5] = (rot_vec_1[2] - rot_vec_2[2]);
        residuals[6] = tras_vec_3[2] - abs(camz_i);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const Eigen::Vector3d &pnp_r_rel,
                                        const Eigen::Vector3d &pnp_t_rel,
                                        const Eigen::Vector3d &robot_r_rel,
                                        const Eigen::Vector3d &robot_t_rel,
                                        const double &camz_i)
    {
        return new ceres::NumericDiffCostFunction<Classic_ax_xb_rel, ceres::CENTRAL, 7, 6>(
                new Classic_ax_xb_rel(pnp_r_rel, pnp_t_rel, robot_r_rel, robot_t_rel, camz_i));
    }

    Eigen::Vector3d pnp_r_rel;
    Eigen::Vector3d pnp_t_rel;
    Eigen::Vector3d robot_r_rel;
    Eigen::Vector3d robot_t_rel;
    double camz_i;
};




struct Classic_ax_xb_rel_multi
{
    Classic_ax_xb_rel_multi(const Eigen::Vector3d &pnp_r_rel1,
                      const Eigen::Vector3d &pnp_t_rel1,
                      const Eigen::Vector3d &pnp_r_rel2,
                      const Eigen::Vector3d &pnp_t_rel2) :
            pnp_r_rel1(pnp_r_rel1),
            pnp_t_rel1(pnp_t_rel1),
            pnp_r_rel2(pnp_r_rel2),
            pnp_t_rel2(pnp_t_rel2){}

    bool operator()(const double* const h2e1,
                    const double* const h2e2,
                    double* residuals) const
    {
        double rot_mat_inv_1[9], rot_vec_1[3], tras_vec_1[3], rot_mat_inv2[9], rot_vec_2[3], tras_vec_2[3], rot_mat_cam1[9], rot_mat_cam2[9],
                h2e_rot_mat1[9], h2e_rot_mat2[9], rot_mat_inv_2[9], rot_mat_rob[9];

        ceres::AngleAxisToRotationMatrix(h2e1, h2e_rot_mat1);
        cv::Mat X1 = (cv::Mat_<double>(4,4) <<
                h2e_rot_mat1[0], h2e_rot_mat1[3], h2e_rot_mat1[6], h2e1[3],
                h2e_rot_mat1[1], h2e_rot_mat1[4], h2e_rot_mat1[7], h2e1[4],
                h2e_rot_mat1[2], h2e_rot_mat1[5], h2e_rot_mat1[8], h2e1[5],
                0, 0, 0, 1);

        ceres::AngleAxisToRotationMatrix(h2e2, h2e_rot_mat2);
        cv::Mat X2 = (cv::Mat_<double>(4,4) <<
                h2e_rot_mat2[0], h2e_rot_mat2[3], h2e_rot_mat2[6], h2e2[3],
                h2e_rot_mat2[1], h2e_rot_mat2[4], h2e_rot_mat2[7], h2e2[4],
                h2e_rot_mat2[2], h2e_rot_mat2[5], h2e_rot_mat2[8], h2e2[5],
                0, 0, 0, 1);


        double cam_r1[3] = {double(pnp_r_rel1(0)), double(pnp_r_rel1(1)), double(pnp_r_rel1(2))},
                cam_t1[3] = {double(pnp_t_rel1(0)), double(pnp_t_rel1(1)), double(pnp_t_rel1(2))};

        double cam_r2[3] = {double(pnp_r_rel2(0)), double(pnp_r_rel2(1)), double(pnp_r_rel2(2))},
                cam_t2[3] = {double(pnp_t_rel2(0)), double(pnp_t_rel2(1)), double(pnp_t_rel2(2))};

        ceres::AngleAxisToRotationMatrix(cam_r1, rot_mat_cam1);

        cv::Mat A = (cv::Mat_<double>(4,4) <<
                rot_mat_cam1[0], rot_mat_cam1[3], rot_mat_cam1[6], cam_t1[0],
                rot_mat_cam1[1], rot_mat_cam1[4], rot_mat_cam1[7], cam_t1[1],
                rot_mat_cam1[2], rot_mat_cam1[5], rot_mat_cam1[8], cam_t1[2],
                0, 0, 0, 1);


        ceres::AngleAxisToRotationMatrix(cam_r2, rot_mat_cam2);

        cv::Mat B = (cv::Mat_<double>(4,4) <<
                rot_mat_cam2[0], rot_mat_cam2[3], rot_mat_cam2[6], cam_t2[0],
                rot_mat_cam2[1], rot_mat_cam2[4], rot_mat_cam2[7], cam_t2[1],
                rot_mat_cam2[2], rot_mat_cam2[5], rot_mat_cam2[8], cam_t2[2],
                0, 0, 0, 1);

        cv::Mat X = X1.inv()*X2;
        cv::Mat chain1 = A * X;
        cv::Mat chain2 = X * B;

        rot_mat_inv_1[0] = chain1.at<double>(0,0);
        rot_mat_inv_1[1] = chain1.at<double>(1,0);
        rot_mat_inv_1[2] = chain1.at<double>(2,0);
        rot_mat_inv_1[3] = chain1.at<double>(0,1);
        rot_mat_inv_1[4] = chain1.at<double>(1,1);
        rot_mat_inv_1[5] = chain1.at<double>(2,1);
        rot_mat_inv_1[6] = chain1.at<double>(0,2);
        rot_mat_inv_1[7] = chain1.at<double>(1,2);
        rot_mat_inv_1[8] = chain1.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv_1, rot_vec_1);

        tras_vec_1[0] = chain1.at<double>(0,3);
        tras_vec_1[1] = chain1.at<double>(1,3);
        tras_vec_1[2] = chain1.at<double>(2,3);


        rot_mat_inv_2[0] = chain2.at<double>(0,0);
        rot_mat_inv_2[1] = chain2.at<double>(1,0);
        rot_mat_inv_2[2] = chain2.at<double>(2,0);
        rot_mat_inv_2[3] = chain2.at<double>(0,1);
        rot_mat_inv_2[4] = chain2.at<double>(1,1);
        rot_mat_inv_2[5] = chain2.at<double>(2,1);
        rot_mat_inv_2[6] = chain2.at<double>(0,2);
        rot_mat_inv_2[7] = chain2.at<double>(1,2);
        rot_mat_inv_2[8] = chain2.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv_2, rot_vec_2);

        tras_vec_2[0] = chain2.at<double>(0,3);
        tras_vec_2[1] = chain2.at<double>(1,3);
        tras_vec_2[2] = chain2.at<double>(2,3);


        residuals[0] = (tras_vec_1[0] - tras_vec_2[0]);
        residuals[1] = (tras_vec_1[1] - tras_vec_2[1]);
        residuals[2] = (tras_vec_1[2] - tras_vec_2[2]);
        residuals[3] = (rot_vec_1[0] - rot_vec_2[0]);
        residuals[4] = (rot_vec_1[1] - rot_vec_2[1]);
        residuals[5] = (rot_vec_1[2] - rot_vec_2[2]);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const Eigen::Vector3d &pnp_r_rel1,
                                        const Eigen::Vector3d &pnp_t_rel1,
                                        const Eigen::Vector3d &pnp_r_rel2,
                                        const Eigen::Vector3d &pnp_t_rel2)
    {
        return new ceres::NumericDiffCostFunction<Classic_ax_xb_rel_multi, ceres::CENTRAL, 6, 6, 6>(
                new Classic_ax_xb_rel_multi(pnp_r_rel1, pnp_t_rel1, pnp_r_rel2, pnp_t_rel2));
    }

    Eigen::Vector3d pnp_r_rel1;
    Eigen::Vector3d pnp_t_rel1;
    Eigen::Vector3d pnp_r_rel2;
    Eigen::Vector3d pnp_t_rel2;
};




struct Classic_ax_xb_wo_z_rel
{
    Classic_ax_xb_wo_z_rel(const Eigen::Vector3d &pnp_r_rel,
                       const Eigen::Vector3d &pnp_t_rel,
                       const Eigen::Vector3d &robot_r_rel,
                       const Eigen::Vector3d &robot_t_rel) :
            pnp_r_rel(pnp_r_rel),
            pnp_t_rel(pnp_t_rel),
            robot_r_rel(robot_r_rel),
            robot_t_rel(robot_t_rel){}

    bool operator()(const double* const h2e,
                    double* residuals) const
    {
        double rot_mat_inv_1[9], rot_vec_1[3], tras_vec_1[3], rot_mat2[9], rot_mat_inv2[9], rot_vec_2[3], tras_vec_2[3],
                cal_rot_mat[9], h2e_rot_mat[9], rot_mat_inv_2[9], rot_mat_rob[9], tras_vec_3[3], tras_vec_4[3];

        ceres::AngleAxisToRotationMatrix(h2e, h2e_rot_mat);
        cv::Mat X = (cv::Mat_<double>(4,4) <<
                h2e_rot_mat[0], h2e_rot_mat[3], h2e_rot_mat[6], h2e[3],
                h2e_rot_mat[1], h2e_rot_mat[4], h2e_rot_mat[7], h2e[4],
                h2e_rot_mat[2], h2e_rot_mat[5], h2e_rot_mat[8], h2e[5],
                0, 0, 0, 1);


        double robot_r[3] = {double(robot_r_rel(0)), double(robot_r_rel(1)), double(robot_r_rel(2))},
                robot_t[3] = {double(robot_t_rel(0)), double(robot_t_rel(1)), double(robot_t_rel(2))};

        //###########################################################################################
        ceres::AngleAxisToRotationMatrix(robot_r, rot_mat_rob);

        cv::Mat A = (cv::Mat_<double>(4,4) <<
                rot_mat_rob[0], rot_mat_rob[3], rot_mat_rob[6], robot_t[0],
                rot_mat_rob[1], rot_mat_rob[4], rot_mat_rob[7], robot_t[1],
                rot_mat_rob[2], rot_mat_rob[5], rot_mat_rob[8], robot_t[2],
                0, 0, 0, 1);


        cv::Mat chain1 = A * X;

        rot_mat_inv_1[0] = chain1.at<double>(0,0);
        rot_mat_inv_1[1] = chain1.at<double>(1,0);
        rot_mat_inv_1[2] = chain1.at<double>(2,0);
        rot_mat_inv_1[3] = chain1.at<double>(0,1);
        rot_mat_inv_1[4] = chain1.at<double>(1,1);
        rot_mat_inv_1[5] = chain1.at<double>(2,1);
        rot_mat_inv_1[6] = chain1.at<double>(0,2);
        rot_mat_inv_1[7] = chain1.at<double>(1,2);
        rot_mat_inv_1[8] = chain1.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv_1, rot_vec_1);

        tras_vec_1[0] = chain1.at<double>(0,3);
        tras_vec_1[1] = chain1.at<double>(1,3);
        tras_vec_1[2] = chain1.at<double>(2,3);


        //###########################################################################################

        // Chain 2
        double pnp_r[3] = {double(pnp_r_rel(0)), double(pnp_r_rel(1)), double(pnp_r_rel(2))};
        double pnp_t[3] = {double(pnp_t_rel(0)), double(pnp_t_rel(1)), double(pnp_t_rel(2))};

        ceres::AngleAxisToRotationMatrix(pnp_r, cal_rot_mat);
        cv::Mat B = (cv::Mat_<double>(4,4) <<
                cal_rot_mat[0], cal_rot_mat[3], cal_rot_mat[6], pnp_t[0],
                cal_rot_mat[1], cal_rot_mat[4], cal_rot_mat[7], pnp_t[1],
                cal_rot_mat[2], cal_rot_mat[5], cal_rot_mat[8], pnp_t[2],
                0, 0, 0, 1);

        cv::Mat chain2 = X * B;

        rot_mat_inv_2[0] = chain2.at<double>(0,0);
        rot_mat_inv_2[1] = chain2.at<double>(1,0);
        rot_mat_inv_2[2] = chain2.at<double>(2,0);
        rot_mat_inv_2[3] = chain2.at<double>(0,1);
        rot_mat_inv_2[4] = chain2.at<double>(1,1);
        rot_mat_inv_2[5] = chain2.at<double>(2,1);
        rot_mat_inv_2[6] = chain2.at<double>(0,2);
        rot_mat_inv_2[7] = chain2.at<double>(1,2);
        rot_mat_inv_2[8] = chain2.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv_2, rot_vec_2);

        tras_vec_2[0] = chain2.at<double>(0,3);
        tras_vec_2[1] = chain2.at<double>(1,3);
        tras_vec_2[2] = chain2.at<double>(2,3);

        //###########################################################################################

        residuals[0] = (tras_vec_1[0] - tras_vec_2[0]);
        residuals[1] = (tras_vec_1[1] - tras_vec_2[1]);
        residuals[2] = (tras_vec_1[2] - tras_vec_2[2]);
        residuals[3] = (rot_vec_1[0] - rot_vec_2[0]);
        residuals[4] = (rot_vec_1[1] - rot_vec_2[1]);
        residuals[5] = (rot_vec_1[2] - rot_vec_2[2]);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const Eigen::Vector3d &pnp_r_rel,
                                        const Eigen::Vector3d &pnp_t_rel,
                                        const Eigen::Vector3d &robot_r_rel,
                                        const Eigen::Vector3d &robot_t_rel)
    {
        return new ceres::NumericDiffCostFunction<Classic_ax_xb_wo_z_rel, ceres::CENTRAL, 6, 6>(
                new Classic_ax_xb_wo_z_rel(pnp_r_rel, pnp_t_rel, robot_r_rel, robot_t_rel));
    }

    Eigen::Vector3d pnp_r_rel;
    Eigen::Vector3d pnp_t_rel;
    Eigen::Vector3d robot_r_rel;
    Eigen::Vector3d robot_t_rel;
};




#endif //MEMROC_CALIBRATOR_HANDEYE_CALIBRATOR_H
