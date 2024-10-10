//
// Created by davide on 09/10/23.
//

#ifndef METRIC_CALIBRATOR_PINHOLECAMERAMODEL_H
#define METRIC_CALIBRATOR_PINHOLECAMERAMODEL_H

#include "CameraInfo.h"

class PinholeCameraModel {
private:
    int img_width_;
    int img_height_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double dist_k0_;
    double dist_k1_;
    double dist_px_;
    double dist_py_;
    double dist_k2_;
    double dist_k3_;
    double dist_k4_;
    double dist_k5_;
public:
    PinholeCameraModel(const CameraInfo camera_info);

    template<typename _T>
    void denormalize(const _T src_pt[2], _T dest_pt[2]) const {
        const _T &x = src_pt[0], &y = src_pt[1];

        _T x2 = x * x, y2 = y * y, xy_dist, r2, r4, r6;
        _T l_radial, h_radial, radial, tangential_x, tangential_y;

        r2 = x2 + y2;
        r6 = r2 * (r4 = r2 * r2);

        l_radial = _T(1.0) + _T(dist_k0_) * r2 + _T(dist_k1_) * r4 + _T(dist_k2_) * r6;
        h_radial = _T(1.0) / (_T(1.0) + _T(dist_k3_) * r2 + _T(dist_k4_) * r4 + _T(dist_k5_) * r6);
        radial = l_radial * h_radial;
        xy_dist = _T(2.0) * x * y;
        tangential_x = xy_dist * _T(dist_px_) + _T(dist_py_) * (r2 + _T(2.0) * x2);
        tangential_y = xy_dist * _T(dist_py_) + _T(dist_px_) * (r2 + _T(2.0) * y2);

        _T x_dist = x * radial + tangential_x;
        _T y_dist = y * radial + tangential_y;

        dest_pt[0] = x_dist * _T(fx_) + _T(cx_);
        dest_pt[1] = y_dist * _T(fy_) + _T(cy_);
    }

    template<typename _T>
    void project(const _T scene_pt[3], _T img_pt[2]) const {
        _T norm_img_pt[2] = {scene_pt[0], scene_pt[1]};
        const _T &z = scene_pt[2];
        _T inv_z = _T(1.0) / z;
        norm_img_pt[0] *= inv_z;
        norm_img_pt[1] *= inv_z;

        denormalize(norm_img_pt, img_pt);
    }
};

#endif //METRIC_CALIBRATOR_PINHOLECAMERAMODEL_H
