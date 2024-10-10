//
// Created by davide on 09/10/23.
//
#include "PinholeCameraModel.h"


PinholeCameraModel::PinholeCameraModel(const CameraInfo camera_info){
    img_width_ = camera_info.getImageWidth();
    img_height_ = camera_info.getImageHeight();
    fx_ = camera_info.getFx();
    fy_ = camera_info.getFy();
    cx_ = camera_info.getCx();
    cy_ = camera_info.getCy();
    dist_k0_ = camera_info.getDistK0();
    dist_k1_ = camera_info.getDistK1();
    dist_px_ = camera_info.getDistPx();
    dist_py_ = camera_info.getDistPy();
    dist_k2_ = camera_info.getDistK2();
    dist_k3_ = camera_info.getDistK3();
    dist_k4_ = camera_info.getDistK4();
    dist_k5_ = camera_info.getDistK5();
}
