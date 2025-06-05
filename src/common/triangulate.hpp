#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <opencv2/core/types.hpp>
#include <flir_icp_calib/camera_config.hpp>

namespace rt3d_tracking
{

void Calc3DCentreDynamic(
    std::vector<cv::Point2f> points,
    std::vector<uint8_t> CamIDs,
    flir_icp_calib::MultiCameras Cameras,
    Eigen::VectorXf& Centre3D
);

} // namespace rt3d_tracking