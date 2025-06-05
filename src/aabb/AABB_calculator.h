#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <vector>

#include "aabb/ConvexHull.h"
#include "aabb/BoundingBox.h"
#include <flir_icp_calib/camera_config.hpp>

namespace rt3d_tracking
{

void Calc3DCentre(
    std::vector<data::BoundingBox2D> BoundingBoxes2D, flir_icp_calib::MultiCameras Cameras, 
    Eigen::VectorXf& Centre3D
);
Eigen::VectorXf Calc2DProjectionTo3D(
    Eigen::VectorXf Point2D, Eigen::VectorXf Centre2D, 
    Eigen::VectorXf Centre3D, flir_icp_calib::CameraParameters Camera
);
data::AABB CalcAABB(
    std::vector<data::BoundingBox2D> BoundingBoxes2D, flir_icp_calib::MultiCameras Cameras
);
void OptimiseAABB(
    data::AABB& initialAABB, std::vector<data::BoundingBox2D> BoundingBoxes2D, bool UseConvexhull, flir_icp_calib::MultiCameras Cameras
);

// Public function
data::AABB CalcAABBfrom2DBBs(
    std::vector<data::BoundingBox2D> BoundingBoxes2D, bool UseConvexhull, flir_icp_calib::MultiCameras Cameras
);

} // namespace rt3d_tracking
