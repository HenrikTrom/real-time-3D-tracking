#pragma once

#include <vector>
#include <algorithm>
#include <iostream>

#include "Eigen/Dense"

#include "flir_icp_calib/camera_config.hpp"
#include "flirmulticamera/config_parser.h"
#include "aabb/BoundingBox.h"

namespace rt3d_tracking{

// find the correspondences of the same objects across all the cameras
class CorrespondenceFinder
{
public:
    static std::vector<std::vector<data::BoundingBox2D>> Find(
        std::array<std::vector<data::BoundingBox2D>, flirmulticamera::GLOBAL_CONST_NCAMS> &BBsAllCams, 
        const flir_icp_calib::MultiCameras &Cams, 
        const uint8_t &ObjectAtLeastSeenBy, const float &DistanceThreshold
    );
};

} // namespace rt3d_tracking