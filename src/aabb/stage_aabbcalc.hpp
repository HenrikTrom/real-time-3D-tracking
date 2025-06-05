#pragma once
#include <cpp_utils/StageBase.h>
#include "aabb/BoundingBox.h"

#include "AABB_calculator.h"

#include <future>
#include <functional>
#include "tracker/config_parser.hpp"

namespace rt3d_tracking{

namespace stages{

class AABBCalculate : public cpp_utils::StageBase<
        std::vector<std::vector<data::BoundingBox2D>>, 
        std::vector<data::AABB>>
{
private:
    const flir_icp_calib::MultiCameras &cameras;
    const config_aabbcalc &cfg_aabbcalc;
    bool ProcessFunction(
        std::vector<std::vector<data::BoundingBox2D>> &aabbcalc_in,
        std::vector<data::AABB> &aabbcalc_out
    );
    std::function<void(std::vector<data::AABB>&)> postProcess;

public:
    AABBCalculate(const config_aabbcalc &cfg_aabbcalc, const flir_icp_calib::MultiCameras &cameras);
    ~AABBCalculate(){};
    void Terminate(void);
};



}; //namespace stages

}; //namespace rt3d_tracking