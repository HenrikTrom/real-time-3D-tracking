#pragma once
#include <cpp_utils/StageBase.h>
#include <detection_inference/data.hpp>
#include "aabb/BoundingBox.h"
#include "aabb/CorrespondenceFinder.hpp"
#include "tracker/config_parser.hpp"

namespace rt3d_tracking{

namespace stages{

void Draw_BB(data::BoundingBox2D BB, cv::Mat &img);

class Correspondance : public cpp_utils::StageBase<
        detection_inference::output_postprocess, 
        std::vector<std::vector<data::BoundingBox2D>>>
{
private:
    float Threshold;
    const config_correspondance &cfg_corr;
    const flir_icp_calib::MultiCameras &Cameras;
    bool ProcessFunction(
        detection_inference::output_postprocess &corr_in,
        std::vector<std::vector<data::BoundingBox2D>> &corr_out
    );

public:
    Correspondance(const config_correspondance &cfg_corr, const flir_icp_calib::MultiCameras &cameras);
    ~Correspondance(){};
    void Terminate(void);
};

} // namespace stages

} // namespace rt3d_tracking