#pragma once

#include "cpp_utils/StageBase.h"
#include "cpp_utils/jsontools.h"
#include "aabb/stage_backcrop.hpp"
#include "keypoint/stage_triangulate.hpp"

namespace rt3d_tracking
{

namespace stages
{

class Color : public cpp_utils::StageBase<
    data::kps_in, std::array<KeyPoint3D, 1>
>
{
private:
    uint64_t frameCounter;

    cv::Scalar lower; // Adjust values as needed
    cv::Scalar upper; 
    std::vector<cv::Mat> downloaded;
    const flir_icp_calib::MultiCameras &cameras;
    const int min_cams=10;
    // cv::Mat hsvFrame, mask;
    
    bool ProcessFunction(
        data::kps_in &input, 
        std::array<KeyPoint3D, 1> &output);

public:
    Color(const flir_icp_calib::MultiCameras &cameras, const int &min_cams);
    ~Color();
    void Terminate(void);
};

} // namespace stages

}
