#pragma once

#include "cpp_utils/StageBase.h"
#include "keypoint/dataType.hpp"
#include "pose_inference/stages.hpp"
#include "flir_icp_calib/camera_config.hpp"
#include "flir_icp_calib/methods.hpp"
#include "flirmulticamera/hardware_constants.h"
#include "common/triangulate.hpp"
#include "config.h"
#include <future>

namespace rt3d_tracking
{

namespace stages
{

KeyPoint3D calc3Dpoint_worker_(
    uint8_t id, std::vector<cv::Point2f> pts2D, 
    flir_icp_calib::MultiCameras cameras
);

KeyPoint3D calc3Dpoint_worker(
    uint8_t id, std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> pts2D, 
    flir_icp_calib::MultiCameras cameras
);

template<uint16_t NKPS>
class Triangulate : public cpp_utils::StageBase<
   std::array<std::array<std::array<float, 2>, NKPS>, flirmulticamera::GLOBAL_CONST_NCAMS>, 
   std::array<KeyPoint3D, NKPS>>
{
private:
uint64_t frameCounter;

bool ProcessFunction(
    std::array<std::array<std::array<float, 2>, NKPS>, flirmulticamera::GLOBAL_CONST_NCAMS> &keypoints, 
    std::array<KeyPoint3D, NKPS> &outputs
){
    std::vector<std::future<KeyPoint3D>> kpts_temp;
    for (uint16_t j = 0; j < NKPS; j++)
    {
        std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> pts;
        for(int cidx = 0; cidx < flirmulticamera::GLOBAL_CONST_NCAMS; cidx++)
        {
            pts.at(cidx).x = keypoints.at(cidx).at(j).at(0);
            pts.at(cidx).y = keypoints.at(cidx).at(j).at(1);
        }
        // parallel here
        // kpts_temp.at(j) = std::async(
        kpts_temp.push_back(std::async( //DID not fix
            std::launch::async, calc3Dpoint_worker, j, pts, this->cameras
        ));
        // outputs.push_back(calc3Dpoint_worker(j, pts, inputs.timestamp, this->min_cams, this->cameras));
    }
    for (uint16_t j = 0; j < NKPS; j++)
    {
        KeyPoint3D tmp = kpts_temp.at(j).get(); //DID not fix
        outputs.at(j) = tmp;
    }
    return true;
};

const flir_icp_calib::MultiCameras &cameras;

public:
Triangulate(const flir_icp_calib::MultiCameras &cameras) : cameras(cameras)
{
    this->ThreadHandle.reset(new std::thread(&Triangulate::ThreadFunction, this));
};
~Triangulate(){};
void Terminate(void){
    this->ShouldClose = true;
    this->ThreadHandle->join();
};

};

} // namespace stages

} // namespace rt3d_tracking
