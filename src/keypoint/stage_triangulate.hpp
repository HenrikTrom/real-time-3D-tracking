#pragma once

#include "cpp_utils/StageBase.h"
#include "keypoint/dataType.hpp"
#include "pose_inference/stages.hpp"
#include "flir_icp_calib/camera_config.hpp"
#include "flir_icp_calib/methods.hpp"
#include "flirmulticamera/config.h"
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

template<std::size_t NKPS>
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
    for (std::size_t j = 0; j < NKPS; j++)
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
    for (std::size_t j = 0; j < NKPS; j++)
    {
        KeyPoint3D tmp = kpts_temp.at(j).get(); 
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
void Terminate(void)
{
    this->ShouldClose = true;
    this->ThreadHandle->join();
};
    
};

template<std::size_t NKPS>
void DrawKPTs(
    std::array<KeyPoint3D, NKPS> keypoints3D, 
    flir_icp_calib::MultiCameras cameras, 
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> images
){
    Eigen::MatrixXf pointMatrix(4, NKPS);
    for (int j = 0; j < NKPS; ++j) {
        pointMatrix.col(j).head<3>() = keypoints3D.at(j).coord;
        pointMatrix(3, j) = 1.0f;
    }

    for (std::size_t cidx = 0;cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++)
    {
        Eigen::MatrixXf projected = cameras.Cam.at(cidx).P * pointMatrix;
        Eigen::MatrixXf normalized = projected.array().rowwise() / projected.row(2).array();
        
        for (int j = 0; j < NKPS; ++j) {
            cv::drawMarker(
                images.at(cidx), 
                cv::Point((int) normalized(0, j), (int) normalized(1, j)),
                cv::Scalar(0, 0, 255)
            );
        }
    }


}


} // namespace stages

} // namespace rt3d_tracking
