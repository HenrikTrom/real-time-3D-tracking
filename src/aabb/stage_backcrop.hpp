#pragma once
#include <cpp_utils/StageBase.h>
#include "aabb/BoundingBox.h"
#include <flir_icp_calib/camera_config.hpp>
#include <flirmulticamera/config_parser.h>
#include <opencv2/opencv.hpp>
#include <algorithm>

namespace rt3d_tracking{

namespace data{

struct backcrop_in{
    std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS> frame;
    std::vector<data::AABB> aabbs;
    timespec timestamp{};
};

struct kps_in{
    std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS> back_crops;
    std::array<cv::Rect, flirmulticamera::GLOBAL_CONST_NCAMS> bboxes;
    timespec timestamp{};
};

struct backcrop_out{ //class x nframe
    kps_in lhand;
    kps_in rhand;
    kps_in body;
    kps_in face;
};

} // namespace data

namespace stages{

void Draw_AABBs(
    std::vector<data::AABB> aabbs, cv::Mat &img, 
    Eigen::MatrixXf Extrinsic, Eigen::MatrixXf Intrinsic
);

class BackCrop : public cpp_utils::StageBase<
        data::backcrop_in, 
        data::backcrop_out>
{
private:
    const flir_icp_calib::MultiCameras &cameras;
    const int n_vertices = 8; // n corners
    const float width = 0.f;
    const float height = 0.f;
    bool ProcessFunction(
        data::backcrop_in &backcrop_in,
        data::backcrop_out &backcrop_out
    );
    Eigen::MatrixXf pointMatrix, projected, normalized;
    float xmin, xmax, ymin, ymax;

public:
    BackCrop(
        const flir_icp_calib::MultiCameras &cameras, const float &width, const float &height 
    );
    ~BackCrop(){};
    void Terminate(void);
};

} // namespace stages


} // namespace rt3d_tracking