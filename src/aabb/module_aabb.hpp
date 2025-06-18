#pragma once
#include "tensorrt-cpp-api/engine.h"
#include "detection_inference/utils.hpp"
#include "tracker/config_parser.hpp"

#include "sys/stat.h"
#include <keiko_msgs/Tracklets.h>
#include "aabb/stage_correspondance.hpp"
#include "aabb/stage_aabbcalc.hpp"
#include "aabb/stage_backcrop.hpp"
#include "aabb/stage_publishaabb.hpp"

namespace rt3d_tracking{

namespace data
{

struct aabb_in
{
    std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS> frame;
    timespec timestamp{};
};

}; //namespace data

namespace modules
{
// StageAABB
// imgs to aabbs
class AABB
{
private:
    const flir_icp_calib::MultiCameras &cameras;
    const config_correspondance cfg_corr;
    const config_aabbcalc cfg_aabbcalc;
    detection_inference::config_detection cfg_det;
    // stages
    std::unique_ptr<detection_inference::PreProcessStage> preprocess_stage;
    std::unique_ptr<detection_inference::NNStage> nn_stage;
    std::unique_ptr<detection_inference::PostProcessStage> postprocess_stage;
    std::unique_ptr<stages::Correspondance> correspondance_stage;
    std::unique_ptr<stages::AABBCalculate> aabbcalc_stage;
    std::unique_ptr<stages::PublishAABB> publish_stage;
    std::unique_ptr<stages::BackCrop> backcrop_stage;

    // Threads
    void ThreadPreprocessNN();
    void ThreadNNPostProcess();
    void ThreadPostProcessCorrespondance();
    void ThreadCorenspondaceAABB(); //-> thread AABBpublish
    void ThreadAABBBackCrop();

    std::unique_ptr<std::thread> ThreadHandlePreprocessNN;
    std::unique_ptr<std::thread> ThreadHandleNNProstprocess;
    std::unique_ptr<std::thread> ThreadHandlePostProcessCorrespondance;
    std::unique_ptr<std::thread> ThreadHandleCorenspondaceAABB; //-> thread AABBpublish
    std::unique_ptr<std::thread> ThreadHandleAABBBackCrop;

    // global queues (drop after back crop)
    std::queue<std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS>> global_det_q;
    std::queue<timespec> global_time_q;
    // ros
    ros::Publisher pub_aabb;
    keiko_msgs::Tracklets msg_aabb;
    uint32_t seq = 0;
    timespec ts{}; // set the timestamp
    //stage-base-stuff
    bool ShouldClose = false;
    bool IsReady_flag = false;

    
    public:
    AABB(
        ros::NodeHandle &nh, const std::string &cfg_type, 
        const flir_icp_calib::MultiCameras &cameras
    );
    ~AABB(){};
    void Terminate(void);

    bool Get(data::backcrop_out &DataOut);
    uint16_t GetInFIFOSize(void);
    uint16_t GetOutFIFOSize(void);
    void InPost(data::aabb_in &aabb_in);
    bool IsReady(void);

    #ifdef SINGLE_IMAGE_DEBUG
        std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> DebugImgs;
    #endif

};

};



} // namespace modules
