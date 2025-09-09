#pragma once
#include "config_parser.hpp"
#include "flirmulticamera/FlirCamera.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "ros-node-interface/interface.hpp"

#include "sys/stat.h"
#include "aabb/module_aabb.hpp"
#include "keypoint/module_kpts.hpp"
#include "keypoint/module_color.hpp"
#include "tracker/stage_publishimages.hpp"
#include <flirmulticamera/videoIO.h>

namespace rt3d_tracking{

namespace modules{

constexpr std::size_t feat_w = 576;
constexpr std::size_t feat_h = 768;

// constexpr std::size_t feat_w = 384;
// constexpr std::size_t feat_h = 512;

class TrackingInterfaceModule : public ros_node_interface::BaseRosInterfaceModule
{
public:
    TrackingInterfaceModule(
        ros::NodeHandle &nh, const config_tracking &cfg_pipeline,
        const flirmulticamera::CameraSettings &cam_settings, 
        const flir_icp_calib::MultiCameras &cameras
    );
    ~TrackingInterfaceModule();
    void Terminate();
    bool start();
    void run();
    const config_tracking cfg;
    const flirmulticamera::CameraSettings cam_settings; 
    const flir_icp_calib::MultiCameras cameras;
    bool ShouldClose = false;

    // camera
    void ThreadCameraSingleImg();
    void ThreadCameraOffline();
    void ThreadCameraOnline();
    std::unique_ptr<std::thread> ThreadHandleCamera;
    std::unique_ptr<flirmulticamera::VideoWriter> writer;

    std::unique_ptr<std::thread> ThreadHandleAABB_KPS;
    void ThreadAABB_KPS();

    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> cpuImgs;
    uint32_t seq = 0;

    // modules
    // std::unique_ptr<stages::PublishImages> stage_publishimages;
    std::unique_ptr<modules::AABB> module_aabb;
    #ifdef TRACK_KPS133
        std::unique_ptr<modules::KPS<133, feat_w, feat_h>> module_kps_full;
    #endif
    #ifdef TRACK_COLOR
        std::unique_ptr<modules::KPS_COLOR> module_color;
    #endif

    std::chrono::steady_clock::time_point now, last; // get publish speed
    std::chrono::milliseconds duration;
    double total_t = 0.;
    double steps = 0.;
};

bool init_trackingInterfaceModule(
    ros::NodeHandle &nh,
    std::unique_ptr<TrackingInterfaceModule> &trackingInterfaceModule
);

} // namespace modules

} // namespace rt3d_tracking