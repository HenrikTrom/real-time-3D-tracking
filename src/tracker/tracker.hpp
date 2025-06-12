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

namespace rt3d_tracking{

namespace modules{

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

    std::unique_ptr<std::thread> ThreadHandleAABB_KPS;
    void ThreadAABB_KPS();

    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> cpuImgs;
    uint32_t seq = 0;
    timespec now{}; // set the timestamp

    // modules
    std::unique_ptr<stages::PublishImages> stage_publishimages;
    std::unique_ptr<modules::AABB> module_aabb;
    #ifdef TRACK_KPS133
        std::unique_ptr<modules::KPS<133, 384, 512>> module_kps_full;
    #endif
    #ifdef TRACK_COLOR
        std::unique_ptr<modules::KPS_COLOR> module_color;
    #endif

};

bool init_trackingInterfaceModule(
    ros::NodeHandle &nh,
    std::unique_ptr<TrackingInterfaceModule> &trackingInterfaceModule
);

} // namespace modules

} // namespace rt3d_tracking