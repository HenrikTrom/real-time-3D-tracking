
#pragma once
#include "pose_inference/stages.hpp"
#include "pose_inference/config_parser.hpp"
#include "tracker/config_parser.hpp"
#include "visualization_msgs/Marker.h"

#include "flirmulticamera/FlirCamera.h"
#include "sys/stat.h"
#include "aabb/stage_backcrop.hpp"
#include "ros/ros.h"
#include "keypoint/stage_filter.hpp"
#include "keypoint/stage_color.hpp"

namespace rt3d_tracking{

namespace modules{
    // StageAABB
    // imgs to aabbs
    class KPS_COLOR
    {
    private:
        ros::NodeHandle nh;
        const flir_icp_calib::MultiCameras &cameras;
        // stages
        std::unique_ptr<stages::Color> color_stage;
        std::unique_ptr<stages::Filter<1>> filter_stage;
        // Threads
        void ThreadColorFilter(); 
        void ThreadFilterPublish(); 
        //threadhandles
        std::unique_ptr<std::thread> ThreadHandleColorFilter;
        std::unique_ptr<std::thread> ThreadHandleFilterPublish;

        std::queue<timespec> global_time_q;
        // ros
        ros::Publisher pub_keypoints;
        visualization_msgs::Marker msg_kps;
        uint32_t seq = 0;
        const double fps = 0.; 
        int min_cams=0;
        std::chrono::steady_clock::time_point now, last; // get publish speed
        std::chrono::milliseconds duration; 
        double dt;
        double total_t = 0;
        double steps = 0;    
        ros::Time ros_time;
        //stage-base-stuff
        bool ShouldClose = false;
        bool IsReady_flag = false;

        
        public:
        KPS_COLOR(
            ros::NodeHandle &nh, const std::string &topic_name, 
            const flir_icp_calib::MultiCameras &cameras, const double &fps
        );
        ~KPS_COLOR(){};
        void Terminate(void);

        uint16_t GetInFIFOSize(void);
        void InPost(data::kps_in kps_in);
        bool IsReady(void);

        #ifdef SINGLE_IMAGE_DEBUG
            std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> DebugImgs;
        #endif

};

};



}
