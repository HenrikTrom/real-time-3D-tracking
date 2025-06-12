#include "tracker/tracker.hpp"

namespace rt3d_tracking{

namespace modules{

TrackingInterfaceModule::TrackingInterfaceModule(
    ros::NodeHandle &nh, 
    const config_tracking &cfg,
    const flirmulticamera::CameraSettings &cam_settings,
    const flir_icp_calib::MultiCameras &cameras
) : cfg(cfg), cam_settings(cam_settings), cameras(cameras)
{
    this->nh = nh;
    this->type = "TrackingInterfaceModule";
};

bool TrackingInterfaceModule::start(){
    spdlog::info("----------------------------------------");
    spdlog::info("---------------- Tracker ---------------");
    spdlog::info("----------------------------------------");

    // AABB
    this->module_aabb.reset(
        new modules::AABB(this->nh, this->cfg.cfg_det, this->cameras)
    );
    while(!this->module_aabb->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // KPS_FULL_133
    #ifdef TRACK_KPS133
        this->module_kps_full.reset(
            new modules::KPS<133, 384, 512>(
                this->nh,
                this->cfg.cfg_pose,
                this->cameras,
                this->cam_settings.fps,
                std::string(BOPDYPOSE133)
            )
        );
        while(!this->module_kps_full->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    #endif
    // KPS_COLOR
    #ifdef TRACK_COLOR
        this->module_color.reset(
            new modules::KPS_COLOR(this->nh, std::string(COLOR_MARKER), this->cameras, this->cam_settings.fps)
        );
        while(!this->module_color->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    #endif
    // camera
    this->ShouldClose = false;
    #ifdef SINGLE_IMAGE_DEBUG // use single img
        this->ThreadHandleCamera.reset(
            new std::thread(&TrackingInterfaceModule::ThreadCameraSingleImg, this)
        );
        spdlog::info("------- Mode: DEBUG SINGLE IMAGE -------");
    #else // use dynamic input
        if (this->cfg.online_mode)
        {
            spdlog::info("-------------- Mode: Online ------------");
            this->ThreadHandleCamera.reset( // online tracking
                    new std::thread(&TrackingInterfaceModule::ThreadCameraOnline, this)
            );
        } 
        else {
            spdlog::info("-------------- Mode: Video -------------");
            this->ThreadHandleCamera.reset(
                new std::thread(&TrackingInterfaceModule::ThreadCameraOffline, this)
            );
        }
    #endif
    
    spdlog::info("----------------------------------------");
    this->ThreadHandleAABB_KPS.reset(new std::thread(&TrackingInterfaceModule::ThreadAABB_KPS, this)); 

    return true;
}

TrackingInterfaceModule::~TrackingInterfaceModule(){};

void TrackingInterfaceModule::Terminate()
{
    this->ShouldClose = true;
    this->ThreadHandleCamera->join();
    this->ThreadHandleAABB_KPS->join();
    this->module_aabb->Terminate();
    #ifdef TRACK_KPS133
        this->module_kps_full->Terminate();
    #endif
    #ifdef TRACK_COLOR
        this->module_color->Terminate();
    #endif
    #ifdef VIDEO_LOGGING
        this->stage_publishimages->Terminate();
    #endif
};

// Threads
void TrackingInterfaceModule::ThreadAABB_KPS()
{
    while (!this->ShouldClose) {
        data::backcrop_out aabb_out;
        if (this->module_aabb->Get(aabb_out)){
            // this->stage_kps_hand->InPost(aabb_out.rhand);
            // this->stage_kps_lhand->InPost(aabb_out.lhand);
            #ifdef TRACK_KPS133
                this->module_kps_full->InPost(aabb_out.body);
            #endif
            #ifdef TRACK_COLOR
                this->module_color->InPost(aabb_out.rhand);
            #endif
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

// Camera
void TrackingInterfaceModule::ThreadCameraSingleImg()
{
    std::string resources = std::string(RESOURCES_IMAGES) + "/";
    std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> fnames;
    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
        fnames.at(cidx) = std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx));
    }
    std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS> PreProcessIn;
    detection_inference::load_image_data(PreProcessIn, this->cpuImgs, fnames, resources);

    #ifdef SINGLE_IMAGE_DEBUG
        for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
            this->module_aabb->DebugImgs.at(cidx) = this->cpuImgs.at(cidx).clone();
            
            #ifdef TRACK_KPS133
                this->module_kps_full->DebugImgs.at(cidx) = this->cpuImgs.at(cidx).clone();
            #endif
            #ifdef TRACK_COLOR
                this->module_color->DebugImgs.at(cidx) = this->cpuImgs.at(cidx).clone();
            #endif
            
        }
    #endif

    cpp_utils::ProgressBar progressBar(MAX_INFERENCE_ITER);
    progressBar.update(0);
    for (std::size_t i = 1; i<=MAX_INFERENCE_ITER; i++)
    {
        if (this->module_aabb->GetInFIFOSize() < cpp_utils::MAXINFIFOSIZE)
        {
            std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS> tmp = PreProcessIn;
            data::aabb_in PreprocessAABB;
            PreprocessAABB.frame = PreProcessIn;
            this->module_aabb->InPost(PreprocessAABB);
            progressBar.update(i);
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(MAX_INFERENCE_SLEEP_MS));
        std::this_thread::sleep_for(std::chrono::milliseconds(14));
    }
    progressBar.finish();
    while (this->module_aabb->GetInFIFOSize() != 0 && this->module_aabb->GetOutFIFOSize() != 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(MAX_INFERENCE_ITER*5));
    this->ShouldClose = true;
};

void TrackingInterfaceModule::ThreadCameraOffline()
{
    data::aabb_in PreprocessAABB;
    const std::string video_dir = std::string(CONFIG_DIR) + "/../test/inputs/videos/";
    const std::string extension = ".mp4";
    std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> fnames = cpp_utils::get_filenames<flirmulticamera::GLOBAL_CONST_NCAMS>(video_dir, extension);

    cpp_utils::SyncVideoIterator iterator(video_dir, fnames);

    std::size_t framecount = 0;
    const std::size_t max_frames = iterator.get_framecount();
    cpp_utils::ProgressBar progressBar(max_frames);
    progressBar.update(framecount);
    while (!this->ShouldClose)
    {
        if (this->module_aabb->GetInFIFOSize() < cpp_utils::MAXINFIFOSIZE)
        {
            iterator.get_next(this->cpuImgs);
            for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++) 
            {
                PreprocessAABB.frame.at(cidx).upload(this->cpuImgs.at(cidx));
                cv::cuda::cvtColor(PreprocessAABB.frame.at(cidx), PreprocessAABB.frame.at(cidx), cv::COLOR_BGR2RGB);
            }
            clock_gettime(CLOCK_REALTIME, &PreprocessAABB.timestamp);
            this->module_aabb->InPost(PreprocessAABB);
            framecount ++;
            if (framecount == max_frames)
            {
                framecount = 0;
                iterator.reset();
            }
            progressBar.update(framecount);
        }
        // publish delay
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    progressBar.finish();
};

void TrackingInterfaceModule::ThreadCameraOnline()
{
    flirmulticamera::FlirCameraHandler fcamerahandler(this->cam_settings);
    if(!fcamerahandler.Configure()){
        throw std::runtime_error("Could not configure camera");
        return;
    };
    fcamerahandler.Start();

    #ifdef VIDEO_LOGGING
        this->stage_publishimages.reset(new stages::PublishImages{this->nh, "images_compressed", 60, 60});
    #else 
        spdlog::info("VIDEO LOGGING ROS: OFF");
    #endif

    std::array<flirmulticamera::Frame, flirmulticamera::GLOBAL_CONST_NCAMS> frame;
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> imgs;
    while(!this->ShouldClose){
        if(fcamerahandler.Get(frame)){
            if (this->module_aabb->GetInFIFOSize() < cpp_utils::MAXINFIFOSIZE){
                data::aabb_in PreprocessAABB;
                clock_gettime(CLOCK_REALTIME, &PreprocessAABB.timestamp);
                for (std::size_t i = 0; i<flirmulticamera::GLOBAL_CONST_NCAMS; i++){
                    this->cpuImgs.at(i) = cv::Mat(
                        frame.at(i).frameData->GetHeight(), 
                        frame.at(i).frameData->GetWidth(), CV_8UC3, 
                        frame.at(i).frameData->GetData()
                    );
                cv::cvtColor(cpuImgs.at(i), cpuImgs.at(i), cv::COLOR_RGB2BGR);
                    PreprocessAABB.frame.at(i).upload(this->cpuImgs.at(i));
                }
                
                this->module_aabb->InPost(PreprocessAABB);
                this->seq++;
                #ifdef VIDEO_LOGGING
                    data::publishimages_in pub_data;
                    pub_data.timestamp = frame.at(0).Timestamp;
                    pub_data.images = this->cpuImgs;
                    this->stage_publishimages->Post(pub_data);
                #endif
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    fcamerahandler.Stop();
};

bool init_trackingInterfaceModule(
    ros::NodeHandle &nh,
    std::unique_ptr<TrackingInterfaceModule> &trackingInterfaceModule
){
    config_tracking cfg;
    if(!load_tracking_config(cfg)){
        throw std::runtime_error("Could not load tracking config");
        return false;
    };
    flirmulticamera::CameraSettings cam_settings;
    if (!flirmulticamera::load_camera_settings(cfg.cfg_multicamera, cam_settings)){
        throw std::runtime_error("Could not load Camera settings");
        return false;
    }
    flir_icp_calib::MultiCameras cameras;
    if (!flir_icp_calib::load_calibration(cfg.cfg_camera_calibration, cameras))
    {
        throw std::runtime_error("Could not load cameras");

        return false;
    }
    // how to make it const:
    trackingInterfaceModule.reset(new TrackingInterfaceModule(nh, cfg, cam_settings, cameras));
    return true;
}

} // namespace modules

} // namespace rt3d_tracking