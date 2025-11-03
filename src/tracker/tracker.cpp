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

bool TrackingInterfaceModule::start()
{
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
            new modules::KPS<133, feat_w, feat_h>(
                this->nh,
                this->cfg.cfg_pose,
                this->cameras,
                this->cam_settings.fps,
                this->cfg.maf_window_size,
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
            new modules::KPS_COLOR(this->nh, std::string(COLOR_MARKER), this->cameras, this->cam_settings.fps, this->cfg.maf_window_size)
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
    // #ifdef VIDEO_LOGGING
    //     this->stage_publishimages->Terminate();
    // #endif
    if (!this->steps == 0.){
    spdlog::info(
        "Average Camera Cycle: {} milliseconds over {} samples, should be {} ms", 
        static_cast<int>(this->total_t/this->steps), static_cast<int>(this->steps), (int ) (1000./this->cam_settings.fps)
    );
    }
    else{
        spdlog::info("Average Camera Cycle: 0 milliseconds over 0 samples");
    }
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
    
    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++) {
        fnames.at(cidx) = this->cam_settings.SNs.at(cidx);
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
    for (std::size_t i = 1; i<=MAX_INFERENCE_ITER; i++) {
        if (this->module_aabb->GetInFIFOSize() < cpp_utils::MAXINFIFOSIZE)
        {
            std::array<cv::cuda::GpuMat, flirmulticamera::GLOBAL_CONST_NCAMS> tmp = PreProcessIn;
            data::aabb_in PreprocessAABB;
            PreprocessAABB.frame = PreProcessIn;
            clock_gettime(CLOCK_MONOTONIC, &PreprocessAABB.timestamp);
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
    std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> fnames;
    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
        fnames.at(cidx) = this->cam_settings.SNs.at(cidx);
    }

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
            clock_gettime(CLOCK_MONOTONIC, &PreprocessAABB.timestamp);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    spdlog::info("VIDEO LOGGING: ON");
    
    std::string ts = cpp_utils::get_timestamp();
    std::string save_dir = cam_settings.save_dir+"/"+ts;
    if (!std::filesystem::create_directory(save_dir)){
        std::string msg = "Could not create " + save_dir;
        spdlog::error(msg);
        throw std::runtime_error(msg);
    }

    this->writer.reset(new flirmulticamera::VideoWriter{
        static_cast<uint32_t>(cam_settings.width), 
        static_cast<uint32_t>(cam_settings.height), 
        static_cast<float>(cam_settings.fps),
        static_cast<std::string>(cam_settings.codec),
        static_cast<std::string>("BGR8") // use BGR8 because the tracker expects it, avoid double conversion
    });
    std::vector<std::string> video_filenames{flirmulticamera::GLOBAL_CONST_NCAMS};   
    for (int cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++)
    {
        video_filenames.at(cidx) = save_dir+"/"+cam_settings.SNs.at(cidx)+".mp4";
        spdlog::info("Opening file name: {}", video_filenames.at(cidx));
    }
    writer->Open(video_filenames);
    #endif

    std::array<flirmulticamera::Frame, flirmulticamera::GLOBAL_CONST_NCAMS> frame;
    this->last = std::chrono::steady_clock::now();
    while(!this->ShouldClose){
        if(fcamerahandler.Get(frame))
        {
            if (this->module_aabb->GetInFIFOSize() < 5) {
                // #ifdef TRACK_COLOR
                //     data::kps_in color_in;
                //     clock_gettime(CLOCK_MONOTONIC, &color_in.timestamp);
                // #endif

                data::aabb_in PreprocessAABB;
                clock_gettime(CLOCK_MONOTONIC, &PreprocessAABB.timestamp);

                for (std::size_t i = 0; i<flirmulticamera::GLOBAL_CONST_NCAMS; i++){
                    this->cpuImgs.at(i) = cv::Mat(
                        frame.at(i).frameData->GetHeight(), 
                        frame.at(i).frameData->GetWidth(), CV_8UC3, 
                        frame.at(i).frameData->GetData()
                    );
                    cv::cvtColor(cpuImgs.at(i), cpuImgs.at(i), cv::COLOR_RGB2BGR);
                    PreprocessAABB.frame.at(i).upload(this->cpuImgs.at(i));
                    // #ifdef TRACK_COLOR
                    //     color_in.back_crops.at(i).upload(this->cpuImgs.at(i));
                    //     color_in.bboxes.at(i) = cv::Rect(0, 0, this->cam_settings.width, this->cam_settings.height);
                    // #endif
                }
                // #ifdef TRACK_COLOR
                //     this->module_color->InPost(color_in);
                // #endif

                this->module_aabb->InPost(PreprocessAABB);
                this->seq++;
            }

            #ifdef VIDEO_LOGGING
            std::vector<Spinnaker::ImagePtr> buffer{};
            for (auto &img : frame) {
                buffer.push_back(img.frameData);
            }
            writer->Write(buffer);
            #endif
            
            this->now = std::chrono::steady_clock::now();
            this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->now - this->last);
            this->total_t += (double) this->duration.count();
            this->steps += 1.;
            this->last = this->now;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    writer->Close();
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
    if (cfg.online_mode)
    {
        if (!flirmulticamera::load_camera_settings(cfg.cfg_multicamera, cam_settings)){
            throw std::runtime_error("Could not load Camera settings");
            return false;
        }
    }
    else
    {
        cam_settings.fps = 50;
        cam_settings.width = 1024;
        cam_settings.height = 768;
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