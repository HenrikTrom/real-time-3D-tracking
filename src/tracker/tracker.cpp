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
    this->module_kps_full.reset(
        new modules::KPS<133, 192, 256>(
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
    // KPS_COLOR
    // this->stage_kps_color.reset(
    //     new stages::KPS_COLOR(this->nh, this->cameras, this->cam_settings.fps)
    // );
    // while(!this->stage_kps_color->IsReady()){
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }

    spdlog::info("----------------------------------------");
    // camera
    this->ShouldClose = false;
    if (this->cfg.online_mode) {
        this->ThreadHandleCamera.reset( // online tracking
                new std::thread(&TrackingInterfaceModule::ThreadCameraOnline, this)
        );
        spdlog::info("-------------- Mode: Online ------------");
    } 
    else {
        #ifdef SINGLE_IMAGE_DEBUG // use single img
            this->ThreadHandleCamera.reset(
                new std::thread(&TrackingInterfaceModule::ThreadCameraSingleImg, this)
            );
            spdlog::info("------- Mode: DEBUG SINGLE IMAGE -------");
        #else // use video
            this->ThreadHandleCamera.reset(
                new std::thread(&TrackingInterfaceModule::ThreadCameraOffline, this)
            );
            spdlog::info("-------------- Mode: Video -------------");
        #endif
    }
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
    this->module_kps_full->Terminate();
    // this->stage_kps_hand->Terminate();
    // this->stage_kps_color->Terminate();
    #ifdef VIDEO_LOGGING
        this->stage_publishimages->Terminate();
    #endif
};

// Threads
void TrackingInterfaceModule::ThreadAABB_KPS()
{
    bool saved = false;
    while (!this->ShouldClose) {
        data::backcrop_out aabb_out;
        if (this->module_aabb->Get(aabb_out)){
            // this->stage_kps_hand->InPost(aabb_out.rhand);
            // this->stage_kps_lhand->InPost(aabb_out.lhand);
            if (!saved){
                for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++)
                {
                    cv::Mat tmp;
                    aabb_out.body.back_crops.at(cidx).download(tmp);
                    cv::cvtColor(tmp, tmp, cv::COLOR_BGR2RGB);
                    std::string fname = std::string(CONFIG_DIR)+"/../test/result/images/" + 
                    std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx))+"_back_crop_body.jpg";
                    spdlog::info("Saved {}", fname);
                    cv::imwrite(fname, tmp);
                }
                saved = true;
            }
            this->module_kps_full->InPost(aabb_out.body);
            // this->stage_kps_face->InPost(aabb_out.face);
            // this->stage_kps_color->InPost(aabb_out.rhand);
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
    detection_inference::load_image_data(PreProcessIn, this->cpuImgs, fnames, resources);;

    while (!this->module_aabb->IsReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    #ifdef SINGLE_IMAGE_DEBUG
        for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
            this->module_aabb->DebugImgs.at(cidx) = this->cpuImgs.at(cidx).clone();
            this->module_kps_full->DebugImgs.at(cidx) = this->cpuImgs.at(cidx).clone();
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
        std::this_thread::sleep_for(std::chrono::milliseconds(MAX_INFERENCE_SLEEP_MS));
    }
    progressBar.finish();
};

void TrackingInterfaceModule::ThreadCameraOffline()
{
    while (!this->module_aabb->IsReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    data::aabb_in PreprocessAABB;
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> cpuImgsTest;
    std::array<cv::VideoCapture, flirmulticamera::GLOBAL_CONST_NCAMS> video_readers;
    std::string error_msg;
    std::string video_dir = "/home/docker/catkin_ws/experiments/human_data2/videos/";
    std::array<std::string, flirmulticamera::GLOBAL_CONST_NCAMS> filenames;
    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
        filenames.at(cidx) = std::string(RESOURCES_VIDEOS)+"/"+
            std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx))
            +".mp4";
    }
    // const int64_t delay = (int64_t) 1000 / (this->cam_settings.fps+10);
    for (std::size_t cidx = 0; cidx<filenames.size(); cidx++) {
        if (!std::filesystem::exists(filenames.at(cidx)))
        {
            error_msg = "File "+filenames.at(cidx)+" does not exist";
            throw std::runtime_error(error_msg);
        }

        video_readers.at(cidx) = cv::VideoCapture(filenames.at(cidx));
    }

    std::size_t framecount = 0;
    const std::size_t max_frames = video_readers.at(0).get(cv::CAP_PROP_FRAME_COUNT);
    cpp_utils::ProgressBar progressBar(max_frames);
    progressBar.update(framecount);
    while (!this->ShouldClose)
    {
        if (this->module_aabb->GetInFIFOSize() < cpp_utils::MAXINFIFOSIZE)
        {
            for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++) 
            {
                if (!video_readers.at(cidx).read(this->cpuImgs.at(cidx)))
                {
                    spdlog::error("Video {} at {}: Could not get frame");
                }
                else
                {
                    PreprocessAABB.frame.at(cidx).upload(this->cpuImgs.at(cidx));
                    cv::cuda::cvtColor(PreprocessAABB.frame.at(cidx), PreprocessAABB.frame.at(cidx), cv::COLOR_BGR2RGB);
                }
            }
            clock_gettime(CLOCK_REALTIME, &PreprocessAABB.timestamp);
            this->module_aabb->InPost(PreprocessAABB);
            framecount ++;
            if (framecount == max_frames) // reset
            {
                framecount = 0;
                for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++)
                {
                    video_readers.at(cidx).set(cv::CAP_PROP_POS_FRAMES, 0);
                }
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
        spdlog::info("VIDEO LOGGING ROS @ {}", TOPIC_IMAGES_COMPRESSED);
    #else 
        spdlog::info("VIDEO LOGGING ROS: OFF");
    #endif

    std::array<flirmulticamera::Frame, flirmulticamera::GLOBAL_CONST_NCAMS> frame;
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> imgs;
    while(!this->ShouldClose){
        if(fcamerahandler.Get(frame)){
            if (this->module_aabb->GetInFIFOSize() < 10){
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
                
                // std::cout << "Queue Size" << global_det_q.size() );
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