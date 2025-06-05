
#pragma once
#include "pose_inference/stages.hpp"
#include "pose_inference/config_parser.hpp"
#include "tracker/config_parser.hpp"
#include "visualization_msgs/Marker.h"

#include "flir_icp_calib/camera_config.hpp"
#include "sys/stat.h"
#include "aabb/stage_backcrop.hpp"
#include "ros/ros.h"
#include "keypoint/stage_triangulate.hpp"
#include "keypoint/stage_filter.hpp"
#include "keypoint/stage_publishkpts.hpp"

namespace rt3d_tracking{

namespace modules{

template <std::size_t NKPS, std::size_t FEATW, std::size_t FEATH>
class KPS
{
private:
    ros::NodeHandle nh;
    const flir_icp_calib::MultiCameras &cameras;
    pose_inference::config_pose cfg;
    // stages
    std::unique_ptr<pose_inference::PreProcessStage> preprocess_stage;
    std::unique_ptr<pose_inference::NNStage> nn_stage;
    std::unique_ptr<pose_inference::PostProcessStage<NKPS, FEATW, FEATH>> postprocess_stage;
    std::unique_ptr<stages::Triangulate<NKPS>> triangulate_stage;
    std::unique_ptr<stages::Filter<NKPS>> filter_stage;
    std::unique_ptr<stages::PublishKPTS<NKPS>> publish_stage;

    // Threads
    void ThreadPreprocessNN()
    {
        while (!this->ShouldClose) {
            std::vector<std::vector<cv::cuda::GpuMat>> NNIn;
            if (this->preprocess_stage->Get(NNIn)){
                this->nn_stage->Post(NNIn);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    };
    void ThreadNNPostProcess()
    {
        while (!this->ShouldClose) {
            pose_inference::input_postprocess BPPostIn;
            if (this->nn_stage->Get(BPPostIn.features)){
                BPPostIn.bboxes = std::move(this->global_bbox_q.front());
                this->postprocess_stage->Post(BPPostIn);
                this->global_bbox_q.pop();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    };
    void ThreadPostProcessTriangulate()
    {
        bool saved = false;
        while (!this->ShouldClose) {
            std::array<std::array<std::array<float, 2>, NKPS>, flirmulticamera::GLOBAL_CONST_NCAMS> keypoints;
            if (this->postprocess_stage->Get(keypoints)){
                if (!saved){
                    for (std::size_t cidx = 0; cidx< flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
                        for (std::size_t j = 0; j <NKPS; j++){
                            cv::drawMarker(
                                this->DebugImgs.at(cidx), 
                                cv::Point2f(keypoints.at(cidx).at(j).at(0), keypoints.at(cidx).at(j).at(1)), 
                                cv::Scalar(0, 255, 0)
                            );
                        }
                        std::string fname = std::string(CONFIG_DIR)+"/../test/result/images/" + 
                            std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx))+"_kpts.jpg";
                        spdlog::info("Saved {}", fname);
                        cv::imwrite(fname, DebugImgs.at(cidx));
                    }
                    saved = true;
                }
                this->triangulate_stage->Post(keypoints);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    };
    void ThreadTriangulateFilter()
    {
        while (!this->ShouldClose) {
            std::array<KeyPoint3D, NKPS> points3d;
            if (this->triangulate_stage->Get(points3d)){
                std::cout<<"4"<<std::endl;
                this->filter_stage->Post(points3d);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }; 
    void ThreadFilterPublish()
    {
        while (!this->ShouldClose) {
            std::array<KeyPoint3D, NKPS> tracked3d;
            if (this->triangulate_stage->Get(tracked3d)){
                std::cout<<"5"<<std::endl;
                // timespec& ts = this->global_time_q.front();
                this->publish_stage->Post(tracked3d);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    };

    std::unique_ptr<std::thread> ThreadHandlePreprocessNN;
    std::unique_ptr<std::thread> ThreadHandleNNProstprocess;
    std::unique_ptr<std::thread> ThreadHandlePostProcessTriangulate;
    std::unique_ptr<std::thread> ThreadHandleTriangulateFilter; //-> thread AABBpublish
    std::unique_ptr<std::thread> ThreadHandleFilterPublish;

    std::queue<std::array<cv::Rect, flirmulticamera::GLOBAL_CONST_NCAMS>> global_bbox_q;
    const double fps;
    //stage-base-stuff
    bool ShouldClose = false;
    bool IsReady_flag = false;

    
public:
    KPS(
        ros::NodeHandle &nh, const std::string path_pose_cfg, 
        const flir_icp_calib::MultiCameras &cameras, const double &fps,
        const std::string topic_name
    ) : cameras(cameras), fps(fps) 
    {
        this->nh=nh;
        std::unique_ptr<Engine<float>> engine;
        pose_inference::load_cfg_engine(path_pose_cfg, this->cfg, engine);
        this->nn_stage.reset(new pose_inference::NNStage(this->cfg, std::move(engine)));
        this->preprocess_stage.reset(new pose_inference::PreProcessStage(this->cfg));
        this->postprocess_stage.reset(new pose_inference::PostProcessStage<NKPS, FEATW, FEATH>(this->cfg));
        this->triangulate_stage.reset(new stages::Triangulate<NKPS>(this->cameras));
        this->filter_stage.reset(new stages::Filter<NKPS>(this->fps)); 
        this->publish_stage.reset(new stages::PublishKPTS<NKPS>(nh, topic_name, 10)); 
        // start threads
        while(!this->preprocess_stage->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while(!this->nn_stage->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while(!this->postprocess_stage->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while(!this->triangulate_stage->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while(!this->filter_stage->IsReady()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        this->ThreadHandlePreprocessNN.reset(new std::thread(&KPS::ThreadPreprocessNN, this)); 
        this->ThreadHandleNNProstprocess.reset(new std::thread(&KPS::ThreadNNPostProcess, this)); 
        this->ThreadHandlePostProcessTriangulate.reset(new std::thread(&KPS::ThreadPostProcessTriangulate, this)); 
        this->ThreadHandleTriangulateFilter.reset(new std::thread(&KPS::ThreadTriangulateFilter, this)); 
        this->ThreadHandleFilterPublish.reset(new std::thread(&KPS::ThreadFilterPublish, this));
        this->IsReady_flag = true;
        std::cout<<"keypoints stage initialized"<<std::endl;
    };
    ~KPS(){};
    void Terminate(void)
    {
        this->ShouldClose=true;
        this->ThreadHandlePreprocessNN->join();
        this->ThreadHandleNNProstprocess->join();
        this->ThreadHandlePostProcessTriangulate->join(); // TODO: test if combine triangulate, filter and publish
        this->ThreadHandleTriangulateFilter->join();
        this->ThreadHandleFilterPublish->join();

        this->preprocess_stage->Terminate();
        this->nn_stage->Terminate();
        this->postprocess_stage->Terminate();
        this->triangulate_stage->Terminate(); 
        this->filter_stage->Terminate();
        this->publish_stage->Terminate();
        spdlog::info("----------- Terminating: AABB ----------");
        spdlog::info("----------- Terminating: KEYPOINTS {} -", NKPS);
    };

    uint16_t GetInFIFOSize(void)
    {
        return this->preprocess_stage->GetInFIFOSize();
    };
    void InPost(data::kps_in &kps_in)
    {
        this->preprocess_stage->Post(kps_in.back_crops);
        this->global_bbox_q.push(kps_in.bboxes);       
        {
            std::lock_guard<std::mutex> lck(this->publish_stage->mtx_tq);
            this->publish_stage->global_time_q.push(kps_in.timestamp);          
        }        
    };
    bool IsReady(void)
    {
        return this->IsReady_flag;
    };
    #ifdef SINGLE_IMAGE_DEBUG
        std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> DebugImgs;
    #endif

};

}; //namespace modules

} //namespace rt3d_tracking
