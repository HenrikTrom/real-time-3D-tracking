# include "aabb/module_aabb.hpp"

namespace rt3d_tracking {

namespace modules{

AABB::AABB(
    ros::NodeHandle &nh, const std::string &cfg_type, 
    const flir_icp_calib::MultiCameras &cameras
) : cameras(cameras)
{
    // start stages
    std::unique_ptr<Engine<float>> engine;
    detection_inference::load_cfg_engine(cfg_type, this->cfg_det, engine);
    this->detection_nn_stage.reset(new detection_inference::NNStage(this->cfg_det, std::move(engine)));
    this->detection_preprocess_stage.reset(new detection_inference::PreProcessStage(this->cfg_det));
    this->detection_postprocess_stage.reset(new detection_inference::PostProcessStage(this->cfg_det));
    this->correspondance_stage.reset(new stages::Correspondance(this->cfg_corr, this->cameras));
    this->aabbcalc_stage.reset(new stages::AABBCalculate(this->cfg_aabbcalc, this->cameras));
    this->backcrop_stage.reset(new stages::BackCrop(this->cameras, this->cfg_det.input_width, this->cfg_det.input_height));
    this->publish_stage.reset(new stages::PublishAABB(nh, 10));
    // start threads
    while(!this->detection_nn_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while(!this->detection_preprocess_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while(!this->detection_postprocess_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while(!this->correspondance_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while(!this->aabbcalc_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while(!this->backcrop_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while(!this->publish_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    this->ThreadHandlePreprocessNN.reset(new std::thread(&AABB::ThreadPreprocessNN, this)); 
    this->ThreadHandleNNProstprocess.reset(new std::thread(&AABB::ThreadNNPostProcess, this)); 
    this->ThreadHandlePostProcessCorrespondance.reset(new std::thread(&AABB::ThreadPostProcessCorrespondance, this)); 
    this->ThreadHandleCorenspondaceAABB.reset(new std::thread(&AABB::ThreadCorenspondaceAABB, this)); 
    this->ThreadHandleAABBBackCrop.reset(new std::thread(&AABB::ThreadAABBBackCrop, this));

    this->IsReady_flag = true;

}

void AABB::InPost(data::aabb_in &aabb_in){
    this->detection_preprocess_stage->Post(aabb_in.frame);
    this->global_det_q.push(aabb_in.frame);
    this->global_time_q.push(aabb_in.timestamp);          
}

void AABB::ThreadPreprocessNN(){
    while (!this->ShouldClose) {
        std::vector<std::vector<cv::cuda::GpuMat>> NNIn;
        if (this->detection_preprocess_stage->Get(NNIn)){
            this->detection_nn_stage->Post(NNIn);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

void AABB::ThreadNNPostProcess(){
    while (!this->ShouldClose) {
        std::vector<std::vector<std::vector<float>>> DetPostIn;
        if (this->detection_nn_stage->Get(DetPostIn)){
            this->detection_postprocess_stage->Post(DetPostIn);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    
}

void AABB::ThreadPostProcessCorrespondance(){
    #ifdef SINGLE_IMAGE_DEBUG
        bool saved = false;
    #endif
    while (!this->ShouldClose) {
        detection_inference::output_postprocess CorrespondanceIn;
        if (this->detection_postprocess_stage->Get(CorrespondanceIn)){
            #ifdef SINGLE_IMAGE_DEBUG // draw bboxes
                if (!saved){
                    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
                        for (auto& bbox : CorrespondanceIn.bboxes.at(cidx))
                        {
                            cv::rectangle(this->DebugImgs.at(cidx), bbox, cv::Scalar(0, 255, 0));
                        }
                    }
                    saved = true;
                }
            #endif
            this->correspondance_stage->Post(CorrespondanceIn);
            
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

void AABB::ThreadCorenspondaceAABB(){
    while (!this->ShouldClose) {
        std::vector<std::vector<data::BoundingBox2D>> CalcAABBIn;
        if (this->correspondance_stage->Get(CalcAABBIn)){
            this->aabbcalc_stage->Post(CalcAABBIn);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

void AABB::ThreadAABBBackCrop(){
    #ifdef SINGLE_IMAGE_DEBUG
        bool saved = false;
    #endif
    while (!this->ShouldClose) {
        std::vector<data::AABB> CalcAABBsOut;
        if (this->aabbcalc_stage->Get(CalcAABBsOut)){
            #ifdef SINGLE_IMAGE_DEBUG // draw aabbs
                if (!saved){
                    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
                        stages::Draw_AABBs(
                            CalcAABBsOut, DebugImgs.at(cidx), 
                            this->cameras.Cam.at(cidx).M,
                            this->cameras.Cam.at(cidx).K
                        );
                        std::string fname = std::string(CONFIG_DIR)+"/../test/result/images/" + 
                            std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx))+"_aabbs.jpg";
                        spdlog::info("Saved {}", fname);
                        cv::imwrite(fname, DebugImgs.at(cidx));
                    }
                    saved = true;
                }
            #endif

            this->ts = this->global_time_q.front();
            // backcrop (before publish to reduce latency)
            data::backcrop_in BackCropIn;
            BackCropIn.aabbs = CalcAABBsOut;
            BackCropIn.timestamp = ts;
            BackCropIn.frame = this->global_det_q.front();
            this->backcrop_stage->Post(BackCropIn);
            //publish
            data::publishaabb_in PublishIn;
            PublishIn.aabbs = CalcAABBsOut; // move is now ok as CalcAABBsOut is unused
            PublishIn.timestamp = ts;
            this->publish_stage->Post(PublishIn);
            this->global_time_q.pop();
            this->global_det_q.pop();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

void AABB::Terminate(void){
    spdlog::info("----------- Terminating: AABB ----------");
    this->ShouldClose=true;
    this->ThreadHandlePreprocessNN->join();
    this->ThreadHandleNNProstprocess->join();
    this->ThreadHandlePostProcessCorrespondance->join();
    this->ThreadHandleCorenspondaceAABB->join();
    this->ThreadHandleAABBBackCrop->join();

    this->detection_preprocess_stage->Terminate();
    this->detection_nn_stage->Terminate();
    this->detection_postprocess_stage->Terminate();
    this->correspondance_stage->Terminate(); 
    this->aabbcalc_stage->Terminate();
    this->backcrop_stage->Terminate();
    this->publish_stage->Terminate();
}

bool AABB::Get(data::backcrop_out &DataOut)
{
    if (this->backcrop_stage->GetOutFIFOSize()!=0)
    {
        return this->backcrop_stage->Get(DataOut);;
    }
    return false;
}

uint16_t AABB::GetInFIFOSize(void)
{
    return this->detection_preprocess_stage->GetInFIFOSize();
}

uint16_t AABB::GetOutFIFOSize(void)
{
    return this->backcrop_stage->GetOutFIFOSize();
}

bool AABB::IsReady(void)
{
    return this->IsReady_flag;
}

} // namespace modules

} // namespace rt3d_tracking
