#include "tracker/stage_publishimages.hpp"

namespace rt3d_tracking
{

namespace stages{

PublishImages::PublishImages(
    ros::NodeHandle &nh, std::string topic_name, int queue_size, int compression_quality
){
    this->init(nh, topic_name, queue_size);
    this->compression_params = {cv::IMWRITE_JPEG_QUALITY, compression_quality};
    this->msg_imgs_compressed.images = std::vector<sensor_msgs::CompressedImage>{
        flirmulticamera::GLOBAL_CONST_NCAMS};
    for (std::size_t camid = 0; camid<flirmulticamera::GLOBAL_CONST_NCAMS; camid++){
        this->msg_imgs_compressed.images.at(camid).format = "jpeg";
        this->msg_imgs_compressed.images.at(camid).header.frame_id = "cam"+std::to_string(camid);
    }
    spdlog::info("VIDEO LOGGING ROS @ {}", topic_name);
    this->ThreadHandle.reset(new std::thread(&PublishImages::ThreadfunctionPublish, this));
}

PublishImages::~PublishImages(){}

void PublishImages::ThreadfunctionPublish(void){
    this->IsReady_flag = true;
    while (!this->ShouldClose)
    {
        {
            std::lock_guard<std::mutex> lck(this->mtx);
            if (!this->InFIFO.empty()){
                data::publishimages_in &input = this->InFIFO.front();
                for (std::size_t j = 0; j<flirmulticamera::GLOBAL_CONST_NCAMS; j++){
                    cv::imencode(".jpg", input.images.at(j), 
                        this->msg_imgs_compressed.images.at(j).data, 
                        this->compression_params
                    );
                }
                this->pub.publish(this->msg_imgs_compressed);
                this->InFIFO.pop();
            }
        }
        this->pub.publish(this->msg);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
}

} // namespace stages
    
} // namespace rt3d_tracking
