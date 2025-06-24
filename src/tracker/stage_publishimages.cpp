#include "tracker/stage_publishimages.hpp"

namespace rt3d_tracking
{

namespace stages{

PublishImages::PublishImages(
    ros::NodeHandle &nh, std::string topic_name, int queue_size, int compression_quality
){
    this->init(nh, topic_name, queue_size);
    this->compression_params = {cv::IMWRITE_JPEG_QUALITY, compression_quality};
    this->msg.images = std::vector<sensor_msgs::CompressedImage>{
        flirmulticamera::GLOBAL_CONST_NCAMS};
    for (std::size_t camid = 0; camid<flirmulticamera::GLOBAL_CONST_NCAMS; camid++){
        this->msg.images.at(camid).format = "jpeg";
        this->msg.images.at(camid).header.frame_id = std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(camid));
    }
    spdlog::info("VIDEO LOGGING ROS @ {}", topic_name);
    for (int i = 0; i<5; i++){
        this->imgs_pubs.at(i) = this->nh.advertise<sensor_msgs::CompressedImage>("cam"+std::to_string(i), 2);
    }
    this->ThreadHandle.reset(new std::thread(&PublishImages::ThreadfunctionPublish, this));
}

PublishImages::~PublishImages(){}

void PublishImages::ThreadfunctionPublish(void){
    this->IsReady_flag = true;
    while (!this->ShouldClose)
    {
        {
            std::lock_guard<std::mutex> lck(this->mtx); // this took so much time???
            if (this->GetInFIFOSize() != 0)
            {
                {
                    this->last = std::chrono::steady_clock::now();
                    data::publishimages_in &input = this->InFIFO.front();
                    for (std::size_t j = 0; j<flirmulticamera::GLOBAL_CONST_NCAMS; j++)
                    {
                        cv::imencode(".jpg", input.images.at(j), 
                            this->msg.images.at(j).data, 
                            this->compression_params
                        );
                    }
                    this->InFIFO.pop();
                }
                
                // this->imgs_pubs.at(0).publish(this->msg.images.at(0));
                auto last_ = std::chrono::steady_clock::now();
                this->pub.publish(this->msg);
                auto now_ = std::chrono::steady_clock::now();
                auto duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now_ - last_);
                std::cout<<"PubImgsCompressed: "<< (double) duration_.count()<<std::endl;
                this->now = std::chrono::steady_clock::now();
                this->duration = std::chrono::duration_cast<std::chrono::seconds>(this->now - this->last);
                this->total_t += (double) this->duration.count();
                this->steps += 1.;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    if (!this->steps == 0.){
        spdlog::info(
            "Average Publish Time ImagesCompressed: {} milliseconds over {} samples", 
            static_cast<int>(this->total_t/this->steps), static_cast<int>(this->steps)
        );
    }
    else{
        spdlog::info("Average Publish Time ImagesCompressed: 0 milliseconds over 0 samples");
    }
}

} // namespace stages
    
} // namespace rt3d_tracking
