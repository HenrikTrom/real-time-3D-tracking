#include "aabb/stage_publishaabb.hpp"
#include "../config.h"

namespace rt3d_tracking
{

namespace stages{

PublishAABB::PublishAABB(
    ros::NodeHandle &nh, const int queue_size
){
    this->init(nh, std::string(TOPIC_AABB), queue_size);
    this->ThreadHandle.reset(new std::thread(&PublishAABB::ThreadfunctionPublish, this));
}

void PublishAABB::ThreadfunctionPublish(void){
    this->IsReady_flag = true;
    while (!this->ShouldClose)
    {
        {
            std::lock_guard<std::mutex> lck(this->mtx);
            if (!this->InFIFO.empty()){
                data::publishaabb_in &input = this->InFIFO.front();
                this->msg.positions.clear();
                this->msg.widths.clear();
                this->msg.est_vs.clear();
                this->msg.header.stamp = ros::Time(input.timestamp.tv_sec, input.timestamp.tv_nsec);
                for (auto &aabb : input.aabbs){
                    geometry_msgs::Point position, width;
                    // geometry_msgs::Point est_v;
                    position.x = aabb.GetCentre()[0];
                    position.y = aabb.GetCentre()[1];
                    position.z = aabb.GetCentre()[2];
                    width.x = (aabb.GetVertexMax()-aabb.GetVertexMin())[0]*0.5;
                    width.y = (aabb.GetVertexMax()-aabb.GetVertexMin())[1]*0.5;
                    width.z = (aabb.GetVertexMax()-aabb.GetVertexMin())[2]*0.5;
                    uint16_t instance_id = aabb.InstanceID;
                    uint8_t class_id = aabb.ClassID;
                    // tracklet.seenbycameras = aabb.SeenByCameras;
                    this->msg.positions.push_back(position);
                    this->msg.widths.push_back(width);
                    this->msg.instance_ids.push_back(instance_id);
                    this->msg.class_ids.push_back(class_id);
                }

                this->InFIFO.pop();
            }
        }
        this->pub.publish(this->msg);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    // TODO: fix hardcoding
    std::this_thread::sleep_for(std::chrono::microseconds(10));
}

} // namespace stages
    
} // namespace rt3d_tracking
