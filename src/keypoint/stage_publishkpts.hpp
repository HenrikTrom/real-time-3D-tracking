#pragma once

#include "tracker/StagePublish.hpp"
#include <visualization_msgs/Marker.h>
#include "aabb/BoundingBox.h"

namespace rt3d_tracking{

namespace stages{

template<std::size_t NKPS>
class PublishKPTS : public stages::StagePublish<
        std::array<KeyPoint3D, NKPS>, 
        visualization_msgs::Marker>
{
public:
    std::queue<timespec> global_time_q;
    std::mutex mtx_tq;
    PublishKPTS(ros::NodeHandle &nh, const std::string &topic_name, const int queue_size)
    {
        this->init(nh, topic_name, queue_size);
        this->msg.header.frame_id = std::string(FRAME_TRACKER);
        this->msg.id = 0;
        this->msg.type = visualization_msgs::Marker::SPHERE_LIST;
        this->msg.action = visualization_msgs::Marker::ADD;
        this->msg.scale.x = 0.01;
        this->msg.scale.y = 0.01;
        this->msg.scale.z = 0.01;
        this->msg.color.a = 1.0;
        this->msg.color.r = 0.0;
        this->msg.color.g = 1.0;
        this->msg.color.b = 0.0;
        this->msg.lifetime = ros::Duration(0.5);
        spdlog::info("Publish Keypoints{} ROS @ {}", NKPS, topic_name);
        this->ThreadHandle.reset(new std::thread(&PublishKPTS::ThreadfunctionPublish, this));
    };
    ~PublishKPTS(){};

private:
    void ThreadfunctionPublish()
    {
        this->IsReady_flag = true;
        this->last = std::chrono::steady_clock::now();
        while (!this->ShouldClose)
        {
                // std::lock_guard<std::mutex> lck(this->mtx);
            if (!this->InFIFO.empty()){
                std::array<KeyPoint3D, NKPS> &input = this->InFIFO.front();
                {
                    // std::lock_guard<std::mutex> lck(this->mtx_tq);
                    // timespec& ts = this->global_time_q.front();
                    // this->msg.header.stamp.sec = ts.tv_sec;
                    // this->msg.header.stamp.nsec = ts.tv_nsec;
                }
                this->msg.points.clear();
                for (size_t i = 0; i < NKPS; i++)
                {
                    geometry_msgs::Point p;
                    p.x = input.at(i).coord(0);
                    p.y = input.at(i).coord(1);
                    p.z = input.at(i).coord(2);
                    this->msg.points.push_back(p);
                }
                this->InFIFO.pop();
                this->now = std::chrono::steady_clock::now();
                this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->now - this->last);
                this->dt = (1.-ALPHA_TIMELOGGING)*this->dt+ALPHA_TIMELOGGING*static_cast<float>(this->duration.count());
                this->total_t += (double) dt;
                this->steps += 1.;
                this->last = this->now;
                this->pub.publish(this->msg);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        if (!this->steps == 0.){
            spdlog::info(
                "Average Publish Time KPTS{}: {} milliseconds over {} samples", 
                NKPS, static_cast<int>(this->total_t/this->steps), static_cast<int>(this->steps)
            );
        }
        else{
            spdlog::info("Average Publish Time KPTS{}: 0 milliseconds over 0 samples", NKPS);
        }
    };
    std::chrono::steady_clock::time_point now, last; // get publish speed
    std::chrono::milliseconds duration; 
    float dt;
    double total_t = 0.;
    double steps = 0.;                         

};

} // namespace stages

} // namespace rt3d_tracking