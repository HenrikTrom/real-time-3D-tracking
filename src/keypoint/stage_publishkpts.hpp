#pragma once

#include "tracker/StagePublish.hpp"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "aabb/BoundingBox.h"

namespace rt3d_tracking{

namespace stages{

template<std::size_t NKPS>
class PublishKPTS : public stages::StagePublish<
        std::array<KeyPoint3D, NKPS>, 
        sensor_msgs::PointCloud2>
{
public:
    std::queue<timespec> global_time_q;
    std::mutex mtx_tq;
    PublishKPTS(ros::NodeHandle &nh, const std::string &topic_name, const int queue_size)
    {
        this->init(nh, topic_name, queue_size);
        this->msg.header.frame_id = std::string(FRAME_TRACKER);
        spdlog::info("Publish Keypoints{} ROS @ {}", NKPS, topic_name);
        this->msg.fields.resize(4);
        this->msg.fields[0].name = "x";
        this->msg.fields[0].count = 1;
        this->msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        this->msg.fields[0].offset = 0;

        this->msg.fields[1].name = "y";
        this->msg.fields[1].count = 1;
        this->msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        this->msg.fields[1].offset = 4;

        this->msg.fields[2].name = "z";
        this->msg.fields[2].count = 1;
        this->msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        this->msg.fields[2].offset = 8;

        this->msg.fields[3].name = "intensity";
        this->msg.fields[3].count = 1;
        this->msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        this->msg.fields[3].offset = 12;

        this->msg.point_step = 4*4; // 4 entries x 4bytes
        this->msg.is_bigendian = false;

        this->msg.height = 1;
        this->msg.width = (uint32_t) NKPS;

        this->msg.row_step = this->msg.point_step * this->msg.width;
        this->msg.is_dense = true;
        this->msg.data.resize(this->msg.row_step);

        this->ThreadHandle.reset(new std::thread(&PublishKPTS::ThreadfunctionPublish, this));
    };
    ~PublishKPTS(){};

private:
    void ThreadfunctionPublish()
    {
        this->IsReady_flag = true;
        while (!this->ShouldClose)
        {
            std::lock_guard<std::mutex> lck(this->mtx);
            if (!this->InFIFO.empty())
            {
                this->last = std::chrono::steady_clock::now();
                std::array<KeyPoint3D, NKPS> &input = this->InFIFO.front();
                {
                    std::lock_guard<std::mutex> lck(this->mtx_tq);
                    this->ts = this->global_time_q.front();
                    this->global_time_q.pop();
                    this->msg.header.stamp.sec = ts.tv_sec;
                    this->msg.header.stamp.nsec = ts.tv_nsec;
                }

                for (std::size_t i = 0; i < NKPS; ++i) {
                    const Eigen::Vector3f& pt = input.at(i).coord;
                    std::memcpy(&this->msg.data[i * this->msg.point_step + 0], &pt.x(), sizeof(float));
                    std::memcpy(&this->msg.data[i * this->msg.point_step + 4], &pt.y(), sizeof(float));
                    std::memcpy(&this->msg.data[i * this->msg.point_step + 8], &pt.z(), sizeof(float));
                    // this->msg.data[i * this->msg.point_step + 12] = 0.5;
                }
                this->InFIFO.pop();
                this->now = std::chrono::steady_clock::now();
                this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->now - this->last);
                this->total_t += (double) this->duration.count();
                this->steps += 1.;
                this->pub.publish(this->msg);
                
                timespec _ts, _diff;
                clock_gettime(CLOCK_MONOTONIC, &_ts);
                _diff.tv_sec = _ts.tv_sec - this->ts.tv_sec;
                _diff.tv_nsec = _ts.tv_nsec - this->ts.tv_nsec;
                if (_diff.tv_nsec < 0) {
                    _diff.tv_sec -= 1;
                    _diff.tv_nsec += 1000000000;
                }

                long long elapsed_ms = _diff.tv_sec * 1000LL + _diff.tv_nsec / 1000000;
                this->total_lat += elapsed_ms;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        if (!this->steps == 0.){
            spdlog::info(
                "Average Publish Time KPTS{}: {} milliseconds over {} samples", 
                NKPS, static_cast<int>(this->total_t/this->steps), static_cast<int>(this->steps)
            );
            spdlog::info(
                "Average Latency KPTS{}: {} milliseconds over {} samples", 
                NKPS, this->total_lat/(long long)this->steps, static_cast<int>(this->steps)
            );
        }
        else{
            spdlog::info("Average Publish Time KPTS{}: 0 milliseconds over 0 samples", NKPS);
        }
    };
    std::chrono::steady_clock::time_point now, last;
    std::chrono::milliseconds duration;
    double total_t{0};
    double steps = 0.;
    long long total_lat{0};
    timespec ts;       

};

} // namespace stages

} // namespace rt3d_tracking