#pragma once
#include <thread>
#include <queue>
#include "ros/ros.h"

// global publish template 
namespace rt3d_tracking
{
    namespace stages
    {
    template <class InputType, class OutputMsg>
    class StagePublish
    {
    public:
        StagePublish(void){};
        ~StagePublish(void){};
        void init(ros::NodeHandle &nh, std::string topic_name, int queue_size);
        void Terminate();

        std::unique_ptr<std::thread> ThreadHandle;
        std::mutex mtx;
        bool ShouldClose = false;
        virtual void ThreadfunctionPublish(void) = 0;
        void Post(InputType &input);
        std::queue<InputType> InFIFO;
        uint16_t GetInFIFOSize(void);
        bool IsReady(void);
        bool IsReady_flag = false;

        ros::Time ros_time;

        ros::NodeHandle nh;
        ros::Publisher pub;
        OutputMsg msg;
    };

    template <class InputType, class OutputMsg>
    void StagePublish<InputType, OutputMsg>::init(
        ros::NodeHandle &nh, std::string topic_name, int queue_size
    ){
        this->nh=nh;
        this->pub = this->nh.advertise<OutputMsg>(topic_name, queue_size);
    }

    template <class InputType, class OutputMsg>
    void StagePublish<InputType, OutputMsg>::Post(InputType &DataIn)
    {
        std::lock_guard<std::mutex> lck(this->mtx);
        this->InFIFO.push(DataIn);
    }

    template <class InputType, class OutputMsg>
    uint16_t StagePublish<InputType, OutputMsg>::GetInFIFOSize(void)
    {
        return static_cast<uint16_t>(this->InFIFO.size());
    }

    template <class InputType, class OutputMsg>
    void StagePublish<InputType, OutputMsg>::Terminate(void)
    {
        this->ShouldClose = true;
        this->ThreadHandle->join();
    }

    template <class InputType, class OutputMsg>
    bool StagePublish<InputType, OutputMsg>::IsReady(void)
    {
        return this->IsReady_flag;
    }

    } // namespace stages

} // namespace rt3d_tracking