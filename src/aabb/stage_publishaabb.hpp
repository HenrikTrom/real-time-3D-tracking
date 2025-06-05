#pragma once

#include "tracker/StagePublish.hpp"
#include <keiko_msgs/Tracklets.h>
#include "aabb/BoundingBox.h"

namespace rt3d_tracking{

namespace data{

struct publishaabb_in{
    std::vector<data::AABB> aabbs;
    timespec timestamp{};
};

} // namespace data

namespace stages{

class PublishAABB : public stages::StagePublish<
        data::publishaabb_in, 
        keiko_msgs::Tracklets>
{
private:
    void ThreadfunctionPublish();


public:
    PublishAABB(ros::NodeHandle &nh, const int queue_size);
    ~PublishAABB(){};
};


} // namespace stages

} // namespace rt3d_tracking