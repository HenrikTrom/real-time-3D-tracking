#pragma once

#include "tracker/StagePublish.hpp"
#include <keiko_msgs/ImgsListCompressed.h>
#include "../config.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include "flirmulticamera/hardware_constants.h"

namespace rt3d_tracking{

namespace data{

struct publishimages_in{
    std::array<cv::Mat, flirmulticamera::GLOBAL_CONST_NCAMS> images;
    timespec timestamp{};
};

} // namespace data

namespace stages{

class PublishImages : public stages::StagePublish<
    data::publishimages_in,
    keiko_msgs::ImgsListCompressed>
{
public:
    PublishImages(ros::NodeHandle &nh, std::string topic_name, int queue_size, int compression_quality);
    ~PublishImages();
private:
    void ThreadfunctionPublish();
    keiko_msgs::ImgsListCompressed msg_imgs_compressed;
    std::vector<int> compression_params;
    bool test();

};


} // namespace stages

} // namespace rt3d_tracking