#pragma once

#include "tracker/StagePublish.hpp"
#include <keiko_msgs/ImgsListCompressed.h>
#include "../config.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include "flirmulticamera/config.h"

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
    PublishImages(ros::NodeHandle &nh, std::string topic_name, int queue_size, int compression_quality, std::vector<std::string> &sns);
    ~PublishImages();
private:
    void ThreadfunctionPublish();
    std::vector<int> compression_params;
    bool test();
    std::chrono::steady_clock::time_point now, last;
    std::chrono::milliseconds duration;
    double total_t{0};
    double steps = 0.;
    std::array<ros::Publisher, 5> imgs_pubs;
};


} // namespace stages

} // namespace rt3d_tracking