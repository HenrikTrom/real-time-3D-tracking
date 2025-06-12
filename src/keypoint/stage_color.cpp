#include "keypoint/stage_color.hpp"

namespace rt3d_tracking
{

namespace stages
{

Color::Color(
    const flir_icp_calib::MultiCameras &cameras, const int &min_cams
) : cameras(cameras), min_cams(min_cams)
{
    rapidjson::Document color_values;
    if (!cpp_utils::load_json_with_schema(
        std::string(CONFIG_DIR)+"/color_config.json",
        std::string(CONFIG_DIR)+"/color_config.scheme.json",
        65536, color_values
    )){throw std::runtime_error("Could not load color stage config!");}
    auto tmpu = color_values["lower"].GetArray();
    auto tmpo = color_values["upper"].GetArray();
    this->lower=cv::Scalar(tmpu[0].GetInt(), tmpu[1].GetInt(), tmpu[2].GetInt());
    this->upper=cv::Scalar(tmpo[0].GetInt(), tmpo[1].GetInt(), tmpo[2].GetInt());
    this->ThreadHandle.reset(new std::thread(&Color::ThreadFunction, this));
}

Color::~Color(){}

bool Color::ProcessFunction(
    data::kps_in &input, 
    std::array<KeyPoint3D, 1> &output
){
    std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> pts;
    cv::Mat img;
    cv::Mat hsvFrame, mask;
    
    for (std::size_t idx=0; idx<flirmulticamera::GLOBAL_CONST_NCAMS; idx++){
        cv::Point2f pt;
        input.back_crops.at(idx).download(img);
        // back-cropping can be problematic if done at the edges..
        if (img.empty()){
            pts.at(idx) = pt;
            continue;
        }
        // Convert to HSV color space
        cv::cvtColor(img, hsvFrame, cv::COLOR_BGR2HSV);
        // Threshold the image to get only purple colors
        cv::inRange(hsvFrame, this->lower, this->upper, mask);
        img.setTo(cv::Scalar(0, 255, 0), mask);
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            pts.at(idx) = pt;
            continue;
        }
        // Find the largest contour
        size_t largestIdx = 0;
        double maxArea = 0.0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                largestIdx = i;
            }
        }
        // Compute the centroid of the largest contour
        cv::Moments m = cv::moments(contours[largestIdx]);
        if (m.m00 != 0) {
            pt.x = static_cast<float>(m.m10 / m.m00)+input.bboxes.at(idx).x;
            pt.y = static_cast<float>(m.m01 / m.m00)+input.bboxes.at(idx).y;
        } 
        pts.at(idx) = pt;
    }
    // triangulate
    output.at(0) = calc3Dpoint_worker(0, pts, this->cameras);
    return true;
}

void Color::Terminate(void)
{
    this->ShouldClose = true;
    this->ThreadHandle->join();
}

} //namespace stages


}//namespace rt3d_tracking
