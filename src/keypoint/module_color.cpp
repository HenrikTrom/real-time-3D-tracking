# include "keypoint/module_color.hpp"


namespace rt3d_tracking{

namespace modules{

KPS_COLOR::KPS_COLOR(
    ros::NodeHandle &nh, const std::string &topic_name,
    const flir_icp_calib::MultiCameras &cameras, 
    const double &fps
) : cameras(cameras), fps(fps) 
{
    this->nh=nh;
    this->IsReady_flag = false;
    this->msg_kps.id = 0; 
    this->msg_kps.type = visualization_msgs::Marker::SPHERE_LIST;
    this->msg_kps.action = visualization_msgs::Marker::ADD;
    this->msg_kps.scale.x = 0.005;
    this->msg_kps.scale.y = 0.005;
    this->msg_kps.scale.z = 0.005;
    this->msg_kps.color.a = 1.0;
    this->msg_kps.color.r = 0.0;
    this->msg_kps.color.g = 1.0;
    this->msg_kps.color.b = 0.0;
    this->msg_kps.pose.orientation.w = 1.0;
    this->msg_kps.lifetime = ros::Duration(1./fps);

    this->pub_keypoints=this->nh.advertise<visualization_msgs::Marker>(topic_name, 60);
    // stages
    this->color_stage.reset(new stages::Color(this->cameras, this->min_cams));
    while(!this->color_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    this->filter_stage.reset(new stages::Filter<1>(this->fps));
    while(!this->filter_stage->IsReady()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // start threads
    this->ThreadHandleColorFilter.reset(new std::thread(&KPS_COLOR::ThreadColorFilter, this)); 
    this->ThreadHandleFilterPublish.reset(new std::thread(&KPS_COLOR::ThreadFilterPublish, this));

    this->IsReady_flag = true;
};
void KPS_COLOR::Terminate(void){
    spdlog::info("--------- Terminating: KPS_COLOR -------");
    this->ShouldClose=true;
    this->ThreadHandleColorFilter->join();
    this->ThreadHandleFilterPublish->join();
    
    this->color_stage->Terminate(); 
    this->filter_stage->Terminate();
    if (!this->steps == 0){
        spdlog::info(
            "Average Publish Time Color: {} milliseconds over {} samples", 
            static_cast<int>(this->total_t/this->steps), static_cast<int>(this->steps)
        );
    }
    else{
        spdlog::info("Average Publish Time Color: 0 milliseconds over 0 samples");
    }
};

void KPS_COLOR::InPost(data::kps_in kps_in){
    this->color_stage->Post(kps_in);
    this->global_time_q.push(kps_in.timestamp);          
};
void KPS_COLOR::ThreadColorFilter(){
    #ifdef SINGLE_IMAGE_DEBUG
        bool saved = false;
    #endif
    while (!this->ShouldClose) {
        std::array<KeyPoint3D, 1> color_out;
        if (this->color_stage->Get(color_out)){
            #ifdef SINGLE_IMAGE_DEBUG
                if (!saved)
                {
                    Eigen::Vector4f points, projected, normalized;
                    points.head<3>() = color_out.at(0).coord;
                    points(3) = 1.f;
                    for (std::size_t cidx = 0; cidx< flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
                        projected = this->cameras.Cam.at(cidx).P * points;
                        normalized = projected/projected(2);
                        cv::drawMarker(
                            this->DebugImgs.at(cidx), 
                            cv::Point((int)normalized(0), (int)normalized(1)), 
                            cv::Scalar(0, 0, 255)
                        );
                        std::string fname = std::string(CONFIG_DIR)+"/../test/result/images/" + 
                            std::string(flirmulticamera::GLOBAL_CONST_CAMERA_SERIAL_NUMBERS.at(cidx))+"_color.jpg";
                        spdlog::info("Saved {}", fname);
                        cv::imwrite(fname, this->DebugImgs.at(cidx));
                    }
                    saved = true;
                }
            #endif
            this->filter_stage->Post(color_out);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
};

void KPS_COLOR::ThreadFilterPublish(){
    this->last = std::chrono::steady_clock::now();
    while (!this->ShouldClose) {
        std::array<KeyPoint3D, 1> tracked3d;
        if (this->filter_stage->Get(tracked3d)){
            timespec& ts = this->global_time_q.front();
            this->ros_time.sec = ts.tv_sec;
            this->ros_time.nsec = ts.tv_nsec;
            this->msg_kps.header.frame_id = std::string(FRAME_TRACKER);
            this->msg_kps.header.stamp = this->ros_time;
            this->msg_kps.points.clear();
            for (size_t i = 0; i < tracked3d.size(); i++)
            {
                geometry_msgs::Point p;
                p.x = tracked3d.at(i).coord(0);
                p.y = tracked3d.at(i).coord(1);
                p.z = tracked3d.at(i).coord(2);
                this->msg_kps.points.push_back(p);
            }
            this->pub_keypoints.publish(this->msg_kps);
            this->global_time_q.pop();
            this->now = std::chrono::steady_clock::now();
            this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->now - this->last);
            this->dt = (1.-ALPHA_TIMELOGGING)*this->dt+ALPHA_TIMELOGGING*static_cast<float>(this->duration.count());
            this->total_t += (double) dt;
            this->steps += 1.;
            this->last = this->now;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
};

uint16_t KPS_COLOR::GetInFIFOSize(void){
    return this->color_stage->GetInFIFOSize();
};
bool KPS_COLOR::IsReady(void){
    return this->IsReady_flag;
};

} //namespace stages

} // namespace rt3d_tracking
