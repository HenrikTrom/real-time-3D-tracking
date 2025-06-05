# include "keypoint/module_color.hpp"


namespace rt3d_tracking{

namespace modules{

KPS_COLOR::KPS_COLOR(
    ros::NodeHandle &nh, 
    const flir_icp_calib::MultiCameras &cameras, 
    const double &fps
) : cameras(cameras), fps(fps) 
{
    //TODO: fix hardcoding
    this->nh=nh;
    this->min_cams=3;
    this->IsReady_flag = false;
    this->msg_kps.id = 4; // color keypoint
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
    this->msg_kps.lifetime = ros::Duration(0.5); // TODO: adapt to fps

    this->pub_keypoints=this->nh.advertise<visualization_msgs::Marker>("kpt_color", 10);
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
    this->ShouldClose=true;
    this->ThreadHandleColorFilter->join();
    this->ThreadHandleFilterPublish->join();
    
    this->color_stage->Terminate(); 
    this->filter_stage->Terminate();
    std::cout<<"--------- Terminating: KPS_COLOR -------"<<std::endl;
};

void KPS_COLOR::InPost(data::kps_in kps_in){
    this->color_stage->Post(kps_in);
    this->global_time_q.push(kps_in.timestamp);          
};
void KPS_COLOR::ThreadColorFilter(){
    while (!this->ShouldClose) {
        std::array<KeyPoint3D, 1> color_out;
        if (this->color_stage->Get(color_out)){
            this->filter_stage->Post(color_out);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
};

void KPS_COLOR::ThreadFilterPublish(){
    while (!this->ShouldClose) {
        std::array<KeyPoint3D, 1> tracked3d;
        if (this->filter_stage->Get(tracked3d)){
            timespec& ts = this->global_time_q.front();
            this->ros_time.sec = ts.tv_sec;
            this->ros_time.nsec = ts.tv_nsec;
            this->msg_kps.header.frame_id = "cam0";
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
            // TODO: Time logging
            // if (true){
                this->now = std::chrono::steady_clock::now();
                this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->now - this->last);
                this->dt = (1-this->alpha)*this->dt+this->alpha*static_cast<double> (this->duration.count());
                // std::cout << this->duration.count() << " ms" << std::endl;
                std::cout << static_cast<int>(this->dt) << " ms" << std::endl;
                // ROS_INFO_STREAM("Publishing bodypose");
                this->last = this->now;
            // }
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
