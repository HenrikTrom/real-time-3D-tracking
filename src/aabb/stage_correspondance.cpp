#include "aabb/stage_correspondance.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace rt3d_tracking{

namespace stages{

Correspondance::Correspondance(
    const config_correspondance &cfg_corr, 
    const flir_icp_calib::MultiCameras &cameras
) : cfg_corr(cfg_corr), Cameras(cameras)
{
    this->type = "Correspondance";
    this->ThreadHandle.reset(new std::thread(&Correspondance::ThreadFunction, this));
};


void Draw_BB(data::BoundingBox2D BB, cv::Mat &img)
{
    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::Scalar colour{255, 0, 0};
    cv::rectangle(
        img, 
        cv::Point(static_cast<int>(BB.Vertex_TopLeft(0)), static_cast<int>(BB.Vertex_TopLeft(1))),
        cv::Point(static_cast<int>(BB.Vertex_BottomRight(0)), static_cast<int>(BB.Vertex_BottomRight(1))),
        colour, thickness, lineType);
}



bool Correspondance::ProcessFunction(
    detection_inference::output_postprocess &corr_in,
    std::vector<std::vector<data::BoundingBox2D>> &corr_out
){
    #ifdef USE_DEBUG_TIME_LOGGING
        this->t1 = std::chrono::steady_clock::now();
    #endif
    std::array<
        std::array<
            std::vector<data::BoundingBox2D>, 
            flirmulticamera::GLOBAL_CONST_NCAMS
        >, 
        N_CLASSES_DETECTIOM
    > BBs_classes_cams;
    
    corr_out.clear();
    for (uint8_t clsid = 0; clsid<N_CLASSES_DETECTIOM; clsid++){
        for (std::size_t camid = 0; camid<flirmulticamera::GLOBAL_CONST_NCAMS; camid++){
            std::vector<data::BoundingBox2D> bboxes;
            for (size_t bbidx = 0; bbidx<corr_in.labels.at(camid).size(); bbidx++){
                if (clsid==corr_in.labels.at(camid).at(bbidx)){
                    data::BoundingBox2D bb(
                        static_cast<float>(corr_in.bboxes.at(camid).at(bbidx).x), //xmin
                        static_cast<float>(corr_in.bboxes.at(camid).at(bbidx).y), //ymin
                        static_cast<float>(corr_in.bboxes.at(camid).at(bbidx).x+corr_in.bboxes.at(camid).at(bbidx).width), //xmax
                        static_cast<float>(corr_in.bboxes.at(camid).at(bbidx).y+corr_in.bboxes.at(camid).at(bbidx).height), //ymax
                        clsid, corr_in.scores.at(camid).at(bbidx),
                        camid
                    );
                    bboxes.push_back(bb);
                }
            }
            BBs_classes_cams.at(clsid).at(camid) = bboxes;
        }
    }

    for (uint8_t classid = 0; classid < this->cfg_corr.NumOfClasses; classid++)
    {
        if (classid == 0)
        {
            this->Threshold = this->cfg_corr.NonSolidObjectDistanceThreshold;
        }
        else
        {
            this->Threshold = this->cfg_corr.SolidObjectDistanceThreshold;
        }
        auto objects = CorrespondenceFinder::Find(
            BBs_classes_cams[classid], this->Cameras, 
            this->cfg_corr.ObjectAtLeastSeenBy, this->Threshold
        );

        if (objects.size() > 0)
        {
            // why not pushback??
            // Avoids nested sequence -> like .extend in python
            corr_out.insert(
                corr_out.end(), objects.begin(), objects.end()
            );
        }
        // TODO: use array
        // corr out: cls-inst-corr
    }
    
    // labels are IN BBs/AABBs
    #if defined(USE_DEBUG_TIME_LOGGING)
        this->t2 = std::chrono::steady_clock::now();
        this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->t2 - this->t1);
        this->n_iterations++;
        this->total_dt += this->duration;
    #endif
    return true;

};

void Correspondance::Terminate(void){
    this->ShouldClose = true;
    this->ThreadHandle->join();
    #ifdef USE_DEBUG_TIME_LOGGING
        if (this->n_iterations != 0){
            spdlog::info(
                "Average {} Time: {} milliseconds over {} samples",
                this->type, this->total_dt.count()/this->n_iterations, 
                this->n_iterations
            );
        }
        else{
            spdlog::info(
                "Average {} Time: 0 milliseconds over 0 samples",
                this->type
            );
        }
    #endif
};

} // namespace stages

} // namespace rt3d_tracking