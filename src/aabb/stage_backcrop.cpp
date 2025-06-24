#include "aabb/stage_backcrop.hpp"

namespace rt3d_tracking{

namespace stages{


void Draw_AABBs(
    std::vector<data::AABB> aabbs, cv::Mat &img, 
    Eigen::MatrixXf Extrinsic, Eigen::MatrixXf Intrinsic
){

    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::Scalar color{51, 51, 255};

    for (auto aabb : aabbs){
        std::vector<cv::Point> points;
        Eigen::Vector3f temp;
        cv::Point Point_temp;

        for (uint8_t i = 0; i < 8; i++){
            temp = Intrinsic * Extrinsic.block<3, 4>(0, 0) * aabb.Vertices[i];
            temp /= temp[2];
            Point_temp.x = int(temp[0]);
            Point_temp.y = int(temp[1]);
            points.push_back(Point_temp);
        }

        cv::line(img, points[0], points[1], color, thickness, lineType);
        cv::line(img, points[1], points[2], color, thickness, lineType);
        cv::line(img, points[2], points[3], color, thickness, lineType);
        cv::line(img, points[3], points[0], color, thickness, lineType);

        cv::line(img, points[4], points[5], color, thickness, lineType);
        cv::line(img, points[5], points[1], color, thickness, lineType);
        cv::line(img, points[1], points[0], color, thickness, lineType);
        cv::line(img, points[0], points[4], color, thickness, lineType);

        cv::line(img, points[7], points[6], color, thickness, lineType);
        cv::line(img, points[6], points[5], color, thickness, lineType);
        cv::line(img, points[5], points[4], color, thickness, lineType);
        cv::line(img, points[4], points[7], color, thickness, lineType);

        cv::line(img, points[3], points[2], color, thickness, lineType);
        cv::line(img, points[2], points[6], color, thickness, lineType);
        cv::line(img, points[6], points[7], color, thickness, lineType);
        cv::line(img, points[7], points[3], color, thickness, lineType);

        cv::line(img, points[4], points[0], color, thickness, lineType);
        cv::line(img, points[0], points[3], color, thickness, lineType);
        cv::line(img, points[3], points[7], color, thickness, lineType);
        cv::line(img, points[7], points[4], color, thickness, lineType);

        cv::line(img, points[5], points[1], color, thickness, lineType);
        cv::line(img, points[1], points[2], color, thickness, lineType);
        cv::line(img, points[2], points[6], color, thickness, lineType);
        cv::line(img, points[6], points[5], color, thickness, lineType);
    }
}

BackCrop::BackCrop(
    const flir_icp_calib::MultiCameras &cameras, const float &width, const float &height
) : cameras(cameras), width(width), height(height)
{
    this->pointMatrix=Eigen::MatrixXf(4, this->n_vertices);
    this->type = "BackCrop";
    this->ThreadHandle.reset(new std::thread(&BackCrop::ThreadFunction, this));
}

bool BackCrop::ProcessFunction(
    data::backcrop_in &backcrop_in,
    data::backcrop_out &backcrop_out
){
    #ifdef USE_DEBUG_TIME_LOGGING
        this->t1 = std::chrono::steady_clock::now();
    #endif
    std::array<std::vector<int>, 5> xvals, yvals;
    std::array<std::vector<int>, 5> xvals_rh, yvals_rh;
    for (auto & aabb: backcrop_in.aabbs)
    {
        for (int i = 0; i < this->n_vertices; ++i) {
            this->pointMatrix.col(i).head<3>() = aabb.Vertices[i].head<3>();
            this->pointMatrix(3, i) = 1.0f;
        }

        for (std::size_t cidx = 0;cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
            this->projected = this->cameras.Cam.at(cidx).P * this->pointMatrix;
            
            // Normalize third row to get 2D coordinates
            this->normalized = this->projected.array().rowwise() / projected.row(2).array();

            this->xmin = std::clamp(this->normalized.row(0).minCoeff(), 0.f, this->width);
            this->xmax = std::clamp(this->normalized.row(0).maxCoeff(), 0.f, this->width);
            this->ymin = std::clamp(this->normalized.row(1).minCoeff(), 0.f, this->height);
            this->ymax = std::clamp(this->normalized.row(1).maxCoeff(), 0.f, this->height);
            
            #ifdef TRACK_COLOR
            if (aabb.ClassID == 1){
                // xvals_rh.at(cidx).push_back((int) xmin);
                // xvals_rh.at(cidx).push_back((int) xmax);
                // yvals_rh.at(cidx).push_back((int) ymin);
                // yvals_rh.at(cidx).push_back((int) ymax);
                xvals_rh.at(cidx).push_back((int) std::clamp(this->xmin-50, 0.f, this->width));
                xvals_rh.at(cidx).push_back((int) std::clamp(this->xmax+50, 0.f, this->width));
                yvals_rh.at(cidx).push_back((int) std::clamp(this->ymin-50, 0.f, this->height));
                yvals_rh.at(cidx).push_back((int) std::clamp(this->ymax+50, 0.f, this->height));



            }
            #endif
            #ifdef TRACK_KPS133
                xvals.at(cidx).push_back((int) xmin);
                xvals.at(cidx).push_back((int) xmax);
                yvals.at(cidx).push_back((int) ymin);
                yvals.at(cidx).push_back((int) ymax);
            #endif
        }
    }
    
    if (xvals.at(0).size() == 0 && xvals_rh.at(0).size() == 0)
    {
        return false;
    }
    #ifdef TRACK_COLOR
        for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
            cv::Rect bbox2d_rh;
            bbox2d_rh.x = *std::min_element(xvals_rh.at(cidx).begin(), xvals_rh.at(cidx).end());
            bbox2d_rh.y = *std::min_element(yvals_rh.at(cidx).begin(), yvals_rh.at(cidx).end());
            bbox2d_rh.width = *std::max_element(xvals_rh.at(cidx).begin(), xvals_rh.at(cidx).end())-bbox2d_rh.x;
            bbox2d_rh.height = *std::max_element(yvals_rh.at(cidx).begin(), yvals_rh.at(cidx).end())-bbox2d_rh.y;
            if (bbox2d_rh.width < 10 && bbox2d_rh.height < 10) // filter out to small proposals
            {
                bbox2d_rh.x = 0;
                bbox2d_rh.y = 0;
                bbox2d_rh.width = 10;
                bbox2d_rh.height = 10;
            }

            backcrop_out.rhand.back_crops.at(cidx) = backcrop_in.frame.at(cidx)(bbox2d_rh).clone();
            backcrop_out.rhand.bboxes.at(cidx) = bbox2d_rh;
            backcrop_out.rhand.timestamp = backcrop_in.timestamp;
        }
    #endif

    #ifdef TRACK_KPS133

    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
        cv::Rect bbox2d;
        bbox2d.x = *std::min_element(xvals.at(cidx).begin(), xvals.at(cidx).end());
        bbox2d.y = *std::min_element(yvals.at(cidx).begin(), yvals.at(cidx).end());
        bbox2d.width = *std::max_element(xvals.at(cidx).begin(), xvals.at(cidx).end())-bbox2d.x;
        bbox2d.height = *std::max_element(yvals.at(cidx).begin(), yvals.at(cidx).end())-bbox2d.y;
        if (bbox2d.width < 10 && bbox2d.height < 10) // filter out to small proposals
        {
            bbox2d.x = 0;
            bbox2d.y = 0;
            bbox2d.width = 10;
            bbox2d.height = 10;
        }

        backcrop_out.body.back_crops.at(cidx) = backcrop_in.frame.at(cidx)(bbox2d).clone();
        backcrop_out.body.bboxes.at(cidx) = bbox2d;
        backcrop_out.body.timestamp = backcrop_in.timestamp;
    }
    #endif

    #if defined(USE_DEBUG_TIME_LOGGING)
        this->t2 = std::chrono::steady_clock::now();
        this->duration = std::chrono::duration_cast<std::chrono::milliseconds>(this->t2 - this->t1);
        this->n_iterations++;
        this->total_dt += this->duration;
    #endif
    return true;
}

void BackCrop::Terminate(void){
    this->ShouldClose=true;
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
}

} // namespace stages

} // namespace rt3d_tracking