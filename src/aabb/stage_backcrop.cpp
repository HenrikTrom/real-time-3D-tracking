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
    this->ThreadHandle.reset(new std::thread(&BackCrop::ThreadFunction, this));
}

bool BackCrop::ProcessFunction(
    data::backcrop_in &backcrop_in,
    data::backcrop_out &backcrop_out
){

    std::array<std::vector<int>, 5> xmins, ymins, xmaxs, ymaxs;
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
            
            if (aabb.ClassID == 0){
                // backcrop_out.lhand.back_crops.at(camid) = backcrop_in.frame.at(camid)(bbox2d).clone();
                // backcrop_out.lhand.bboxes.at(camid) = bbox2d;
                xmins.at(cidx).push_back((int) xmin);
                ymins.at(cidx).push_back((int) ymin);
                xmaxs.at(cidx).push_back((int) xmax);
                ymaxs.at(cidx).push_back((int) ymax);
            }
            else if (aabb.ClassID == 1){
                // backcrop_out.rhand.back_crops.at(camid) = backcrop_in.frame.at(camid)(bbox2d).clone();
                // backcrop_out.rhand.bboxes.at(camid) = bbox2d;
                xmins.at(cidx).push_back((int) xmin);
                ymins.at(cidx).push_back((int) ymin);
                xmaxs.at(cidx).push_back((int) xmax);
                ymaxs.at(cidx).push_back((int) ymax);
            }
            else if (aabb.ClassID == 2){
                // backcrop_out.face.back_crops.at(camid) = backcrop_in.frame.at(camid)(bbox2d).clone();
                // backcrop_out.face.bboxes.at(camid) = bbox2d;
                xmins.at(cidx).push_back((int) xmin);
                ymins.at(cidx).push_back((int) ymin);
                xmaxs.at(cidx).push_back((int) xmax);
                ymaxs.at(cidx).push_back((int) ymax);
            }
            else if (aabb.ClassID == 3 && xmins.at(cidx).size() < 3){
                xmins.at(cidx).push_back((int) xmin);
                ymins.at(cidx).push_back((int) ymin);
                xmaxs.at(cidx).push_back((int) xmax);
                ymaxs.at(cidx).push_back((int) ymax);
            }
        }
    }

    if (xmins.at(0).size() == 0)
    {
        return false;
    }

    for (std::size_t cidx = 0; cidx<flirmulticamera::GLOBAL_CONST_NCAMS; cidx++){
        cv::Rect bbox2d;
        bbox2d.x = *std::min_element(xmins.at(cidx).begin(), xmins.at(cidx).end());
        bbox2d.y = *std::min_element(ymins.at(cidx).begin(), ymins.at(cidx).end());
        bbox2d.width = *std::max_element(xmaxs.at(cidx).begin(), xmaxs.at(cidx).end())-bbox2d.x;
        bbox2d.height = *std::max_element(ymaxs.at(cidx).begin(), ymaxs.at(cidx).end())-bbox2d.y;
        backcrop_out.body.back_crops.at(cidx) = backcrop_in.frame.at(cidx)(bbox2d).clone();
        backcrop_out.body.bboxes.at(cidx) = bbox2d;
    }

    return true;
}

void BackCrop::Terminate(void){
    this->ShouldClose=true;
    this->ThreadHandle->join();
}

} // namespace stages

} // namespace rt3d_tracking