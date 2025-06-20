#include "keypoint/stage_triangulate.hpp"

namespace rt3d_tracking
{

namespace stages
{
    
KeyPoint3D calc3Dpoint_worker(
    uint8_t id, std::array<cv::Point2f, flirmulticamera::GLOBAL_CONST_NCAMS> pts2D, 
    flir_icp_calib::MultiCameras cameras
){
    std::vector<uint8_t> cids{};
    std::vector<cv::Point2f> pts2D_processed{};
    for (uint8_t i = 1; i < pts2D.size(); i++)
    {
        if (pts2D[i].x != 0 && pts2D[i].y != 0)
        {
            pts2D_processed.push_back(pts2D[i]);
            cids.push_back(i);
        }
    }
    if (cids.size() >= MIN_CAMS_TRIANGULATION)
    {
        Eigen::VectorXf point3D{};
        Calc3DCentreDynamic(pts2D_processed, cids, cameras, point3D);
        
        KeyPoint3D keypoint3D_out{};
        keypoint3D_out.SeenBy = (int16_t) cids.size();

        keypoint3D_out.coord << point3D(0), point3D(1), point3D(2);
        keypoint3D_out.id = id;
        return keypoint3D_out;
    }
    else
    {
        return KeyPoint3D{id, 0.0, 0.0, 0.0, 0};
    }
}

KeyPoint3D calc3Dpoint_worker_(
    uint8_t id, std::vector<cv::Point2f> pts2D, 
    flir_icp_calib::MultiCameras cameras
){
    std::vector<uint8_t> cids{};
    std::vector<cv::Point2f> pts2D_processed{};
    for (uint8_t i = 0; i < pts2D.size(); i++)
    {
        if (pts2D[i].x != 0 && pts2D[i].y != 0)
        {
            pts2D_processed.push_back(pts2D[i]);
            cids.push_back(i);
        }
    }
    if (cids.size() >= MIN_CAMS_TRIANGULATION)
    {
        Eigen::VectorXf point3D{};
        Calc3DCentreDynamic(pts2D_processed, cids, cameras, point3D);
        
        KeyPoint3D keypoint3D_out{};
        keypoint3D_out.SeenBy = (int16_t) cids.size();

        keypoint3D_out.coord << point3D(0), point3D(1), point3D(2);
        keypoint3D_out.id = id;
        return keypoint3D_out;
    }
    else
    {
        return KeyPoint3D{id, 0.0, 0.0, 0.0, 0};
    }
}

} // namespace stages

}
