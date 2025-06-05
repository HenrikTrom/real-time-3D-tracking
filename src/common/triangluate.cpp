#include "common/triangulate.hpp"


namespace rt3d_tracking
{

void Calc3DCentreDynamic(
    std::vector<cv::Point2f> points,
    std::vector<uint8_t> CamIDs,
    flir_icp_calib::MultiCameras Cameras,
    Eigen::VectorXf& Centre3D
){
    uint8_t numOfPoints = (uint8_t)points.size();

    // Form H * Centre3D = 0

    Eigen::MatrixXf H(numOfPoints * 2, 4);

    for (Eigen::Index i = 0; i < numOfPoints; i++)
    {
        H(2 * i, 0) = Cameras.Cam[CamIDs.at(i)].P(0, 0) - Cameras.Cam[CamIDs.at(i)].P(2, 0) * points[i].x;
        H(2 * i, 1) = Cameras.Cam[CamIDs.at(i)].P(0, 1) - Cameras.Cam[CamIDs.at(i)].P(2, 1) * points[i].x;
        H(2 * i, 2) = Cameras.Cam[CamIDs.at(i)].P(0, 2) - Cameras.Cam[CamIDs.at(i)].P(2, 2) * points[i].x;
        H(2 * i, 3) = Cameras.Cam[CamIDs.at(i)].P(0, 3) - Cameras.Cam[CamIDs.at(i)].P(2, 3) * points[i].x;

        H(2 * i + 1, 0) = Cameras.Cam[CamIDs.at(i)].P(1, 0) - Cameras.Cam[CamIDs.at(i)].P(2, 0) * points[i].y;
        H(2 * i + 1, 1) = Cameras.Cam[CamIDs.at(i)].P(1, 1) - Cameras.Cam[CamIDs.at(i)].P(2, 1) * points[i].y;
        H(2 * i + 1, 2) = Cameras.Cam[CamIDs.at(i)].P(1, 2) - Cameras.Cam[CamIDs.at(i)].P(2, 2) * points[i].y;
        H(2 * i + 1, 3) = Cameras.Cam[CamIDs.at(i)].P(1, 3) - Cameras.Cam[CamIDs.at(i)].P(2, 3) * points[i].y;
    }

    // Calculate Centre3D using SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinV);
    auto V = svd.matrixV();
    Centre3D = V.col(V.cols() - 1);
    Centre3D /= Centre3D(3);
}

} // namespace rt3d_tracking