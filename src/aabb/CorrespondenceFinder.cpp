#include "aabb/CorrespondenceFinder.hpp"

using namespace std;
using namespace Eigen;

namespace rt3d_tracking{

static float PointToLineDistance(
    VectorXf &Point, VectorXf &Line
){
    float Dis = (Line(0) * Point(0) + Line(1) * Point(1) + Line(2)) / 
                std::sqrt((Line(0) * Line(0) + Line(1) * Line(1)));
    return fabsf(Dis);
}

static bool IsCorresponded2Cam(
    data::BoundingBox2D &BB0, 
    data::BoundingBox2D &BB1, 
    const flir_icp_calib::MultiCameras& Cams, 
    const float &DistanceThreshold
){
    VectorXf ELine01 = Cams.F[BB0.CamID][BB1.CamID] * BB0.Centre;
    if (ELine01(0) != 0)
    {
        ELine01 /= ELine01(0);
    }
    VectorXf ELine10 = Cams.F[BB1.CamID][BB0.CamID] * BB1.Centre;
    if (ELine10(0) != 0)
    {
        ELine10 /= ELine10(0);
    }
    if (PointToLineDistance(BB1.Centre, ELine01) < DistanceThreshold &&
        PointToLineDistance(BB0.Centre, ELine10) < DistanceThreshold)
    {
        return true;
    }
    return false;
}

static void Calc3DCentre(
    vector<data::BoundingBox2D> &BoundingBoxes2D, 
    const flir_icp_calib::MultiCameras &Cameras, 
    VectorXf& Centre3D)
{
    uint8_t numOfPoints = (uint8_t)BoundingBoxes2D.size();
    // Form H * Centre3D = 0
    MatrixXf H(numOfPoints * 2, 4);

    for (Index i = 0; i < numOfPoints; i++)
    {
        H(2 * i, 0) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(0, 0) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 0) * BoundingBoxes2D[i].Centre(0);
        H(2 * i, 1) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(0, 1) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 1) * BoundingBoxes2D[i].Centre(0);
        H(2 * i, 2) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(0, 2) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 2) * BoundingBoxes2D[i].Centre(0);
        H(2 * i, 3) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(0, 3) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 3) * BoundingBoxes2D[i].Centre(0);

        H(2 * i + 1, 0) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(1, 0) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 0) * BoundingBoxes2D[i].Centre(1);
        H(2 * i + 1, 1) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(1, 1) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 1) * BoundingBoxes2D[i].Centre(1);
        H(2 * i + 1, 2) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(1, 2) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 2) * BoundingBoxes2D[i].Centre(1);
        H(2 * i + 1, 3) = Cameras.Cam[BoundingBoxes2D[i].CamID].P(1, 3) - Cameras.Cam[BoundingBoxes2D[i].CamID].P(2, 3) * BoundingBoxes2D[i].Centre(1);
    }

    // Calculate Centre3D using SVD
    JacobiSVD<MatrixXf> svd(H, ComputeThinV);
    auto V = svd.matrixV();
    Centre3D = V.col(V.cols() - 1);
    Centre3D /= Centre3D(3);
}

static bool IsCorrespondedNCam(
    data::BoundingBox2D &BB0, std::vector<data::BoundingBox2D> &BBs, 
    const flir_icp_calib::MultiCameras& Cams, 
    const float &DistanceThreshold
){
    Eigen::VectorXf Centre3D{4};
    Calc3DCentre(BBs, Cams, Centre3D);

    Eigen::VectorXf Projection{3};

    Projection = Cams.Cam[BB0.CamID].P * Centre3D;
    Projection /= Projection(2);

    float distance = (Projection - BB0.Centre).norm();
    if (distance * 2 < DistanceThreshold)
    {
        return true;
    }
    return false;
}

template<typename T>
void EraseByValue(vector<T>& Vec, T& Val)
{
    Vec.erase(remove(Vec.begin(), Vec.end(), Val), Vec.end());
}

vector<vector<data::BoundingBox2D>> CorrespondenceFinder::Find(
    std::array<std::vector<data::BoundingBox2D>, flirmulticamera::GLOBAL_CONST_NCAMS> &BBsAllCams, 
    const flir_icp_calib::MultiCameras &Cams, 
    const uint8_t &ObjectAtLeastSeenBy, const float &DistanceThreshold
)
{
    // TODO: remove buffer?
    // buffer the input data
    auto BBsAll_Buffer = BBsAllCams;
    std::array<uint8_t, flirmulticamera::GLOBAL_CONST_NCAMS> tmp_pool;
    for (std::size_t i = 0; i < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); i++)
    {
        tmp_pool.at(i) = i;
    }
    // sorting the buffers according to how many BBs they have
    for (std::size_t i = 0; i < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); i++)
    {
        for (std::size_t j = 0; j < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); j++)
        {
            if (BBsAll_Buffer[i].size() > BBsAll_Buffer[j].size())
            {
                auto tmp = BBsAll_Buffer[i];
                BBsAll_Buffer[i] = BBsAll_Buffer[j];
                BBsAll_Buffer[j] = tmp;

                auto indextmp = tmp_pool[i];
                tmp_pool[i] = tmp_pool[j];
                tmp_pool[j] = indextmp;
            }
        }
    }
    // re-group camids
    std::vector<uint8_t> index_pool{};
    for (uint8_t i = 0; i < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); i++)
    {
        for (uint8_t j = 0; j < static_cast<uint8_t>(flirmulticamera::GLOBAL_CONST_NCAMS); j ++)
        {
            if (tmp_pool[j] == i)
            {
                index_pool.push_back(j);
            }
        }
    }

    // Matching. Here I iterate through all the components in BBsAll_Buffer[BB_Sizes[0]]
    std::vector<std::vector<data::BoundingBox2D>> CorrespondencesAllCam{};
    for (auto& Current_BB : BBsAll_Buffer[0])
    {
        // Construct a tree
        std::vector<std::vector<data::BoundingBox2D>> Trees{
            std::vector<data::BoundingBox2D>{Current_BB}};

        for (uint8_t CamIndex = 1; CamIndex < BBsAll_Buffer.size(); CamIndex ++)
        {
            for (auto& BB_inOtherCam : BBsAll_Buffer[CamIndex])
            {
                auto TreeBuffer = Trees;
                for (auto& tree : TreeBuffer)
                {
                    auto tree_tmp = tree;
                    if ((tree_tmp.size() < 2) && (IsCorresponded2Cam(BB_inOtherCam, tree_tmp[0], Cams, DistanceThreshold)))
                    {
                        tree_tmp.push_back(BB_inOtherCam);
                        Trees.push_back(tree_tmp);
                    }
                    else if (IsCorrespondedNCam(BB_inOtherCam, tree_tmp, Cams, DistanceThreshold))
                    {
                        tree_tmp.push_back(BB_inOtherCam);
                        Trees.push_back(tree_tmp);
                    }
                }
            }
        }
        
        uint16_t MaxID = 0;
        for (uint16_t i = 1; i < Trees.size(); i++)
        {
            if (Trees[MaxID].size() < Trees[i].size())
            {
                MaxID = i;
            }
        }
        if (Trees[MaxID].size() >= ObjectAtLeastSeenBy)
        {
            for (auto& obj : Trees[MaxID])
            {
                if (obj.CamID != Cams.TopCamID && obj.CamID != tmp_pool[0])
                {
                    EraseByValue<data::BoundingBox2D>(BBsAll_Buffer[index_pool[obj.CamID]], obj);
                }
            }
            CorrespondencesAllCam.push_back(Trees[MaxID]);
        }
    }
    return CorrespondencesAllCam;
}

} // namespace rt3d_tracking