#include "aabb/AABB_calculator.h"

using namespace std;
using namespace Eigen;

namespace rt3d_tracking{

static VectorXf MaxOf(vector<VectorXf> Data)
{
    VectorXf max_val{Data[0].size()};
    max_val << Data[0];

    for (uint16_t n = 0; n < Data.size(); n++)
    {
        for (uint16_t dim = 0; dim < Data[0].size(); dim++)
        {
            if (Data[n](dim) > max_val(dim))
            {
                max_val(dim) = Data[n](dim);
            }
        }
    }
    return max_val;
}

static VectorXf MinOf(vector<VectorXf> Data)
{
    VectorXf min_val{ Data[0].size() };
    min_val << Data[0];

    for (uint16_t n = 0; n < Data.size(); n++)
    {
        for (uint16_t dim = 0; dim < Data[0].size(); dim++)
        {
            if (Data[n](dim) < min_val(dim))
            {
                min_val(dim) = Data[n](dim);
            }
        }
    }
    return min_val;
}

void Calc3DCentre(
    vector<data::BoundingBox2D> BoundingBoxes2D, 
    flir_icp_calib::MultiCameras Cameras, VectorXf& Centre3D
){
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

VectorXf Calc2DProjectionTo3D(VectorXf Point2D, VectorXf Centre2D, VectorXf Centre3D,flir_icp_calib::CameraParameters Camera)
{
    VectorXf Centre3DCam = Camera.M * Centre3D;
    VectorXf Point3D{ 4 };
    float norm = Centre3DCam.norm();
    VectorXf Centre2Dtmp{2};
    Centre2Dtmp << Centre2D(0), Centre2D(1);

    float tmp = std::pow((Camera.Principle - Centre2Dtmp).norm(), 2);

    Point3D(0) = (Point2D(0) - Centre2D(0)) * norm / std::sqrt(tmp + std::pow(Camera.fx, 2));
    Point3D(1) = (Point2D(1) - Centre2D(1)) * norm / std::sqrt(tmp + std::pow(Camera.fy, 2));
    Point3D += Centre3DCam;
    Point3D(2) = Centre3DCam(2);
    Point3D(3) = 1;
    Point3D = Camera.M_inv * Point3D;
    Point3D /= Point3D(3);

    return Point3D;
}

data::AABB CalcAABB(vector<data::BoundingBox2D> BoundingBoxes2D,flir_icp_calib::MultiCameras Cameras)
{
    VectorXf Centre3D;
    Calc3DCentre(BoundingBoxes2D, Cameras, Centre3D);

    vector<VectorXf> Reprojected_Vertices;

    for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
    {
        Reprojected_Vertices.push_back(Calc2DProjectionTo3D(BoundingBoxes2D[i].Vertex_TopLeft, BoundingBoxes2D[i].Centre, Centre3D, Cameras.Cam[BoundingBoxes2D[i].CamID]));
        Reprojected_Vertices.push_back(Calc2DProjectionTo3D(BoundingBoxes2D[i].Vertex_BottomRight, BoundingBoxes2D[i].Centre, Centre3D, Cameras.Cam[BoundingBoxes2D[i].CamID]));
    }

    VectorXf maxVal = MaxOf(Reprojected_Vertices);
    VectorXf minVal = MinOf(Reprojected_Vertices);

    data::AABB aabb{};
    aabb.ClassID = BoundingBoxes2D[0].ClassID;
    aabb.Vertices[0](0) = minVal(0); aabb.Vertices[0](1) = maxVal(1); aabb.Vertices[0](2) = maxVal(2); aabb.Vertices[0](3) = 1;
    aabb.Vertices[1](0) = minVal(0); aabb.Vertices[1](1) = minVal(1); aabb.Vertices[1](2) = maxVal(2); aabb.Vertices[1](3) = 1;
    aabb.Vertices[2](0) = maxVal(0); aabb.Vertices[2](1) = minVal(1); aabb.Vertices[2](2) = maxVal(2); aabb.Vertices[2](3) = 1;
    aabb.Vertices[3](0) = maxVal(0); aabb.Vertices[3](1) = maxVal(1); aabb.Vertices[3](2) = maxVal(2); aabb.Vertices[3](3) = 1;

    aabb.Vertices[4](0) = minVal(0); aabb.Vertices[4](1) = maxVal(1); aabb.Vertices[4](2) = minVal(2); aabb.Vertices[4](3) = 1;
    aabb.Vertices[5](0) = minVal(0); aabb.Vertices[5](1) = minVal(1); aabb.Vertices[5](2) = minVal(2); aabb.Vertices[5](3) = 1;
    aabb.Vertices[6](0) = maxVal(0); aabb.Vertices[6](1) = minVal(1); aabb.Vertices[6](2) = minVal(2); aabb.Vertices[6](3) = 1;
    aabb.Vertices[7](0) = maxVal(0); aabb.Vertices[7](1) = maxVal(1); aabb.Vertices[7](2) = minVal(2); aabb.Vertices[7](3) = 1;

    return aabb;
}

float CalcError(data::AABB& aabb, data::BoundingBox2D BB2D,flir_icp_calib::CameraParameters camera)
{
    vector<VectorXf> projected;
    for (uint8_t i = 0; i < 8; i++)
    {
        projected.push_back(camera.P * aabb.Vertices[i]);
        projected[i] /= projected[i](2);
    }

    VectorXf maxVec = MaxOf(projected);
    VectorXf minVec = MinOf(projected);

    return (minVec - BB2D.Vertex_TopLeft).norm() + (maxVec - BB2D.Vertex_BottomRight).norm();
}

float CalcErrorAll(data::AABB& aabb, vector<data::BoundingBox2D> BB2Ds,flir_icp_calib::MultiCameras Cameras)
{
    float Loss = 0.0;
    for (auto& BB2D : BB2Ds)
    {
        Loss += CalcError(aabb, BB2D, Cameras.Cam[BB2D.CamID]);
    }
    return Loss;
}

void OptimiseAABB(data::AABB& initialAABB, vector<data::BoundingBox2D> BoundingBoxes2D,flir_icp_calib::MultiCameras Cameras)
{
    float L0, L1;
    const float step = 0.002;
    deque<int> Loss_queue;
    while (1)
    {
        // min.x
        data::AABB tempAABB = initialAABB;

        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        tempAABB.Vertices[0](0) += step; tempAABB.Vertices[1](0) += step; tempAABB.Vertices[4](0) += step; tempAABB.Vertices[5](0) += step;
        L1 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L1 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            initialAABB.Vertices[0](0) -= step; initialAABB.Vertices[1](0) -= step; initialAABB.Vertices[4](0) -= step; initialAABB.Vertices[5](0) -= step;
        }
        // min.y
        tempAABB = initialAABB;
        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        tempAABB.Vertices[1](1) += step; tempAABB.Vertices[2](1) += step; tempAABB.Vertices[5](1) += step; tempAABB.Vertices[6](1) += step;
        L1 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L1 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            initialAABB.Vertices[1](1) -= step; initialAABB.Vertices[2](1) -= step; initialAABB.Vertices[5](1) -= step; initialAABB.Vertices[6](1) -= step;
        }
        // min.z
        tempAABB = initialAABB;
        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        tempAABB.Vertices[4](2) += step; tempAABB.Vertices[5](2) += step; tempAABB.Vertices[6](2) += step; tempAABB.Vertices[7](2) += step;
        L1 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L1 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            initialAABB.Vertices[4](2) -= step; initialAABB.Vertices[5](2) -= step; initialAABB.Vertices[6](2) -= step; initialAABB.Vertices[7](2) -= step;
        }
        // max.x
        tempAABB = initialAABB;
        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        tempAABB.Vertices[2](0) += step; tempAABB.Vertices[3](0) += step; tempAABB.Vertices[6](0) += step; tempAABB.Vertices[7](0) += step;
        L1 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L1 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            initialAABB.Vertices[2](0) -= step; initialAABB.Vertices[3](0) -= step; initialAABB.Vertices[6](0) -= step; initialAABB.Vertices[7](0) -= step;
        }
        // max.y
        tempAABB = initialAABB;
        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        tempAABB.Vertices[0](1) += step; tempAABB.Vertices[3](1) += step; tempAABB.Vertices[4](1) += step; tempAABB.Vertices[7](1) += step;
        L1 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L1 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            initialAABB.Vertices[0](1) -= step; initialAABB.Vertices[3](1) -= step; initialAABB.Vertices[4](1) -= step; initialAABB.Vertices[7](1) -= step;
        }
        // max.z
        tempAABB = initialAABB;
        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        tempAABB.Vertices[0](2) += step; tempAABB.Vertices[1](2) += step; tempAABB.Vertices[2](2) += step; tempAABB.Vertices[3](2) += step;
        L1 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L1 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            initialAABB.Vertices[0](2) -= step; initialAABB.Vertices[1](2) -= step; initialAABB.Vertices[2](2) -= step; initialAABB.Vertices[3](2) -= step;
        }
        // final
        L0 = 0;
        for (uint8_t i = 0; i < BoundingBoxes2D.size(); i++)
        {
            L0 += CalcError(tempAABB, BoundingBoxes2D[i], Cameras.Cam[BoundingBoxes2D[i].CamID]);
        }
        // cout << "loss: " << L0 << "\n";
        Loss_queue.push_back(static_cast<int>(L0));
        if (Loss_queue.size() > 20)
        {
            Loss_queue.pop_front();
            auto tempLossQueue = Loss_queue;
            sort(tempLossQueue.begin(), tempLossQueue.end());
            auto out = unique(tempLossQueue.begin(), tempLossQueue.end());
            tempLossQueue.erase(out, tempLossQueue.end());
            if (Loss_queue.size() - tempLossQueue.size() > 1)
            {
                // Converged
                // make sure the AABB is not flipped or upside down
                if (initialAABB.Vertices[5](0) > initialAABB.Vertices[6](0))
                {
                    //invert x
                    float temp = initialAABB.Vertices[5](0);
                    initialAABB.Vertices[0](0) = initialAABB.Vertices[6](0);
                    initialAABB.Vertices[1](0) = initialAABB.Vertices[6](0);
                    initialAABB.Vertices[4](0) = initialAABB.Vertices[6](0);
                    initialAABB.Vertices[5](0) = initialAABB.Vertices[6](0);

                    initialAABB.Vertices[2](0) = temp;
                    initialAABB.Vertices[3](0) = temp;
                    initialAABB.Vertices[6](0) = temp;
                    initialAABB.Vertices[7](0) = temp;

                }
                if (initialAABB.Vertices[5](1) > initialAABB.Vertices[4](1))
                {
                    //invert y
                    float temp = initialAABB.Vertices[5](1);
                    initialAABB.Vertices[1](1) = initialAABB.Vertices[4](1);
                    initialAABB.Vertices[2](1) = initialAABB.Vertices[4](1);
                    initialAABB.Vertices[5](1) = initialAABB.Vertices[4](1);
                    initialAABB.Vertices[6](1) = initialAABB.Vertices[4](1);

                    initialAABB.Vertices[0](1) = temp;
                    initialAABB.Vertices[3](1) = temp;
                    initialAABB.Vertices[4](1) = temp;
                    initialAABB.Vertices[7](1) = temp;
                }
                if (initialAABB.Vertices[5](2) > initialAABB.Vertices[1](2))
                {
                    //invert z
                    float temp = initialAABB.Vertices[5](2);
                    initialAABB.Vertices[4](2) = initialAABB.Vertices[1](2);
                    initialAABB.Vertices[5](2) = initialAABB.Vertices[1](2);
                    initialAABB.Vertices[6](2) = initialAABB.Vertices[1](2);
                    initialAABB.Vertices[7](2) = initialAABB.Vertices[1](2);

                    initialAABB.Vertices[0](2) = temp;
                    initialAABB.Vertices[1](2) = temp;
                    initialAABB.Vertices[2](2) = temp;
                    initialAABB.Vertices[3](2) = temp;
                }
                break;
            }
        }
    }
}

bool IsPointInPolygon(vector<VectorXf> Polygon, VectorXf Point)
{
    double minX = Polygon[0](0);
    double maxX = Polygon[0](0);
    double minY = Polygon[0](1);
    double maxY = Polygon[0](1);
    for (uint16_t i = 1; i < Polygon.size(); i++)
    {
        minX = (Polygon[i](0) < minX) ? Polygon[i](0) : minX;
        maxX = (Polygon[i](0) > maxX) ? Polygon[i](0) : maxX;
        minY = (Polygon[i](1) < minY) ? Polygon[i](1) : minY;
        maxY = (Polygon[i](1) > maxY) ? Polygon[i](1) : maxY;
    }

    if (Point(0) < minX || Point(0) > maxX || Point(1) < minY || Point(1) > maxY)
    {
        return false;
    }

    bool inside = false;
    for (uint16_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
    {
        if ((Polygon[i](1) > Point(1)) != (Polygon[j](1) > Point(1)) &&
            Point(0) < (Polygon[j](0) - Polygon[i](0)) * (Point(1) - Polygon[i](1)) / (Polygon[j](1) - Polygon[i](1)) + Polygon[i](0))
        {
            inside = !inside;
        }
    }

    return inside;
}

bool ConstrainInsideBB(data::AABB& aabb, vector<data::BoundingBox2D> BoundingBoxes2D,flir_icp_calib::MultiCameras Cameras)
{
    // get all the projected points
    for (auto& BB : BoundingBoxes2D)
    {
        if (BB.CamID == 0)
        {
            vector<VectorXf> projected;
            for (uint8_t i = 0; i < 8; i++)
            {
                projected.push_back(Cameras.Cam[BB.CamID].P * aabb.Vertices[i]);
                projected[i] /= projected[i](2);
            }
            // find convex hull
            ConvexhullCalculator Convexhullcalculator{}; 
            auto ConvexHull = Convexhullcalculator.GrahamScan(projected);
            // check if the vertices of the BoundingBox is inside the convex hull
            if(!IsPointInPolygon(ConvexHull, BB.Vertex_TopLeft) || !IsPointInPolygon(ConvexHull, BB.Vertex_BottomRight))
            {
                return false;
            }
        }
    }
    return true;
}

void EnlargeAABB(data::AABB& aabb, float stepx, float stepy, float stepz)
{
    aabb.Vertices[0](0) -= stepx; aabb.Vertices[1](0) -= stepx; aabb.Vertices[4](0) -= stepx; aabb.Vertices[5](0) -= stepx;
    aabb.Vertices[1](1) -= stepy; aabb.Vertices[2](1) -= stepy; aabb.Vertices[5](1) -= stepy; aabb.Vertices[6](1) -= stepy;
    aabb.Vertices[4](2) -= stepz; aabb.Vertices[5](2) -= stepz; aabb.Vertices[6](2) -= stepz; aabb.Vertices[7](2) -= stepz;

    aabb.Vertices[2](0) += stepx; aabb.Vertices[3](0) += stepx; aabb.Vertices[6](0) += stepx; aabb.Vertices[7](0) += stepx;
    aabb.Vertices[0](1) += stepy; aabb.Vertices[3](1) += stepy; aabb.Vertices[4](1) += stepy; aabb.Vertices[7](1) += stepy;
    aabb.Vertices[0](2) += stepz; aabb.Vertices[1](2) += stepz; aabb.Vertices[2](2) += stepz; aabb.Vertices[3](2) += stepz;
}

void ShiftAABB(data::AABB& aabb, float step, uint8_t dim)
{
    for (auto& Vertex : aabb.Vertices)
    {
        Vertex(dim) += step;
    }
}

void OptimiseAABB_prop(data::AABB& initialAABB, vector<data::BoundingBox2D> BoundingBoxes2D, bool UseConvexhull,flir_icp_calib::MultiCameras Cameras)
{
    float L0, L1;
    const float step = 0.002;
    float kx = 1;
    float ky = fabsf((initialAABB.Vertices[0](1) - initialAABB.Vertices[1](1)) / (initialAABB.Vertices[2](0) - initialAABB.Vertices[0](0)));
    float kz = fabsf((initialAABB.Vertices[0](2) - initialAABB.Vertices[4](2)) / (initialAABB.Vertices[2](0) - initialAABB.Vertices[0](0)));
    float stepx = kx * step;
    float stepy = ky * step;
    float stepz = kz * step;

    deque<int> Loss_queue;
    while (1)
    {
        // enlarge or shrink
        data::AABB tempAABB = initialAABB;

        L0 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        EnlargeAABB(tempAABB, -stepx, -stepy, -stepz);
        L1 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            EnlargeAABB(initialAABB, stepx, stepy, stepz);
        }
        // shift x
        tempAABB = initialAABB;
        L0 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        ShiftAABB(tempAABB, step, 0);
        L1 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            ShiftAABB(initialAABB, -step, 0);
        }
        // shift y
        tempAABB = initialAABB;
        L0 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        ShiftAABB(tempAABB, step, 1);
        L1 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            ShiftAABB(initialAABB, -step, 1);
        }
        // shift z
        tempAABB = initialAABB;
        L0 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        ShiftAABB(tempAABB, step, 2);
        L1 = CalcErrorAll(tempAABB, BoundingBoxes2D, Cameras);
        if (L1 <= L0)
        {
            initialAABB = tempAABB;
        }
        else
        {
            ShiftAABB(initialAABB, -step, 2);
        }
        // final
        L0 = CalcErrorAll(initialAABB, BoundingBoxes2D, Cameras);
        // cout << "loss: " << L0 << "\n";
        Loss_queue.push_back(static_cast<int>(L0));
        if (Loss_queue.size() > 20)
        {
            Loss_queue.pop_front();
            auto tempLossQueue = Loss_queue;
            sort(tempLossQueue.begin(), tempLossQueue.end());
            auto out = unique(tempLossQueue.begin(), tempLossQueue.end());
            tempLossQueue.erase(out, tempLossQueue.end());
            if (Loss_queue.size() - tempLossQueue.size() > 1)
            {
                // Converged
                // make sure the data::AABB is not flipped or upside down
                if (initialAABB.Vertices[5](0) > initialAABB.Vertices[6](0))
                {
                    //invert x
                    float temp = initialAABB.Vertices[5](0);
                    initialAABB.Vertices[0](0) = initialAABB.Vertices[6](0);
                    initialAABB.Vertices[1](0) = initialAABB.Vertices[6](0);
                    initialAABB.Vertices[4](0) = initialAABB.Vertices[6](0);
                    initialAABB.Vertices[5](0) = initialAABB.Vertices[6](0);

                    initialAABB.Vertices[2](0) = temp;
                    initialAABB.Vertices[3](0) = temp;
                    initialAABB.Vertices[6](0) = temp;
                    initialAABB.Vertices[7](0) = temp;

                }
                if (initialAABB.Vertices[5](1) > initialAABB.Vertices[4](1))
                {
                    //invert y
                    float temp = initialAABB.Vertices[5](1);
                    initialAABB.Vertices[1](1) = initialAABB.Vertices[4](1);
                    initialAABB.Vertices[2](1) = initialAABB.Vertices[4](1);
                    initialAABB.Vertices[5](1) = initialAABB.Vertices[4](1);
                    initialAABB.Vertices[6](1) = initialAABB.Vertices[4](1);

                    initialAABB.Vertices[0](1) = temp;
                    initialAABB.Vertices[3](1) = temp;
                    initialAABB.Vertices[4](1) = temp;
                    initialAABB.Vertices[7](1) = temp;
                }
                if (initialAABB.Vertices[5](2) > initialAABB.Vertices[1](2))
                {
                    //invert z
                    float temp = initialAABB.Vertices[5](2);
                    initialAABB.Vertices[4](2) = initialAABB.Vertices[1](2);
                    initialAABB.Vertices[5](2) = initialAABB.Vertices[1](2);
                    initialAABB.Vertices[6](2) = initialAABB.Vertices[1](2);
                    initialAABB.Vertices[7](2) = initialAABB.Vertices[1](2);

                    initialAABB.Vertices[0](2) = temp;
                    initialAABB.Vertices[1](2) = temp;
                    initialAABB.Vertices[2](2) = temp;
                    initialAABB.Vertices[3](2) = temp;
                }
                break;
            }
        }
    }
    if (UseConvexhull)
    {
        // make sure the project of the data::AABB is inside the BoundingBoxes
        while (!ConstrainInsideBB(initialAABB, BoundingBoxes2D, Cameras))
        {
            EnlargeAABB(initialAABB, stepx, stepy, 0.0);
        }
    }
}

data::AABB CalcAABBfrom2DBBs(vector<data::BoundingBox2D> BoundingBoxes2D, bool UseConvexhull,flir_icp_calib::MultiCameras Cameras)
{
    data::AABB aabb = CalcAABB(BoundingBoxes2D, Cameras);
    OptimiseAABB(aabb, BoundingBoxes2D, Cameras);
    // OptimiseAABB_prop(aabb, BoundingBoxes2D, UseConvexhull, Cameras);
    aabb.ClassID = BoundingBoxes2D[0].ClassID;
    for (auto& BB : BoundingBoxes2D)
    {
        aabb.SeenByCameras.push_back(BB.CamID);
    }
    return aabb;
}

} // namespace rt3d_tracking