#include "ConvexHull.h"

using namespace std;
using namespace Eigen;

namespace rt3d_tracking{

static float GetSine(VectorXf P0, VectorXf P1, VectorXf P2)
{
    // the definition of cross product
    // a x b = ||a||*||b|| * sin(theta) * n
    // so the cross product indicates the sin of the angle

    // if the result is less than zero, then the vector P0P2 is at the counterclockwise direction of P1P2, or vice versa

    return (P1(0) - P0(0)) * (P2(1) - P0(1)) - (P2(0) - P0(0)) * (P1(1) - P0(1));
}

ConvexhullCalculator::ConvexhullCalculator()
{

}

ConvexhullCalculator::~ConvexhullCalculator()
{

}

bool ConvexhullCalculator::CompareFunction(VectorXf P1, VectorXf P2)
{
    float SinTheta = GetSine(this->P_First, P1, P2);

    if (fabsf(SinTheta) < 0.0001)
    {
        return (P1 - this->P_First).norm() < (P2 - this->P_First).norm();
    }
    else
    {
        return SinTheta > 0;
    }
}

vector<VectorXf> ConvexhullCalculator::GrahamScan(vector<VectorXf> Points)
{
    vector<VectorXf> POut;
    vector<VectorXf> P = Points;

    uint16_t index_min = 0;

    // find the point with min y
    for (uint16_t i = 1; i < P.size(); i++)
    {
        if (P[index_min](1) > P[i](1))
        {
            index_min = i;
        }
    }

    this->P_First = P[index_min];
    auto temp = P[0];
    P[0] = P[index_min];
    P[index_min] = temp;

    // sort points by angles, if the angles are the same, then the distance
    sort(P.begin() + 1, P.end(),  [this](VectorXf a, VectorXf b){return this->CompareFunction(a, b);});

    POut.push_back(P[0]);
    POut.push_back(P[1]);
    POut.push_back(P[2]);

    // scan
    int top = 2;
    for (uint16_t i = 3; i < P.size(); i++)
    {
        while (top > 0 && GetSine(POut[top - 1], P[i], POut[top]) >= 0)
        {
            top--;
            POut.pop_back();
        }
        POut.push_back(P[i]);
        top++;
    }

    // output
    return POut;
}

} // namespace rt3d_tracking