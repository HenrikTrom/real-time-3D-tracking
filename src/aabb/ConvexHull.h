#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>

namespace rt3d_tracking{

class ConvexhullCalculator
{
private:
    Eigen::VectorXf P_First{3};
    bool CompareFunction(Eigen::VectorXf P1, Eigen::VectorXf P2);
public:
    ConvexhullCalculator();
    ~ConvexhullCalculator();
    std::vector<Eigen::VectorXf> GrahamScan(std::vector<Eigen::VectorXf> Points);
};

} // namespace rt3d_tracking