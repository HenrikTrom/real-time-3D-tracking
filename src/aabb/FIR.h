#pragma once

#include <vector>
#include <deque>
#include <algorithm>
#include <Eigen/Dense>

class FIR_filter
{
private:
    std::deque<std::vector<Eigen::VectorXf>> buffer;
    std::vector<float> b;
    uint16_t num_taps;

public:
    FIR_filter(std::vector<float> b);
    ~FIR_filter();
    std::vector<Eigen::VectorXf> filter(std::vector<Eigen::VectorXf> DataIn);
};