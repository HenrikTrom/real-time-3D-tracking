#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <iostream>
#include <vector>

#include "flirmulticamera/config.h"

namespace rt3d_tracking{

class MAF
{
public:
    MAF(std::size_t window_size) : window_size(window_size){};
    ~MAF(){};

    // measurement-update
    void update(Eigen::Vector3f measurement, Eigen::Vector3f &state, float &conf)
    {
        this->states.push_back(measurement);
        if (this->states.size() > this->window_size)
        {
            this->states.erase(this->states.begin());
        }
        this->update(state, conf);
    };
    // no measurement-update
    void update(Eigen::Vector3f &state, float &conf)
    {
        state << 0, 0, 0;
        conf = 0;
        if (!this->states.empty())
        {
            for (Eigen::Vector3f &s :this->states){
                state +=s;
            }
            state /= (float) this->states.size();
            conf = (float) this->states.size() / (float) this->window_size;
        }
    };
    
private:
    std::vector<Eigen::Vector3f> states;
    Eigen::Vector3f state;
    const std::size_t window_size;
};

} // namespace rt3d_tracking
