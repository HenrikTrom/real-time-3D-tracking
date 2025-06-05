#pragma once

#include "cpp_utils/StageBase.h"
#include "keypoint/dataType.hpp"

namespace rt3d_tracking
{

namespace stages
{

template<uint16_t NKPS>
class Filter : public cpp_utils::StageBase<
    std::array<KeyPoint3D, NKPS>, std::array<KeyPoint3D, NKPS>
>
{
private:
uint64_t frameCounter;
std::vector<KeyPoint3D_Augment> buffer;
const float dt;     // dt for kalman filter prediction (Newton's equation of motions)
bool ProcessFunction(std::array<KeyPoint3D, NKPS> &inputs, std::array<KeyPoint3D, NKPS> &outputs)
{
    // UKF1 filtering
    // make prediction
    for (auto& point : this->buffer)
    {
        point.Predict(this->dt);
    }

    // update
    for (uint16_t j = 0; j < NKPS; j ++)
    {
        // confidence is great enough, other wise it's assigned to (0,0,0). (according to Henrik)
        if (inputs[j].coord.norm() != 0.0)
        {
            this->buffer.at(j).Update(inputs[j]);
        }
        else
        {
            this->buffer.at(j).SeenFor = 0;
        }
    }
    // output
    for (uint16_t j = 0; j < NKPS; j ++)
    {
        // Remove lost points
        if (this->buffer.at(j).NotSeenFor > 5)
        {
            this->buffer.at(j).ukf->state << 0,0,0,0,0,0;
        }
        KeyPoint3D point_tmp{};
        if (this->buffer.at(j).SeenFor > 5){// only use points that are online for more than 5 iterations
            point_tmp.coord = this->buffer.at(j).coord;
        }
        else{
            point_tmp.coord << 0,0,0;               // project points to origin otherwise
        }
        point_tmp.id = this->buffer.at(j).id;
        outputs.at(j) = point_tmp;
    }

    this->frameCounter ++;
    return true;
};
public:
Filter(const double &fps) : dt(static_cast<float>(1./fps))
{
    this->frameCounter = 0;

    // Init buffer
    this->buffer.clear();
    for (uint16_t i = 0; i < NKPS; i++)
    {
        KeyPoint3D_Augment tmp{};
        tmp.id = i;
        tmp.coord = Eigen::Vector3f{0.0, 0.0, 0.0};
        tmp.SeenFor = 0;
        tmp.NotSeenFor = 0;

        std::vector<float> b{};
        b.push_back(1.0);
        
        this->buffer.emplace_back(tmp, b);
    }

    this->ThreadHandle.reset(new std::thread(&Filter::ThreadFunction, this));
};
~Filter(){};
void Terminate(void)
{
    this->ShouldClose = true;
    this->ThreadHandle->join();
};

};

} // namespace stages

}
