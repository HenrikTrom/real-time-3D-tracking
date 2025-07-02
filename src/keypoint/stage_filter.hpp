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
std::array<KeyPoint3D_Augment, NKPS> buffer;
const float dt;     // dt for kalman filter prediction (Newton's equation of motions)
bool ProcessFunction(std::array<KeyPoint3D, NKPS> &inputs, std::array<KeyPoint3D, NKPS> &outputs)
{
    // for (auto& point : this->buffer)
    // {
    //     point.Predict(this->dt);
    // }
    // update
    for (uint16_t j = 0; j < NKPS; j ++)
    {
        this->buffer.at(j).Update(inputs[j]);
    }
    // output
    for (uint16_t j = 0; j < NKPS; j ++)
    {
        if (this->buffer.at(j).conf_maf >= 0.4){ // ~ points must be visible for half window 
            outputs.at(j).coord = this->buffer.at(j).coord;
        }
        else{
            outputs.at(j).coord << 0,0,0;               // project points to origin otherwise
        }
        outputs.at(j).id = this->buffer.at(j).id;
    }

    this->frameCounter ++;
    return true;
};
public:
Filter(const double &fps) : dt(static_cast<float>(1./fps))
{
    this->frameCounter = 0;

    // Init buffer
    for (uint16_t i = 0; i < NKPS; i++)
    {
        this->buffer.at(i).id = i;
        this->buffer.at(i).coord = Eigen::Vector3f{0.0, 0.0, 0.0};
        this->buffer.at(i).SeenFor = 0;
        this->buffer.at(i).NotSeenFor = 0;
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
