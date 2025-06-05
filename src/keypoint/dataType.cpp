#include "keypoint/dataType.hpp"

namespace rt3d_tracking
{

KeyPoint2D::KeyPoint2D(uint8_t camID, uint8_t id, float x, float y, float conf)
{
    this->camID = camID;
    this->id = id;
    this->coord = Eigen::Vector2f{x, y};
    this->confidence = conf;
}

KeyPoint2D::~KeyPoint2D(){}

KeyPoint3D::KeyPoint3D(uint8_t id, float x, float y, float z, int16_t seenby)
{
    this->id = id;
    this->coord = Eigen::Vector3f(x, y, z);
    this->SeenBy = seenby;
}

KeyPoint3D::~KeyPoint3D(){}

KeyPoint3D_Augment::KeyPoint3D_Augment(void)
{
    this->id = 0;
    this->coord = Eigen::Vector3f{0.0, 0.0, 0.0};

    this->ukf.reset(new UKF{6});
    this->ukf->state << this->coord, 0, 0, 0;
    // this->FIR.reset(new FIR_filter{FIR_b});
    this->NotSeenFor = 0;
    this->SeenFor = 1;
}

KeyPoint3D_Augment::KeyPoint3D_Augment(KeyPoint3D point3D, std::vector<float> FIR_b)
{
    this->id = point3D.id;
    this->coord = point3D.coord;

    this->ukf.reset(new UKF{6});
    this->ukf->state << this->coord, 0, 0, 0;
    // this->FIR.reset(new FIR_filter{FIR_b});
    this->NotSeenFor = 0;
    this->SeenFor = 1;
}

KeyPoint3D_Augment::~KeyPoint3D_Augment()
{

}

void KeyPoint3D_Augment::Predict(float dt)
{
    this->ukf->predict(dt);
    this->coord << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2);
    this->NotSeenFor++;
}

void KeyPoint3D_Augment::Update(KeyPoint3D &pointIn)
{
    this->ukf->update(pointIn.coord, pointIn.SeenBy);
    this->coord << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2);
    this->NotSeenFor = 0;
    this->SeenFor ++;
}


} //namespace rt3d_tracking