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

KeyPoint3D_Augment::KeyPoint3D_Augment(){}

KeyPoint3D_Augment::~KeyPoint3D_Augment(){}

void KeyPoint3D_Augment::init(const std::size_t &window_size){
    this->id = 0;
    this->coord = Eigen::Vector3f{0.0, 0.0, 0.0};

    this->ukf.reset(new UKF{6});
    this->ukf->state << this->coord, 0, 0, 0;
    this->maf.reset(new MAF{window_size});
    this->NotSeenFor = 0;
    this->SeenFor = 1;
}

void KeyPoint3D_Augment::Predict(float dt)
{
    this->ukf->predict(dt);
    this->coord << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2);
    this->NotSeenFor++;
}

void KeyPoint3D_Augment::Update(KeyPoint3D &pointIn)
{
    
    if (pointIn.coord.norm() != 0.0)
    {
        this->maf->update(pointIn.coord, this->coord, this->conf_maf);
    }
    else
    {
        this->maf->update(this->coord, this->conf_maf);
    }
    // this->ukf->update(pointIn.coord, pointIn.SeenBy);
    // this->coord << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2);
    // this->NotSeenFor = 0;
    
}


} //namespace rt3d_tracking