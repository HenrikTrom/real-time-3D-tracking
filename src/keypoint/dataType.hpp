#pragma once

#include "opencv2/core.hpp"
#include "Eigen/Dense"

#include "common/UKF.hpp"
#include "common/MAF.hpp"

namespace rt3d_tracking
{

class KeyPoint2D
{
// private:

public:
    uint8_t camID;
    uint8_t id = 0;
    Eigen::Vector2f coord = Eigen::Vector2f(0.0, 0.0);
    float confidence;
    KeyPoint2D(){};
    KeyPoint2D(uint8_t camID, uint8_t cid, float x, float y, float conf);
    ~KeyPoint2D();
};

class KeyPoint3D
{
public:
    uint8_t id = 0;
    Eigen::Vector3f coord = Eigen::Vector3f(0.0, 0.0, 0.0);
    uint16_t SeenBy = 0;
    KeyPoint3D(){};
    KeyPoint3D(uint8_t id, float x, float y, float z, int16_t seenby);
    ~KeyPoint3D();
};

class KeyPoint3D_Augment : public KeyPoint3D
{
private:
    // std::shared_ptr<FIR_filter> FIR;
public:
    std::shared_ptr<MAF> maf;
    std::shared_ptr<UKF> ukf;
    uint16_t NotSeenFor;
    uint64_t SeenFor;
    float conf_maf;
    
    KeyPoint3D_Augment(void);
    ~KeyPoint3D_Augment();

    void Predict(float dt);
    void Update(KeyPoint3D &pointIn);
};

} //namespace rt3d_tracking