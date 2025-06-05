#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <deque>
#include <memory>

#include "aabb/FIR.h"
#include "common/UKF.hpp"

namespace rt3d_tracking{

namespace data{

class BoundingBox2D
{
public:
    BoundingBox2D(void);
    BoundingBox2D(float x_min, float y_min, float x_max, float y_max, uint8_t ClassID, float Confidence, uint8_t CamID);
    BoundingBox2D(Eigen::VectorXf Vertex_TopLeft, Eigen::VectorXf Vertex_BottomRight, uint8_t ClassID, float Confidence, uint8_t CamID);
    ~BoundingBox2D();
    Eigen::VectorXf Vertex_TopLeft;
    Eigen::VectorXf Vertex_BottomRight;
    Eigen::VectorXf Centre;
    float Confidence;
    uint8_t ClassID;
    uint8_t CamID;
    void Set_Val(Eigen::VectorXf Vertex_TopLeft, Eigen::VectorXf Vertex_BottomRight);
    bool operator==(const BoundingBox2D Right);
};

class AABB
{
public:
    uint8_t ClassID;
    std::vector<Eigen::VectorXf> Vertices;
    std::vector<uint8_t> SeenByCameras;
    uint16_t InstanceID;

    AABB(void);
    ~AABB();
    Eigen::VectorXf GetCentre(void);
    Eigen::VectorXf GetVertexMin(void);
    Eigen::VectorXf GetVertexMax(void);
    // 4    7
    //   0    3
    // 5    6
    //   1    2
};

class AABB_Augmented : public AABB
{
private:
    std::shared_ptr<UKF> ukf;
    std::shared_ptr<FIR_filter> FIR;

    Eigen::VectorXf previous_centre;

public:
    uint16_t NotSeenFor;
    uint64_t SeenFor;
    
    AABB_Augmented(void);
    AABB_Augmented(AABB AABB_base, uint16_t InstanceID = 0, std::vector<float> FIR_b = std::vector<float>{});
    ~AABB_Augmented();

    void Predict(float dt);
    void Predict(Eigen::VectorXf dp, float dt);
    void Update(AABB &AABBIn);
    Eigen::VectorXf get_dp(void);
    std::vector<Eigen::VectorXf> Filter(void);
};

} // namespace data

} // namespace rt3d_tracking