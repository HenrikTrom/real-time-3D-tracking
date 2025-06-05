#include "aabb/BoundingBox.h"

using namespace std;
using namespace Eigen;

namespace rt3d_tracking{

namespace data{

BoundingBox2D::BoundingBox2D(void)
{
    this->Vertex_TopLeft = VectorXf{ 3 };
    this->Vertex_BottomRight = VectorXf{ 3 };
    this->Confidence = 0.0;
    this->ClassID = 0;
    this->Centre = VectorXf{ 3 };
}

BoundingBox2D::BoundingBox2D(float x_min, float y_min, float x_max, float y_max, uint8_t ClassID, float Confidence, uint8_t CamID)
{
    this->Vertex_TopLeft = VectorXf{ 3 };
    this->Vertex_BottomRight = VectorXf{ 3 };
    this->Vertex_TopLeft << x_min, y_min, 1.0;
    this->Vertex_BottomRight << x_max, y_max, 1.0;
    this->Confidence = Confidence;
    this->ClassID = ClassID;
    this->CamID = CamID;
    this->Centre = 0.5 * (Vertex_TopLeft + Vertex_BottomRight);
}

BoundingBox2D::BoundingBox2D(VectorXf Vertex_TopLeft, VectorXf Vertex_BottomRight, uint8_t ClassID, float Confidence, uint8_t CamID)
{
    this->Vertex_TopLeft = Vertex_TopLeft;
    this->Vertex_BottomRight = Vertex_BottomRight;
    this->Confidence = Confidence;
    this->ClassID = ClassID;
    this->CamID = CamID;
    this->Centre = 0.5 * (Vertex_TopLeft + Vertex_BottomRight);
}

BoundingBox2D::~BoundingBox2D()
{

}

void BoundingBox2D::Set_Val(VectorXf Vertex_TopLeft, VectorXf Vertex_BottomRight)
{
    this->Vertex_TopLeft = Vertex_TopLeft;
    this->Vertex_BottomRight = Vertex_BottomRight;
    this->Centre = 0.5 * (Vertex_TopLeft + Vertex_BottomRight);
}

bool BoundingBox2D::operator==(const BoundingBox2D Right)
{
    return (this->CamID == Right.CamID) &&
           (this->ClassID == Right.ClassID) &&
           (this->Confidence == Right.Confidence) &&
           (this->Vertex_TopLeft == Right.Vertex_TopLeft) &&
           (this->Vertex_BottomRight == Right.Vertex_BottomRight);
}

AABB::AABB(void)
{
    this->ClassID = 0;
    this->Vertices = vector<VectorXf>{ 8, VectorXf{4} };
    this->InstanceID = 0;
}

AABB::~AABB()
{

}

VectorXf AABB::GetCentre(void)
{
    VectorXf Centre{3};
    auto temp = 0.5 * (this->Vertices[5] + this->Vertices[3]);
    Centre << temp(0), temp(1), temp(2);

    return Centre;
}

Eigen::VectorXf AABB::GetVertexMin(void)
{
    return this->Vertices[5];
}

Eigen::VectorXf AABB::GetVertexMax(void)
{
    return this->Vertices[3];
}

AABB_Augmented::AABB_Augmented(void)
{
    
}

AABB_Augmented::AABB_Augmented(AABB AABB_base, uint16_t InstanceID, std::vector<float> FIR_b)
{
    this->ClassID = AABB_base.ClassID;
    this->Vertices = AABB_base.Vertices;
    this->SeenByCameras = AABB_base.SeenByCameras;
    this->InstanceID = InstanceID;

    this->ukf.reset(new UKF{6});
    this->ukf->state << this->GetCentre(), 0, 0, 0;
    this->FIR.reset(new FIR_filter{FIR_b});
    this->NotSeenFor = 0;
    this->SeenFor = 1;
    this->previous_centre = this->GetCentre();
}

AABB_Augmented::~AABB_Augmented()
{
}

void AABB_Augmented::Predict(float dt)
{
    this->ukf->predict(dt/8);
    this->NotSeenFor++;
    // update the predicted location
    this->previous_centre = this->GetCentre();
    VectorXf Centre_current{4};
    VectorXf Centre_predicted{4};

    Centre_current = 0.5 * (this->Vertices[1] + this->Vertices[7]);
    Centre_predicted << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2), 1;

    VectorXf shift_vector = Centre_predicted - Centre_current;
    for (uint8_t i = 0; i < 8; i++)
    {
        this->Vertices[i] += shift_vector;
        this->Vertices[i](3) = 1;
    }
}

void AABB_Augmented::Predict(Eigen::VectorXf dp, float dt)
{
    this->ukf->predict(dp, dt/8);
    this->NotSeenFor++;
    // update the predicted location
    this->previous_centre = this->GetCentre();
    VectorXf Centre_current{4};
    VectorXf Centre_predicted{4};

    Centre_current = 0.5 * (this->Vertices[1] + this->Vertices[7]);
    Centre_predicted << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2), 1;

    VectorXf shift_vector = Centre_predicted - Centre_current;
    for (uint8_t i = 0; i < 8; i++)
    {
        this->Vertices[i] += shift_vector;
        this->Vertices[i](3) = 1;
    }
}

void AABB_Augmented::Update(AABB &AABBIn)
{
    this->NotSeenFor = 0;
    this->SeenFor ++;
    this->ukf->update(AABBIn.GetCentre());
    this->Vertices = AABBIn.Vertices;
    // update the predicted location
    VectorXf Centre_current{4};
    VectorXf Centre_predicted{4};

    Centre_current = 0.5 * (this->Vertices[1] + this->Vertices[7]);
    Centre_predicted << this->ukf->state(0), this->ukf->state(1), this->ukf->state(2), 1;

    VectorXf shift_vector = Centre_predicted - Centre_current;
    for (uint8_t i = 0; i < 8; i++)
    {
        this->Vertices[i] += shift_vector;
        this->Vertices[i](3) = 1;
    }
}

std::vector<Eigen::VectorXf> AABB_Augmented::Filter(void)
{
    return this->FIR->filter(this->Vertices);
}

Eigen::VectorXf AABB_Augmented::get_dp(void)
{
    return this->GetCentre() - this->previous_centre;
}

} // namespace data

} // namespace rt3d_tracking