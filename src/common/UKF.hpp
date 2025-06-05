#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <iostream>
#include <vector>

namespace rt3d_tracking{

class UKF
{
public:
    UKF(uint8_t n);
    ~UKF();

    Eigen::VectorXf state;              // [x, y, z, x_dot, y_dot, z_dot].transpose
    Eigen::MatrixXf cov;

    // assumption: constant velocity
    Eigen::VectorXf model_function(Eigen::VectorXf input, float dt);

    void predict(Eigen::VectorXf dp, float dt);
    void predict(float dt);
    void update(Eigen::VectorXf Measurement);
    void update(Eigen::VectorXf Measurement, int16_t seenby);
    
private:
    // params of the filter
    Eigen::MatrixXf Q;                  // process noise covariance matrix, v*vT , v ~ N(0, sigma^2)

    float Ax;                           // process noise for augmented acceleration
    float Ay;
    float Az;

    float Px;                           // measurement noise
    float Py;
    float Pz;

    const float max_cams = 5.;

    // params of the unscented transform
    uint8_t n, m;                       // m = n + 1, there would be m sigma points
    uint8_t n_aug;
    float lambda;
    Eigen::VectorXf w;                  // weight of each sigma point
    Eigen::MatrixXf SigmaPoints;        // In these matrices, data are stored as colmn vectors
    Eigen::MatrixXf SigmaPointsPred;
    void Generate_sigma(void);
    // statistics tool function
    static void CalculateMeanCovariance(Eigen::MatrixXf& Data, Eigen::VectorXf& mean, Eigen::MatrixXf& Covariance, Eigen::VectorXf Weights);
};

} // namespace rt3d_tracking