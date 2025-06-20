#include "common/UKF.hpp"

using namespace Eigen;
using namespace std;

namespace rt3d_tracking{
    
    UKF::UKF(uint8_t n)
    {
        // initialisation of the instance
        this->state = VectorXf{ n };
        this->cov = MatrixXf{ n, n };
        
        this->n = n;
        this->n_aug = n + 3;
        this->m = 2 * this->n_aug + 1;
        
        // Weights of sigma points
        this->lambda = 3 - static_cast<float>(this->n_aug);
        
        this->w = VectorXf{ this->m };
        float t = lambda + static_cast<float>(this->n_aug);
        this->w(0) = lambda / t;
        this->w.tail(this->m - 1).fill(0.5 / t);
        
        this->SigmaPoints = MatrixXf{ this->n_aug, this->m };
        this->SigmaPointsPred = MatrixXf{ this->n, this->m };
        
        // initialisation of the state
        this->state.fill(0);
        
        this->Ax = 5;
        this->Ay = 5;
        this->Az = 5;
        
        this->Px = 0.0001;         // measurement noise for x axis
        this->Py = 0.0001;         // measurement noise for y axis
        this->Pz = 0.0001;         // measurement noise for y axis
        
        this->cov <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
        this->Q = MatrixXf{ n, n };
    // x, y, z, vx, vy, vz
    this->Q <<
    0.01,  0,     0,     0,     0,     0,
    0,     0.01,  0,     0,     0,     0,
    0,     0,     0.01,  0,     0,     0,
    0,     0,     0,     1,     0,     0,
    0,     0,     0,     0,     1,     0,
    0,     0,     0,     0,     0,     1;
}

UKF::~UKF()
{
    
}

VectorXf UKF::model_function(VectorXf input, float dt)
{
    // implementation of the model tracking [x,y,z,x_dot,y_dot,z_dot,x_dot_dot_dot,y_dot_dot,z_dot_dot]T
    VectorXf output{ this->n };
    
    output(0) = input(0) + input(3) * dt + 0.5 * input(6) * dt * dt; // x = x0 + vx*t+0.5*ax*t**2
    output(1) = input(1) + input(4) * dt + 0.5 * input(7) * dt * dt; // y = y0 + vy*t+0.5*ay*t**2
    output(2) = input(2) + input(5) * dt + 0.5 * input(8) * dt * dt; // z = z0 + vz*t+0.5*az*t**2
    output(3) = input(3) + input(6) * dt;
    output(4) = input(4) + input(7) * dt;
    output(5) = input(5) + input(8) * dt;
    
    return output;
}

void UKF::predict(float dt)
{
    // generate sigma points
    this->Generate_sigma();
    
    // iterate through
    for (uint8_t i = 0; i < this->m; i++)
    {
        this->SigmaPointsPred.col(i) = this->model_function(this->SigmaPoints.col(i), dt);
    }
    // calculate new mean& covariance
    UKF::CalculateMeanCovariance(this->SigmaPointsPred, this->state, this->cov, this->w);
}

void UKF::predict(VectorXf dp, float dt)
{
    // generate sigma points
    this->Generate_sigma();
    
    // iterate through
    for (uint8_t i = 0; i < this->m; i++)
    {
        this->SigmaPointsPred.col(i)(0) = this->SigmaPoints.col(i)(0) + dp(0);
        this->SigmaPointsPred.col(i)(1) = this->SigmaPoints.col(i)(1) + dp(1);
        this->SigmaPointsPred.col(i)(2) = this->SigmaPoints.col(i)(2) + dp(2);
        this->SigmaPointsPred.col(i)(3) = this->SigmaPoints.col(i)(3) + this->SigmaPoints.col(i)(6) * dt;
        this->SigmaPointsPred.col(i)(4) = this->SigmaPoints.col(i)(4) + this->SigmaPoints.col(i)(7) * dt;
        this->SigmaPointsPred.col(i)(5) = this->SigmaPoints.col(i)(5) + this->SigmaPoints.col(i)(8) * dt;
    }
    // calculate new mean& covariance
    UKF::CalculateMeanCovariance(this->SigmaPointsPred, this->state, this->cov, this->w);
}

void UKF::update(VectorXf Measurement)
{
    constexpr int n_z = 3;
    // convert the predicted sigma points into measure space
    MatrixXf Zsig{ n_z, this->m };
    Zsig.row(0) = this->SigmaPointsPred.row(0);     // assign x coord
    Zsig.row(1) = this->SigmaPointsPred.row(1);     // assign y coord
    Zsig.row(2) = this->SigmaPointsPred.row(2);     // assign z coord
    
    VectorXf Zmean{ n_z };
    MatrixXf S{ n_z, n_z };
    MatrixXf R = MatrixXf(n_z, n_z);
    R << this->Px,  0,          0,
    0,         this->Py,   0,
    0,         0,          this->Pz;
    
    UKF::CalculateMeanCovariance(Zsig, Zmean, S, this->w);
    //add measurement noise covariance matrix
    S = S + R;
    
    // calculate kalman gain
    
    //create matrix for cross correlation Tc
    MatrixXf Tc = MatrixXf(this->n, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (uint8_t i = 0; i < this->m; i++)
    {
        // residual
        VectorXf z_diff = Zsig.col(i) - Zmean;
        // state difference
        VectorXf x_diff = this->SigmaPointsPred.col(i) - this->state;
        Tc = Tc + this->w(i) * x_diff * z_diff.transpose();
    }
    
    // Kalman gain K
    MatrixXf K = Tc * S.inverse();
    // residual
    VectorXf z_diff = Measurement - Zmean;
    // update state mean and covariance matrix
    this->state = this->state + K * z_diff;
    this->cov = this->cov - K * S * K.transpose();
    
}


void UKF::update(VectorXf Measurement, int16_t seenby)
{
    constexpr int n_z = 3;
    // convert the predicted sigma points into measure space
    MatrixXf Zsig{ n_z, this->m };
    Zsig.row(0) = this->SigmaPointsPred.row(0);     // assign x coord
    Zsig.row(1) = this->SigmaPointsPred.row(1);     // assign y coord
    Zsig.row(2) = this->SigmaPointsPred.row(2);     // assign z coord

    VectorXf Zmean{ n_z };
    MatrixXf S{ n_z, n_z };
    MatrixXf R = MatrixXf(n_z, n_z);
    R << this->Px,  0,          0,
         0,         this->Py,   0,
         0,         0,          this->Pz;

    UKF::CalculateMeanCovariance(Zsig, Zmean, S, this->w);
        //add measurement noise covariance matrix
    S = S + R * (static_cast<float>(seenby)/((float) flirmulticamera::GLOBAL_CONST_NCAMS));

    // calculate kalman gain

    //create matrix for cross correlation Tc
    MatrixXf Tc = MatrixXf(this->n, n_z);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (uint8_t i = 0; i < this->m; i++)
    {
        // residual
        VectorXf z_diff = Zsig.col(i) - Zmean;
        // state difference
        VectorXf x_diff = this->SigmaPointsPred.col(i) - this->state;
        Tc = Tc + this->w(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K
    MatrixXf K = Tc * S.inverse();
    // residual
    VectorXf z_diff = Measurement - Zmean;
    // update state mean and covariance matrix
    this->state = this->state + K * z_diff;
    this->cov = this->cov - K * S * K.transpose();
}


void UKF::Generate_sigma(void)
{
    VectorXf x_aug = VectorXf(this->n_aug);
    x_aug << this->state, 0, 0, 0;
    
    MatrixXf cov_aug{ this->n_aug, this->n_aug };
    cov_aug.fill(0);
    cov_aug.topLeftCorner(this->n, this->n) = this->cov + this->Q;
    cov_aug(this->n_aug - 3, this->n_aug - 1) = this->Ax;
    cov_aug(this->n_aug - 2, this->n_aug - 2) = this->Ay;
    cov_aug(this->n_aug - 1, this->n_aug - 1) = this->Az;

    MatrixXf sqrt_P = cov_aug.llt().matrixL();
    
    this->SigmaPoints.col(0) = x_aug;
    for (int i = 0; i < this->n_aug; i++)
    {
        // columns 1 -> n_aug_ = x + sqrt((lambda + n_aug_) * P_) 
        this->SigmaPoints.col(i + 1) = x_aug + sqrt(this->lambda + static_cast<float>(this->n_aug)) * sqrt_P.col(i);
        // columns n_aug_+1 -> 2*n_aug_+1 = x - sqrt((lambda + n_aug_) * P_)
        this->SigmaPoints.col(i + 1 + this->n_aug) = x_aug - sqrt(this->lambda + static_cast<float>(this->n_aug)) * sqrt_P.col(i);
    }
}

void UKF::CalculateMeanCovariance(MatrixXf& Data, VectorXf& mean, MatrixXf &Covariance, VectorXf Weights)
{
    // calculate mean
    mean.fill(0);
    
    for (int i = 0; i < Data.cols(); i++)
    {
        mean += Weights(i) * Data.col(i);
    }
    // calculate covariance
    Covariance.fill(0.0);
    for (int i = 0; i < Data.cols(); i++) 
    {
        // state difference
        VectorXf x_diff = Data.col(i) - mean;
        Covariance += Weights(i) * x_diff * x_diff.transpose();
    }
}

} // namespace rt3d_tracking