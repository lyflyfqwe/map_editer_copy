#include "path_motion_planning/util/vel_estimator.h"

namespace path_motion_planning{
namespace util{

const Eigen::Matrix<double, 3, 6> VelocityEstimator::H_ = []{
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;  // x位置
    H(1, 2) = 1.0;  // y位置
    H(2, 4) = 1.0;  // z位置
    return H;
}();

void VelocityEstimator::predict(double dt){
    // 状态转移矩阵
    Matrix6d F = Matrix6d::Identity();
    for(int i=0; i<6; i+=2) F(i, i+1) = dt;
    
    // 过程噪声矩阵
    Matrix6d Q = Matrix6d::Zero();
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    
    const Eigen::Matrix2d Q_block {
        {dt4/4 * sigma_a_sq_, dt3/2 * sigma_a_sq_},
        {dt3/2 * sigma_a_sq_, dt2 * sigma_a_sq_}
    };
    
    for(int i{0}; i<6; i+=2) Q.block<2,2>(i,i) = Q_block;
    
    // 预测步骤
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void VelocityEstimator::correct(const Eigen::Vector3d& z){
    const Eigen::Vector3d y = z - H_ * x_;
    const Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    const Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();
    
    x_ += K * y;
    P_ = (Matrix6d::Identity() - K * H_) * P_;
}

VelocityEstimator::VelocityEstimator(): sigma_a_sq_(0.5){
    x_ = Vector6d::Zero();
    P_ = Matrix6d::Identity();
    R_ = Eigen::Matrix3d::Identity() * 0.1;
}

void VelocityEstimator::initialize(const Eigen::Vector3d& init_pos, double pos_variance, 
        double velocity_variance, double process_noise, double obs_noise){
    x_.setZero();
    x_.segment<3>(0) = init_pos;

    P_.diagonal() << pos_variance, velocity_variance, 
    pos_variance, velocity_variance,
    pos_variance, velocity_variance;

    sigma_a_sq_ = process_noise;
    R_ = Eigen::Matrix3d::Identity() * obs_noise;
}

void VelocityEstimator::update(double dt, const Eigen::Vector3d& position){
    if(dt <= 0) return;  // 无效时间间隔
    predict(dt);
    correct(position);
}

Eigen::Vector3d VelocityEstimator::getPosition() const{
    return x_.segment<3>(0);
}

Eigen::Vector3d VelocityEstimator::getVelocity() const{
    return {x_(1), x_(3), x_(5)};
}

Eigen::Matrix<double, 6, 6> VelocityEstimator::getCovariance() const{
    return P_;
}

} //namespace util
} //namespace path_motion_planning