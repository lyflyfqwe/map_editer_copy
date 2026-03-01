#ifndef PATH_MOTION_PLANNING_UTIL_VEL_ESTIMATOR_H
#define PATH_MOTION_PLANNING_UTIL_VEL_ESTIMATOR_H

#include <memory>
#include <Eigen/Eigen>

namespace path_motion_planning{
namespace util{
class VelocityEstimator{
private:
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    Vector6d x_;        // 状态向量 [x, vx, y, vy, z, vz]
    Matrix6d P_;        // 协方差矩阵
    Eigen::Matrix3d R_; // 观测噪声
    double sigma_a_sq_; // 过程噪声参数

    // 固定矩阵
    static const Eigen::Matrix<double, 3, 6> H_; // 观测矩阵 

    void predict(double dt);

    void correct(const Eigen::Vector3d& z);

public:
    typedef std::shared_ptr<VelocityEstimator> Ptr;
    VelocityEstimator();

    /**
     * @param velocity_variance 初始速度方差
     * @param process_noise 过程噪声
     * @param obs_noise 观测噪声
     */
    void initialize(const Eigen::Vector3d& init_pos, double pos_variance = 0.1,
        double velocity_variance = 10.0, double process_noise = 0.5, double obs_noise = 0.1);

    void update(double dt, const Eigen::Vector3d& position);

    // 获取当前状态
    Eigen::Vector3d getPosition() const;

    Eigen::Vector3d getVelocity() const;

    Eigen::Matrix<double, 6, 6> getCovariance() const;


};
} //namespace util
} //namespace path_motion_planning

#endif