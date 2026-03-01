#ifndef PATH_MOTION_PLANNING_MOTION_TRACKER_H
#define PATH_MOTION_PLANNING_MOTION_TRACKER_H

#include <memory>
#include <Eigen/Eigen>

namespace path_motion_planning{
namespace motion{
class Tracker{
private:
    inline double getYaw(const Eigen::Matrix3d& mat){
        return std::atan2(mat(1, 0), mat(0, 0));
    }

    inline double getYaw(const Eigen::Vector3d& v){
        return std::atan2(v.y(), v.x());
    }

    inline void normYaw(double& yaw){
        if(yaw > M_PI) yaw -= 2*M_PI;
        else if(yaw < -M_PI) yaw += 2*M_PI;
    }

    inline void slowSpeed(double speed, double accel, double& cmd){
        if(std::abs(speed) < accel) cmd = 0;
        else cmd = speed>0? speed-accel : speed+accel;
    }

public:
    typedef std::shared_ptr<Tracker> Ptr;

    struct TrackerConfig{
        int traj_ahead_num_;
        double max_accel_add_; //x,y分方向的加速最大加速度
        double max_accel_del_; //x,y分方向的减速最大加速度
        double max_speed_; //x,y分方向的最大速度
        double speed_diff_rate_; //真实速度与输出速度的比值，需实际测量得到
        double speed_yaw_diff_rate; //yaw旋转输出乘于该倍率 
        double control_interval_s_; //控制时间间隔 秒 
        double stop_dis_; //小于该距离则停止移动
        double stop_yaw_; //小于该值，yaw停止转动
    }t_cfg_;
    

    Tracker(){
        t_cfg_.traj_ahead_num_ = 5;
        t_cfg_.max_accel_add_ = 3.0;
        t_cfg_.max_accel_del_ = 12.0;
        t_cfg_.max_speed_ = 2.0;
        t_cfg_.speed_diff_rate_ = 2.39;
        t_cfg_.speed_yaw_diff_rate = 0.8;
        t_cfg_.control_interval_s_ = 0.01;
        t_cfg_.stop_dis_ = 0.3;
        t_cfg_.stop_yaw_ = 0.15;
    }

    /**
     * @brief 更新速度,位置和朝向, 输出控制 x, y, r分量
     * @param path 输入的路径，第一个元素即为最靠近 robot_pos 的点
     * @param taregt_pos 目标的坐标
     * @param robot_pos 机器人中心坐标
     * @param robot_speed 机器人中心的速度 
     * @param robot_yaw 机器人的姿态
     * @return Eigen::Vector3d(x, y, r) 输出的控制命令 
     */
    Eigen::Vector3d updateState(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& taregt_pos, 
            Eigen::Vector3d& robot_pos, Eigen::Vector2d& robot_speed, double& robot_yaw, Eigen::Vector2d& road_spd){
        Eigen::Vector2d accel; //速度的变化量
        Eigen::Vector3d cmd; //输出的速度命令
        if((robot_pos - taregt_pos).norm() > t_cfg_.stop_dis_ && static_cast<int>(path.size()) > t_cfg_.traj_ahead_num_){ //加速直到最大速度
            Eigen::Vector3d pose_target = (path.at(t_cfg_.traj_ahead_num_) - robot_pos).normalized();
            // Eigen::Vector3d pose_target = (path.at(t_cfg_.traj_ahead_num_) - path.front()).normalized();
            Eigen::Vector2d spd_target =  //目标速度即为最大速度
                    Eigen::Vector2d(pose_target.x(), pose_target.y()).normalized() * t_cfg_.max_speed_;

            road_spd = spd_target;

            Eigen::Vector2d spd_error = spd_target - robot_speed;

            for(int i{0}; i < 2; i++){
                if(std::abs(spd_error[i]) < t_cfg_.max_accel_add_) accel[i] = spd_error[i]; //速度差异
                else accel[i] = spd_error[i]>0? t_cfg_.max_accel_add_ : -t_cfg_.max_accel_add_;
            }

            auto yaw_diff = getYaw(pose_target) - robot_yaw;
            normYaw(yaw_diff);
            cmd.z() = std::abs(yaw_diff) < t_cfg_.stop_yaw_? 0 : -yaw_diff*t_cfg_.speed_yaw_diff_rate;
            robot_yaw += (-yaw_diff) * t_cfg_.control_interval_s_;
        }
        else{ //靠近终点减速
            for(int i{0}; i < 2; i++){
                if(std::abs(robot_speed[i]) > t_cfg_.max_accel_del_) 
                        accel[i] = robot_speed[i]>0? -t_cfg_.max_accel_del_ : t_cfg_.max_accel_del_;
                else accel[i] = -robot_speed[i];
            }
            cmd.z() = 0;
        }

        robot_speed += accel;

        Eigen::Vector2d pos_diff = robot_speed * t_cfg_.control_interval_s_;
        robot_pos.x() += pos_diff.x();
        robot_pos.y() += pos_diff.y();

        cmd.x() = (robot_speed.x()*std::cos(robot_yaw) + robot_speed.y()*std::sin(robot_yaw)) * t_cfg_.speed_diff_rate_;
        cmd.y() = (robot_speed.x()*-std::sin(robot_yaw) + robot_speed.y()*std::cos(robot_yaw)) * t_cfg_.speed_diff_rate_;

        // cmd.x() = 6;
        // cmd.y() = 0;

        return cmd;
    }

    /**
     * @brief 在订阅到全局路径后，将yaw旋转至全局路径的大致朝向
     * @param pose_traget 全局路径的大致朝向
     * @param robot_mat 机器人中心的位姿
     * @param command Eigen::Vector3d(x, y, r)
     * @return 是否满足朝向要求
     */
    bool updateStateSuitGlobalPose(const Eigen::Vector3d& taregt_pos, Eigen::Vector2d& robot_speed,
            double& robot_yaw, Eigen::Vector3d& cmd){
        auto yaw_diff = getYaw(taregt_pos) - robot_yaw;
        normYaw(yaw_diff);

        Eigen::Vector2d speed;
        slowSpeed(robot_speed.x(), t_cfg_.max_accel_del_, speed.x());
        slowSpeed(robot_speed.y(), t_cfg_.max_accel_del_, speed.y());

        cmd.x() = (speed.x()*std::cos(robot_yaw) + speed.y()*std::sin(robot_yaw)) * t_cfg_.speed_diff_rate_;
        cmd.y() = (speed.x()*-std::sin(robot_yaw) + speed.y()*std::cos(robot_yaw)) * t_cfg_.speed_diff_rate_;

        if(std::abs(yaw_diff) < t_cfg_.stop_yaw_){
            cmd.z() = 0;
            return true;
        }
        cmd.z() = -yaw_diff*t_cfg_.speed_yaw_diff_rate;
        return false;
    }

}; //class Tracker
} //namespace motion
} // namespace path_motion_planning

#endif