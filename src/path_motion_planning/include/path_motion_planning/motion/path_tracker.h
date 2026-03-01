#ifndef PATH_MOTION_PLANNING_MOTION_PATH_TRACKER_H
#define PATH_MOTION_PLANNING_MOTION_PATH_TRACKER_H

#include <memory>
#include <Eigen/Eigen>
#include <iostream>

namespace path_motion_planning{
namespace motion{
class Tools{
public:
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

    /**
     * @brief 找到path中距离robot_pos最近的点的索引
     * @return 如果path.empty() 返回-1
     */
    int findClosedIndexOfPath(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& robot_pos){
        if(path.empty()) return -1;
        auto min_dis = (path.front() - robot_pos).norm(); 
        int min_i{0};
        for(int i{1}; i < static_cast<int>(path.size()); i++){
            auto dis = (path.at(i) - robot_pos).norm();
            if(dis < min_dis){
                min_dis = dis;
                min_i = i;
            }
        }
        return min_i;
    }
}; //class Tools

class PathTracker: public Tools{
private:
    bool is_init_{false};
    double accel_add_, accel_del_;
    double spd_rate_{1.0};

    void init(){
        if(!is_init_){
            accel_add_ = pt_cfg_.control_interval_s * pt_cfg_.max_accel_add;
            accel_del_ = pt_cfg_.control_interval_s * pt_cfg_.max_accel_del;
            is_init_ = true;
        }
    }

public:
    typedef std::shared_ptr<PathTracker> Ptr;
    
    struct PathTrackerConfig{
        int traj_ahead_num;
        double max_accel_add; //x,y分方向的加速最大加速度
        double max_accel_del; //x,y分方向的减速最大加速度
        double max_speed; //x,y分方向的最大速度
        double speed_diff_rate; //真实速度与输出速度的比值，需实际测量得到
        double speed_yaw_diff_rate; //yaw旋转输出乘于该倍率 
        double control_interval_s; //控制时间间隔 秒 
        double stop_dis; //小于该距离则停止移动
        double stop_yaw; //小于该值，yaw停止转动
    }pt_cfg_;

    PathTracker(){
        pt_cfg_.traj_ahead_num = 20;
        pt_cfg_.max_accel_add = 3.0;
        pt_cfg_.max_accel_del = 12.0;
        pt_cfg_.max_speed = 2.0;
        pt_cfg_.speed_diff_rate = 2.39;
        pt_cfg_.speed_yaw_diff_rate = 0.8;
        pt_cfg_.control_interval_s = 0.01;
        pt_cfg_.stop_dis = 0.3;
        pt_cfg_.stop_yaw = 0.15;
    }

    /**
     * @brief 更新速度,位置和朝向, 输出控制 x, y, r分量
     * @param path 输入的路径，第一个元素即为最靠近 robot_pos 的点
     * @param g_goal 全局终点的坐标
     * @param robot_pos 机器人中心坐标
     * @param robot_speed 机器人中心的速度 
     * @param robot_yaw 机器人的姿态
     * @return Eigen::Vector3d(x, y, r) 输出的控制命令 
     */
    Eigen::Vector3d getRobotCmd(const std::vector<Eigen::Vector3d>& path, 
            const Eigen::Vector3d& g_goal, Eigen::Vector3d& bot, Eigen::Vector2d& spd, double& yaw){
        init();
        Eigen::Vector2d accel; //速度的变化量
        Eigen::Vector3d cmd; //输出的速度命令
        auto path_size = static_cast<int>(path.size());
        // std::cout << "bot: " << bot.x() << " " << bot.y() << " " << bot.z() << "\n";
        // std::cout << "g_goal: " << g_goal.x() << " " << g_goal.y() << " " << g_goal.z() << " " << (bot - g_goal).norm() << "\n";
        if((bot - g_goal).norm() > pt_cfg_.stop_dis && path_size > 1){

            auto closed_i = Tools::findClosedIndexOfPath(path, bot);
            auto traj_index = path_size > pt_cfg_.traj_ahead_num + closed_i? pt_cfg_.traj_ahead_num + closed_i : path_size-1;
            Eigen::Vector3d target_pose = path.at(traj_index) - bot;
            Eigen::Vector2d target_spd = Eigen::Vector2d(target_pose.x(), target_pose.y()).normalized() * pt_cfg_.max_speed;
            // std::cout << "target_spd: " << pt_cfg_.max_speed << " " << target_spd.x() << " " << target_spd.y() << "\n";
            Eigen::Vector2d error_spd = target_spd - spd;

            for(int i{0}; i < 2; i++){
                if(std::abs(error_spd[i]) < accel_add_) accel[i] = error_spd[i]; //速度差异
                else accel[i] = error_spd[i]>0? accel_add_ : -accel_add_;
            }

            auto diff_yaw = Tools::getYaw(target_pose) - yaw;
            Tools::normYaw(diff_yaw);
            cmd.z() = std::abs(diff_yaw) < pt_cfg_.stop_yaw? 0 : -diff_yaw * pt_cfg_.speed_yaw_diff_rate;

            yaw -= diff_yaw * pt_cfg_.control_interval_s;
            Tools::normYaw(yaw);
        }
        else{
            std::cout << "[getRobotCmd] del\n";
            for(int i{0}; i < 2; i++){
                if(std::abs(spd[i]) > accel_del_) 
                        accel[i] = spd[i]>0? -accel_del_ : accel_del_;
                else accel[i] = -spd[i];
            }
            cmd.z() = 0;
        }

        spd += accel;
        Eigen::Vector2d pos_diff = spd * pt_cfg_.control_interval_s;
        bot.x() += pos_diff.x();
        bot.y() += pos_diff.y();

        cmd.x() = (spd.x()*std::cos(yaw) + spd.y()*std::sin(yaw)) * pt_cfg_.speed_diff_rate * spd_rate_;
        cmd.y() = (spd.x()*-std::sin(yaw) + spd.y()*std::cos(yaw)) * pt_cfg_.speed_diff_rate * spd_rate_;

        return cmd;
    }

    /**
     * @brief 在订阅到全局路径后，将yaw旋转至全局路径的大致朝向
     * @param pose 全局路径的大致朝向
     * @param spd 机器人中心的全局速度
     * @param yaw 机器人中心的全局yaw
     * @param cmd Eigen::Vector3d(x, y, r)
     * @return 是否满足朝向要求
     */
    bool getRobotCmdSuitGPose(const Eigen::Vector3d& pose, Eigen::Vector2d& spd,
            double& yaw, Eigen::Vector3d& cmd){
        init();
        std::cout << "pose: " << pose.x() << " " << pose.y() << " " << pose.z() << "\n";
        auto diff_yaw = Tools::getYaw(pose) - yaw;
        std::cout << "[getRobotCmdSuitGPose]: " << Tools::getYaw(pose) << " " << yaw << " " << diff_yaw << "\n";
        Tools::normYaw(diff_yaw);
        std::cout << "diff_yaw: " << diff_yaw << "\n";

        Tools::slowSpeed(spd.x(), accel_del_, spd.x());
        Tools::slowSpeed(spd.y(), accel_del_, spd.y());

        cmd.x() = (spd.x()*std::cos(yaw) + spd.y()*std::sin(yaw)) * pt_cfg_.speed_diff_rate * spd_rate_;
        cmd.y() = (spd.x()*-std::sin(yaw) + spd.y()*std::cos(yaw)) * pt_cfg_.speed_diff_rate * spd_rate_;

        if(std::abs(diff_yaw) < pt_cfg_.stop_yaw){
            cmd.z() = 0;
            return true;
        }
        cmd.z() = -diff_yaw * pt_cfg_.speed_yaw_diff_rate;
        return false;
    }

    /**
     * @brief 设置速度档位 默认1
     */
    void setSpeedRate(double rate){
        spd_rate_ = rate;
    }
}; //class PathTracker

} //namespace motion
} // namespace path_motion_planning

#endif