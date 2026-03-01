#ifndef PATH_MOTION_PLANNING_MOTION_CONTROLLER_H
#define PATH_MOTION_PLANNING_MOTION_CONTROLLER_H

#include <condition_variable>
#include <vector>
#include <Eigen/Eigen>
#include <thread>
#include "path_motion_planning/motion/path_tracker.h"

namespace path_motion_planning{
namespace motion{

class MutexData{
private:
    std::vector<Eigen::Vector3d> data_;
    std::condition_variable cv_;
    mutable std::mutex mutex_;
    bool is_working_{false};

public:
    void setData(const std::vector<Eigen::Vector3d>& path){
        std::unique_lock<std::mutex> lock(mutex_);  
        cv_.wait(lock, [this]{ return !is_working_; });
        data_ = path;
    }

    std::vector<Eigen::Vector3d> getData(){
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }

    void lock(bool state){
        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_working_ = state;
        }
        cv_.notify_all();
    }
}; //class MutexData

class Controller: public Tools{
private:
    MutexData data_path_;
    PathTracker::Ptr tracker_;
    Eigen::Vector3d robot_p_;
    double robot_yaw_;
    Eigen::Vector2d robot_spd_{Eigen::Vector2d::Zero()}; //参考输出的速度 
    Eigen::Vector2d real_spd_; //真实速度

    struct GroupGlobal{
        Eigen::Vector3d target_pose; //启动时的目标朝向
        Eigen::Vector3d target_pos; //导航目标
        bool is_new_data{false};
    }global_;

    // struct MotionConfig{
    //     int control_interval_ms; //控制时间间隔
    //     int traj_ahead_num;
    // }m_cfg_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Controller> Ptr;

    Controller(){ tracker_ = std::make_shared<PathTracker>(); }

    /**
     * @brief 更新机器人的坐标，朝向，速度
     */
    void updateOdom(const Eigen::Vector3d& pos, const Eigen::Matrix3d& mat, const Eigen::Vector2d& spd);

    /**
     * @brief 更新局部路径
     */
    void updatePathL(const std::vector<Eigen::Vector3d>& path);

    /**
     * @brief 更新全局路径，更新后应将yaw旋转至全局路径的大致朝向
     * @param path 全局搜索的路径
     * @param traj_ahead_num 向前初始化朝向的点的索引
     */
    void updatePathG(const std::vector<Eigen::Vector3d>& path, int traj_ahead_num);

    /**
     * @brief 获取控制命令
     * @return Eigen::Vector3d(x, y, r)
     */
    Eigen::Vector3d getCommand();

    /**
     * @brief 获取控制命令
     * @param spd 引用返回map下的速度 Eigen::Vector2d(x, y)
     * @return Eigen::Vector3d(x, y, r)
     */
    Eigen::Vector3d getCommand(Eigen::Vector2d& spd);

    PathTracker::PathTrackerConfig& trackerConfig(){
        return tracker_->pt_cfg_;
    }

    /**
     * @brief 设置路径跟踪时的速度档位
     * @param rate 速度倍率，输出的速度将乘于该值
     */
    void setSpeedRate(double rate){
        tracker_->setSpeedRate(rate);
    }

}; //class Controller
} //namespace motion
} // namespace path_motion_planning

#endif