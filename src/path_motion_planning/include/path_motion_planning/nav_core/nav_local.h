#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_H
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_H

#include "path_motion_planning/util/vel_estimator.h"
#include "path_motion_planning/path_searcher/theta_star_explored.h"
#include "path_motion_planning/nav_core/nav_core.h"
#include "path_motion_planning/motion/controller.h"
#include "altitude_map/local_map.h"

namespace path_motion_planning{
namespace nav_core{
class NavLocal: public NavCore{
protected:
    altitude_map::LocalMap::Ptr map_;
    util::VelocityEstimator::Ptr vel_estimator_;
    motion::Controller::Ptr controller_;

    Eigen::Vector3d lidar_p_; //雷达中心在全局的里程计
    PosI2 safe_i_;

    PathData<Eigen::Vector4d> path_from_g_; //临时存储的全局路径

    struct RobotState{
        Eigen::Vector3d pos{}; //全局坐标系下，机器人中心坐标
        Eigen::Matrix3d mat; //全局坐标系下，机器人姿态
        Eigen::Vector3i ind; ////局部坐标系下，机器人中心坐标所在的索引
        Eigen::Vector2d spd{}; //全局坐标系下的x,y的速度
        
        struct Controller{
            double default_accel_add; //参数中加速时的最大加速度
            double default_accel_del; //参数中减速时的最大加速度
            double default_spd_max; //参数中最大速度
        }ctl; //用于运行途中切换速度档位
    }robot_;

    /**
     * @brief 在由start,end 连接成的线段中均匀插值，返回点的集合
     */
    std::vector<Eigen::Vector2d> 
    interpolatePoints(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double resolution);

    /**
     * @brief 从参考线段中选择导航终点
     * @param safe_start safe_start_p global·坐标系
     * @param begin, end 路径的索引
     * @param target 终点 global·坐标系
     * @return success
     */
    bool selectTargetFromReferedLine(
        const Eigen::Vector3d& safe_start, const std::vector<Eigen::Vector2d>::iterator begin, 
        const std::vector<Eigen::Vector2d>::iterator end, Eigen::Vector3d& target);

    /**
     * @brief 从全局路径中选择导航终点
     * @param safe_start safe_start_p global·坐标系
     * @param begin, end 路径的索引
     * @param target 终点 global·坐标系
     * @param safe_state 判断是否安全，非安全，意味着需要进入跟随模式
     * @return success
     */
    bool selectTargetFromReferedPath(
        const Eigen::Vector3d& safe_start, const std::vector<Eigen::Vector4d>::iterator begin, 
        const std::vector<Eigen::Vector4d>::iterator end, Eigen::Vector3d& target, bool& safe_state);

    /**
     * @brief 设置路径跟踪时的速度档位
     * @param rate 速度倍率，输出的速度将乘于该值
     */
    void setSpeedRate(double rate){
        controller_->setSpeedRate(rate);
    }

public:
    typedef std::shared_ptr<NavLocal> Ptr;

    struct LocalNavConfig{
        Eigen::Vector3d offset_robot_p; 
        Eigen::Matrix3d offset_robot_m;

        struct ProbMapConfig{
            Eigen::Vector3d map_size_d = Eigen::Vector3d(7, 7, 2);
        }p_cfg;

        struct AltitudeMapConfig{
            double trun_radius{0.4}; // tsdf截断距离（类似于膨胀半径）默认0.4m
            int down_spread_dis{2}; // 障碍物向下传播的距离 默认2格
        }alti_cfg;

        struct SearchConfig{
            double v_cost{30};
            double dead_cost{0.5};
            int num_limit{10000};
            int z_error_thres{5}; //将全局路径作为局部路径搜索的参考时，二者在z轴上的最大误差
        }sec_cfg;

        struct CtlConfig{
            int traj_ahead_num;
            double max_accel_add; //x,y分方向的加速最大加速度
            double max_accel_del; //x,y分方向的减速最大加速度
            double max_speed; //x,y分方向的最大速度
            double speed_diff_rate; //真实速度与输出速度的比值，需实际测量得到
            double speed_yaw_diff_rate; //yaw旋转输出乘于该倍率 
            int control_interval_ms; //控制时间间隔 ms 
            double stop_dis; //小于该距离则停止移动
            double stop_yaw; //小于该值，yaw停止转动
        }ctl_cfg;

    }l_cfg_;

    void updateMap(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud_in, 
        const Eigen::Vector3d &odom_p, const Eigen::Matrix3d& odom_q, double update_t);

    /**
     * @brief 获取局部搜索的参考线，即起点和终点的连线
     * @param p_end 终点
     */
    std::vector<Eigen::Vector2d> getReferLine(const Eigen::Vector3d& p_end);

    /**
     * @brief 缓存用来参考的全局路径
     * @param path_in 输入的全局路径
     */
    inline void bufferGlobalPath(std::vector<Eigen::Vector4d>& path_in){
        path_from_g_.setData(path_in);
    }

    /**
     * @brief 从参考的路径中获取起点和终点
     * @param line 参考的路径 2d
     */
    bool selectPosFromLine(std::vector<Eigen::Vector2d>& line, std::vector<Eigen::Vector3d>& safe_p);

    /**
     * @brief 从参考的路径中获取起点和终点，这里的参考路径是缓存的全局路径
     * @param safe_p 用于可视化的起点和终点 
     * @param safe_state 判断是否安全，非安全，意味着需要进入跟随模式
     */
    virtual bool selectPosFromGPath(std::vector<Eigen::Vector3d>& safe_p, bool& safe_state);

    virtual bool seachPath(std::vector<Eigen::Vector3d>& path);

    /**
     * @brief 设置默认参数，仅用于测试
     */
    void setDefaultParams();

    virtual void init();

    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud(){
        return map_->getMapTSDF(false);
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud(){
    //     return map_->getMapOccupied();
    // }

    std::tuple<Eigen::Vector3d, Eigen::Quaterniond, Eigen::Vector2d> getRobotState(){
        return {robot_.pos, Eigen::Quaterniond(robot_.mat), robot_.spd};
    }

    /**
     * @brief 更新controller的初始朝向
     */
    void updatePathG(const std::vector<Eigen::Vector3d>& path_g){
        controller_->updatePathG(path_g, l_cfg_.ctl_cfg.traj_ahead_num);
    }

    /**
     * @brief 获取控制命令
     * @return Eigen::Vector3d(x, y, r)
     */
    Eigen::Vector3d getCommand(){
        return controller_->getCommand();
    }

    /**
     * @brief 获取控制命令
     * @param speed 引用返回map下的速度 Eigen::Vector2d(x, y)
     * @return Eigen::Vector3d(x, y, r)
     */
    Eigen::Vector3d getCommand(Eigen::Vector2d& speed){
        return controller_->getCommand(speed);
    }

}; //class NavLocal
} //namespace nav_core
} //namespace path_motion_planning

#endif