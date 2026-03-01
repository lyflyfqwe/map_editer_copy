#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_GL_ROS_H
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_GL_ROS_H

#include "path_motion_planning/nav_core/nav_local_base.hpp"
#include "path_motion_planning/nav_core/nav_gl.h"

namespace path_motion_planning{
namespace nav_core{
class NavGLRos: public NavLocalBase, public rclcpp::Node{
private:
    NavGL::Ptr nav_plan_;

    Eigen::Vector3d goal_global_; //全局搜索的目的地
    bool is_new_task_global_{false}; //刚获得全局搜索的目标
    bool is_reach_goal_{true}; //是否到达全局目标
    int result_path_global_{0}; //最近一次全局搜索结果

    void callBackCloudOdom(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom);
    
    inline void loadParamsOfPlanG(NavGL::Ptr& nav_plan); 

    /**
     * @brief 尝试一次全局路径搜索
     */
    inline void tryGlobalNav();

    /**
     * @brief 订阅rviz点坐标，设置终点坐标，供tryGlobalNav计算 path
     */
    void callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);

    void callBackNavServer(const rm_interface::srv::NavCore::Request::SharedPtr& req,
            const rm_interface::srv::NavCore::Response::SharedPtr& res);

    void callBackBotMode(const rm_interface::srv::SetRobotMode::Request::SharedPtr& req,
            const rm_interface::srv::SetRobotMode::Response::SharedPtr& res);

    /**
     * @brief 尝试一次全局路径搜索
     */
    inline void tryLocalNav();

    inline void visCloudGlocal();
    
public:
    NavGLRos();

    ~NavGLRos();
}; //class NavGLRos
} //namespace nav_core
} //namespace path_planning

#endif