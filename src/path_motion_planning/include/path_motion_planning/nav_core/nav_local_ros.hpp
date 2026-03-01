#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_ROS_HPP
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_ROS_HPP

#include "path_motion_planning/nav_core/nav_local_base.hpp"
#include "path_motion_planning/nav_core/nav_local.h"

namespace path_motion_planning{
namespace nav_core{
class NavLocalRos: public NavLocalBase, public rclcpp::Node{
private:
    NavLocal::Ptr nav_plan_;

    void callBackCloudOdom(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
            const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom);

    inline void tryLocalNav();

    inline void getLocalReferedLine(const Eigen::Vector3d& target_pose);

    /**
     * @brief 订阅rviz点坐标，设置终点坐标，供tryLocalNav计算 line 和 path_l
     */
    void callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);

    void callBackNavServer(const rm_interface::srv::NavCore::Request::SharedPtr& req,
            const rm_interface::srv::NavCore::Response::SharedPtr& res);

public:
    NavLocalRos();

    ~NavLocalRos();

}; //class LocalAssistNavMotionRos
} //namespace nav_core
} //namespace path_planning

#endif