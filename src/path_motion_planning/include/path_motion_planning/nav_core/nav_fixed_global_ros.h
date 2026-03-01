#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "path_motion_planning/nav_core/nav_fixed_global.h"
#include "path_motion_planning/util/vis_ros.h"

namespace path_motion_planning{
namespace nav_core{
class NavFixedGlobalRos: public rclcpp::Node, public util::VisRos{
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pos_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_click_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pos_click_;

    NavFixedGlobal::Ptr plan_;
    altitude_map::FixedGlobalMap::Ptr save_;
    std::vector<Eigen::Vector3d> ps_safe_;
    std::thread thread_vis_;
    int whatMode = 0;
    std::pair<Eigen::Vector3d,Eigen::Vector3d> cube;
    std::pair<Eigen::Vector3d,Eigen::Vector3d> cube_del;
    std::vector<std::pair<Eigen::Vector3i,Eigen::Vector3i>> cubes;
    std::vector<std::pair<Eigen::Vector3i,Eigen::Vector3i>> cubes_del;
    int whatCubePoint = 0;
    void callBackSwich(const geometry_msgs::msg::PoseStamped& msg);
    void callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);
    inline void rosRelated();

public:
    NavFixedGlobalRos();
    ~NavFixedGlobalRos();

};

}
}