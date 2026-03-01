#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "altitude_map/global_map.h"

namespace altitude_map{
class GlobalNode: public rclcpp::Node{
private:
    GlobalMap::Ptr map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_result_;
    rclcpp::TimerBase::SharedPtr timer_vis_result_;

    void visResult(){
        if(pub_cloud_result_->get_subscription_count() < 1) return;
        auto cloud_result = map_->getMapTSDF(true);

        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::toROSMsg(*cloud_result, msg_cloud);
        msg_cloud.header.frame_id = "map";
        msg_cloud.header.stamp = this->now();
        pub_cloud_result_->publish(msg_cloud);    
    }

public:
    GlobalNode(): Node("altitude_map_global_node", rclcpp::NodeOptions()){
        map_ = std::make_shared<GlobalMap>();
        
        Eigen::Vector3d bound_min{-30, -30, -1}, bound_max{30, 30, 1.0};
        double resolution{0.05};
         if (bound_max.x() <= bound_min.x() ||
    bound_max.y() <= bound_min.y() ||
    bound_max.z() <=bound_min.z()) {
    std::cerr << "Error: max boundary must be greater than min boundary!" << std::endl;
    return;  // 或者抛出异常或终止程序
}
        map_->init(bound_min, bound_max, resolution, "/home/qwelyflyf/Documents/point_cloud/flower_2.pcd");

        pub_cloud_result_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/altitude_map_test", 10);
        timer_vis_result_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&GlobalNode::visResult, this));
    }
};
} //namespace altitude_map

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altitude_map::GlobalNode>();
    rclcpp::spin(node);
    return 0;
}