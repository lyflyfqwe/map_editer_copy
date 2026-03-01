#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "altitude_map/cloud_generator.h"

namespace altitude_map{
class GenerateCloud: public rclcpp::Node{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    CloudGenerator generator_;

public:
    GenerateCloud(): Node("altitude_map_cloud_generator_node", rclcpp::NodeOptions()){
        cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/altitude_map_cloud_generator", 10);

        *cloud_ += *generator_.cylinder({0.4, 0, 0}, 0.1, 1.1, 0.02);
        *cloud_ += *generator_.cylinder({-0.4, 0, 0}, 0.1, 1.1, 0.02);
        *cloud_ += *generator_.triangle({-2, 2, -0.1}, {2, 2, -0.1}, {0, -2, 1}, 0.02);
        *cloud_ += *generator_.triangle({-4, 1, 0}, {4, 0.5, 0}, {0, 4, 0}, 0.02);
        *cloud_ += *generator_.triangle({-3, 4, 0.2}, {3, 4.5, 0.2}, {0, 2.8, 0.2}, 0.02);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this](){
            sensor_msgs::msg::PointCloud2 msg_cloud;
            pcl::toROSMsg(*cloud_, msg_cloud);
            msg_cloud.header.frame_id = "map";
            msg_cloud.header.stamp = this->now();
            pub_cloud_->publish(msg_cloud);    
        });
    }
};
} //namespace altitude_map

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altitude_map::GenerateCloud>();
    rclcpp::spin(node);
    return 0;
}