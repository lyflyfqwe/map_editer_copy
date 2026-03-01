#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_conversions/pcl_conversions.h>

#include "altitude_map/local_map.h"

namespace altitude_map{
class LocalNode: public rclcpp::Node{
private:
    typedef std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>> SynchronizerCloudOdom;
    SynchronizerCloudOdom syn_chronizer_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_cloud_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_icp_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

    LocalMap::Ptr map_;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose_icp_;
    Eigen::Vector3d odom_p_;

    void icpOdomCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr& msg_icp){
        auto pos = Eigen::Vector3d(msg_icp->pose.pose.position.x, msg_icp->pose.pose.position.y, msg_icp->pose.pose.position.z);
        auto mat = Eigen::Quaterniond(msg_icp->pose.pose.orientation.w, msg_icp->pose.pose.orientation.x, 
                msg_icp->pose.pose.orientation.y, msg_icp->pose.pose.orientation.z).matrix();
        pose_icp_ = std::make_pair(pos, mat);
    }

    void cloud2OdomCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom){
        debug_tools::Timer t_0;
        odom_p_ = Eigen::Vector3d(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);
        auto odom_q = Eigen::Quaterniond(msg_odom->pose.pose.orientation.w, msg_odom->pose.pose.orientation.x, 
                msg_odom->pose.pose.orientation.y, msg_odom->pose.pose.orientation.z).matrix();

        auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg_cloud, *cloud_in);
        if(cloud_in->points.size() <= 0) return;

        std::for_each(std::execution::par, cloud_in->points.begin(), cloud_in->points.end(), [&](pcl::PointXYZI& p){
            Eigen::Vector3d point_origin(p.x, p.y, p.z), point_world;
            point_world = pose_icp_.second*(odom_q * point_origin + odom_p_) + pose_icp_.first;
            p.x = point_world(0);
            p.y = point_world(1);
            p.z = point_world(2);
        });
        Eigen::Vector3d pos_map = pose_icp_.second * odom_p_ + pose_icp_.first;
        map_->updateMap(cloud_in, pos_map);
        debug_tools::Timer t_1(t_0); t_1.log("prob map upate:", "ms");
    }

    void visualCallBack(){
        if(pub_cloud_->get_subscription_count() < 1) return;
        auto cloud = map_->getMapTSDF();
        sensor_msgs::msg::PointCloud2 msg_cloud;;
        pcl::toROSMsg(*cloud, msg_cloud);
        msg_cloud.header.frame_id = "map";
        msg_cloud.header.stamp = this->now();
        pub_cloud_->publish(msg_cloud);
    }

public:
    LocalNode(): Node("altitude_map_local_node", rclcpp::NodeOptions()){
        map_ = std::make_shared<LocalMap>();
        Eigen::Vector3d bound_min{-2, -2, -1}, bound_max{2, 2, 0.1};
        double resolution{0.05};
          if (bound_max.x() <= bound_min.x() ||
    bound_max.y() <= bound_min.y() ||
    bound_max.z() <=bound_min.z()) {
    std::cerr << "Error: max boundary must be greater than min boundary!" << std::endl;
    return;  }// 或者抛出异常或终止程序
        map_->init(bound_min, bound_max, resolution);

        std::string topic_prob_map_out = declare_parameter<std::string>("topic_prob_map_out", "/prob_map_out");
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/altitude_map_test", 10);

        std::string topic_icp = declare_parameter<std::string>("topic_odom_icp", "/icp_odom");
        sub_odom_icp_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic_icp, 10, std::bind(&LocalNode::icpOdomCallBack, this, std::placeholders::_1));

        std::string topic_cloud_in = declare_parameter<std::string>("topic_cloud_in_add", "/cloud_registered_body");
        std::string topic_odom_in = declare_parameter<std::string>("topic_odom", "/fast_lio_odom");
        sub_cloud_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
        sub_cloud_->subscribe(this, topic_cloud_in);
        sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
        sub_odom_->subscribe(this, topic_odom_in);
        syn_chronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, 
                    nav_msgs::msg::Odometry>>(*sub_cloud_, *sub_odom_, 10);
        syn_chronizer_->registerCallback(std::bind(&LocalNode::cloud2OdomCallBack, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&LocalNode::visualCallBack, this));
    }
};
} //namespace altitude_map

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altitude_map::LocalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
