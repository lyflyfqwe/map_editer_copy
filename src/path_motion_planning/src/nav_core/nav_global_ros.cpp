#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "path_motion_planning/nav_core/nav_global.h"
#include "path_motion_planning/util/vis_ros.h"

namespace path_motion_planning{
namespace nav_core{
class NavGlobalRos: public rclcpp::Node, public util::VisRos{
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pos_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_click_;

    NavGlobal::Ptr plan_;
    std::vector<Eigen::Vector3d> ps_safe_;
    std::thread thread_vis_;

    void callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg){
        auto pos = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
        debug_tools::Debug().print("click: [", pos.x(), pos.y(), pos.z(), "]");
        Eigen::Vector3d p_safe;
        auto size = plan_->selectPos(pos, p_safe);
        if(ps_safe_.size() > size || ps_safe_.size() >= 2) ps_safe_.clear();
        ps_safe_.push_back(p_safe);
        util::VisRos::pubBallVisual(pub_pos_, this, ps_safe_, {1.0f, 0.0f, 0.0f});

        std::vector<Eigen::Vector3d> path;
        if(size != 2 || !plan_->seachPath(path)) return;
        util::VisRos::pubPathVisual(pub_path_, this, path);
    }

    inline void rosRelated(){
        std::string topic_pub_cloud = declare_parameter<std::string>("visual.topic_pub_cloud_global", "/nav_core_cloud_g");
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_cloud, 10);

        std::string topic_pub_safe_p = declare_parameter<std::string>("visual.topic_pub_safe_p", "/nav_core_safe_p");
        pub_pos_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_pub_safe_p, 10);

        std::string topic_pub_path = declare_parameter<std::string>("visual.topic_pub_path_global", "/nav_core_path_g");
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_pub_path, 10);
        
        sub_click_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
           "/clicked_point", 10, std::bind(&NavGlobalRos::callBackClick, this, std::placeholders::_1));
    }

public:
    NavGlobalRos(): Node("nav_global_ros_node", rclcpp::NodeOptions()){
        plan_ = std::make_shared<NavGlobal>();
        plan_->setDefaultParams();
        std::string path_pcd = declare_parameter<std::string>(
                "plan.global.pcd_map", "/home/simpletime/code/nav/map_0.pcd");
        plan_->init(path_pcd);

        //
        rosRelated();
        thread_vis_ = std::thread([this](){
            while(rclcpp::ok()){
                auto cloud = plan_->getMap();
                debug_tools::Debug().print("Points :",cloud->points.size());
                util::VisRos::pubCloudVisual(pub_cloud_, this, *cloud);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }); 
    }

    ~NavGlobalRos(){
        if(thread_vis_.joinable()) thread_vis_.join();
    }
};
} //namespace nav_core
} //namespace path_motion_planning

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_motion_planning::nav_core::NavGlobalRos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

