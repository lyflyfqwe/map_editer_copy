#include <thread>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "rm_interface/msg/command.hpp"
#include "debug_tools/debug_tools.h"

namespace path_planning{
namespace test{
class VirtualSpeed: public rclcpp::Node{
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_info_;
    rclcpp::Publisher<rm_interface::msg::Command>::SharedPtr pub_cmd_;
    Eigen::Vector3d nav_start_p_;
    bool is_spin_{false}; 
    bool speed_mode_{true}; //true: ahead false:back
    std::thread thd_cmd_;

    void callBackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg){
        auto pos = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, 
                msg->pose.pose.position.z);
        if(is_spin_){
            nav_start_p_ = pos;
            return;
        } 
        if((nav_start_p_ - pos).norm() > 0.5){
            speed_mode_ = !speed_mode_;
            if(speed_mode_) is_spin_ = true;
            nav_start_p_ = pos;
        } 
        
    }

    void callBackInfo(const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg){
        if(msg->data.size() < 2){
            debug_tools::Debug().print("msg->data.size() < 2");
            return;
        }
        if(msg->data.at(1) == 0) is_spin_ = true;
    }

    void callBackSendCmd(){
        static int spin_counter{0};
        while(rclcpp::ok()){
            rm_interface::msg::Command msg_cmd;
            if(is_spin_){
                if(spin_counter++ > 100){
                    spin_counter = 0;
                    is_spin_ = false;
                }
                msg_cmd.x = 0;
                msg_cmd.y = 0;
                msg_cmd.r = 1;
            }
            else{
                msg_cmd.x = speed_mode_? 2 : -2;
                msg_cmd.y = 0;
                msg_cmd.r = 0;
            }
            pub_cmd_->publish(msg_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

public:
    VirtualSpeed(): Node("virtual_speed_node", rclcpp::NodeOptions()){
        pub_cmd_ = this->create_publisher<rm_interface::msg::Command>("/motion_cmd", 1);
        sub_info_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/rm_sentry_info", 10, std::bind(&VirtualSpeed::callBackInfo, this, std::placeholders::_1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/nav_core_odom", 10, std::bind(&VirtualSpeed::callBackOdom, this, std::placeholders::_1));
        thd_cmd_ = std::thread(&VirtualSpeed::callBackSendCmd, this);
    }

    ~VirtualSpeed(){
        if(thd_cmd_.joinable()) thd_cmd_.join();
        debug_tools::Debug().print("~VirtualSpeed");
    }
};
} //namespace test
} //namespace path_planning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning::test::VirtualSpeed>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}