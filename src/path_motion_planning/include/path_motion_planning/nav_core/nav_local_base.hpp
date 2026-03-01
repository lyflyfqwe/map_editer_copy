#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_BASE_HPP
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_BASE_HPP

#include <execution>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rm_interface/srv/nav_core.hpp"
#include "rm_interface/srv/set_robot_mode.hpp"
#include "rm_interface/msg/local_state.hpp"
#include "rm_interface/msg/command.hpp"

#include "path_motion_planning/util/group_runner.h"
#include "path_motion_planning/util/vis_ros.h"
#include "debug_tools/debug_tools.h"

namespace path_motion_planning{
namespace nav_core{
class NavLocalBase: public util::VisRos, public util::Tools{
protected:
    struct GroupSensor{
        rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
        rclcpp::CallbackGroup::SharedPtr group;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_icp;
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose_icp;
        typedef std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>> SynchronizerCloudOdom;
        SynchronizerCloudOdom syn_chronizer;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_cloud;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom;
        bool init_icp{false}, init_odom{false};
        rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
        Eigen::Vector<double, 4> FOV{}; //down up left right
        double min_p_dis{0.15}; //雷达坐标系下点云的最小距离,小于则忽略,用于去除机器人自身点 

        std::string topic_sub_icp;
    }sensor_;

    struct GroupPlan{
        rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
        rclcpp::CallbackGroup::SharedPtr group;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_click;
        rclcpp::Service<rm_interface::srv::NavCore>::SharedPtr sev_nav;
        rclcpp::Service<rm_interface::srv::SetRobotMode>::SharedPtr sev_bot_mode;
        rclcpp::Publisher<rm_interface::msg::LocalState>::SharedPtr pub_local_state_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
        rclcpp::Publisher<rm_interface::msg::Command>::SharedPtr pub_cmd_;
        util::GroupRun run_local, run_global;
        std::thread thread_cmd_;
        struct Line{
            std::vector<Eigen::Vector2d> path_2d; 
        }line;

        std::string topic_sub_pos_2d, topic_sub_click, topic_service_nav, topic_service_bot_mode;
    }plan_;

    struct GroupVisual{
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud, pub_cloud_g;
        std::thread td_show_cloud, td_show_cloud_g;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_safe_p, pub_safe_p_g, pub_for_test;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_robot;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path, pub_line;
        int sleep_pub_cloud_local, sleep_pub_cloud_g;
        double robot_z; //仅用于可视化，2d-line时提供z轴高度
    }visual_;

    void callBackIcp(const nav_msgs::msg::Odometry::ConstSharedPtr& msg_icp){
        auto pos = Eigen::Vector3d(msg_icp->pose.pose.position.x, msg_icp->pose.pose.position.y, msg_icp->pose.pose.position.z);
        auto mat = Eigen::Quaterniond(msg_icp->pose.pose.orientation.w, msg_icp->pose.pose.orientation.x, 
                msg_icp->pose.pose.orientation.y, msg_icp->pose.pose.orientation.z).matrix();
        sensor_.pose_icp = std::make_pair(pos, mat);
        sensor_.init_icp = true;
    }

    /**
     * @brief 加载plan相关的参数
     */
    template<typename T> inline void loadParamsOfPlan(const T& nav_plan, rclcpp::Node* node){
        auto bound_min = node->declare_parameter<std::vector<double>>("plan.local.bound_min", {-2.5, -2.5, -0.8});
        util::Tools::vecToEigen(bound_min, nav_plan->nav_cfg_.bound_min);
    
        auto bound_max = node->declare_parameter<std::vector<double>>("plan.local.bound_max", {2.5, 2.5, 0.3});
        util::Tools::vecToEigen(bound_max, nav_plan->nav_cfg_.bound_max);
    
        nav_plan->nav_cfg_.resolution = node->declare_parameter<double>("plan.local.resolution", 0.05);
    
        nav_plan->nav_cfg_.safe_cost = node->declare_parameter<double>("plan.local.safe_cost", 0.6);
    
        nav_plan->nav_cfg_.safe_radius = node->declare_parameter<int>("plan.local.safe_radius", 3);
    
        auto offset_robot_p = node->declare_parameter<std::vector<double>>("plan.local.offset_robot_p", {0.0, 0.07, -0.07});
        util::Tools::vecToEigen(offset_robot_p, nav_plan->l_cfg_.offset_robot_p);
    
        auto offset_robot_rpy = 
            node->declare_parameter<std::vector<double>>("plan.local.offset_robot_rpy", {0.0, -45.0, -90.0});
        nav_plan->l_cfg_.offset_robot_m = util::Tools::eulerToQuaternion( offset_robot_rpy.at(0)/180.0*M_PI, 
                offset_robot_rpy.at(1)/180.0*M_PI, offset_robot_rpy.at(2)/180.0*M_PI ).matrix();
    
        auto prob_map_size = node->declare_parameter<std::vector<double>>("plan.local.prob.map_size", {7.0, 7.0, 2.0});
        util::Tools::vecToEigen(prob_map_size, nav_plan->l_cfg_.p_cfg.map_size_d);

        nav_plan->l_cfg_.alti_cfg.trun_radius = node->declare_parameter<double>("plan.local.altitude.trun_radius", 0.4);

        nav_plan->l_cfg_.alti_cfg.down_spread_dis = node->declare_parameter<int>("plan.local.altitude.down_spread_dis", 4);
    
        nav_plan->l_cfg_.sec_cfg.v_cost = node->declare_parameter<double>("plan.local.search.v_cost", 30.0);
    
        nav_plan->l_cfg_.sec_cfg.dead_cost = node->declare_parameter<double>("plan.local.search.dead_cost", 0.9);
        debug_tools::Debug().print("[] search dead_cost:", nav_plan->l_cfg_.sec_cfg.dead_cost);
    
        nav_plan->l_cfg_.sec_cfg.num_limit = node->declare_parameter<int>("plan.local.search.num_limit", 20000);

        nav_plan->l_cfg_.sec_cfg.z_error_thres = node->declare_parameter<int>("plan.local.search.z_error_thres", 10);
    
        nav_plan->l_cfg_.ctl_cfg.traj_ahead_num = node->declare_parameter<int>("plan.local.control.traj_ahead_num", 20);
    
        nav_plan->l_cfg_.ctl_cfg.max_accel_add = node->declare_parameter<double>("plan.local.control.max_accel_add", 6.0);
    
        nav_plan->l_cfg_.ctl_cfg.max_accel_del = node->declare_parameter<double>("plan.local.control.max_accel_del", 12.0);
    
        nav_plan->l_cfg_.ctl_cfg.max_speed = node->declare_parameter<double>("plan.local.control.max_speed", 1.5);
    
        nav_plan->l_cfg_.ctl_cfg.speed_diff_rate = node->declare_parameter<double>("plan.local.control.speed_diff_rate", 2.39);

        nav_plan->l_cfg_.ctl_cfg.speed_yaw_diff_rate = node->declare_parameter<double>("plan.local.control.speed_yaw_diff_rate", 1.0);
    
        nav_plan->l_cfg_.ctl_cfg.control_interval_ms = node->declare_parameter<int>("plan.local.control.control_interval_ms", 10);
    
        nav_plan->l_cfg_.ctl_cfg.stop_dis = node->declare_parameter<double>("plan.local.control.stop_distance", 1.0);
    
        nav_plan->l_cfg_.ctl_cfg.stop_yaw = node->declare_parameter<double>("plan.local.control.stop_yaw", 0.15);

        std::string topic_pub_odom = node->declare_parameter<std::string>("plan.topic_pub_odom", "/nav_core_odom");
        plan_.pub_odom = node->create_publisher<nav_msgs::msg::Odometry>(topic_pub_odom, rclcpp::QoS(1).best_effort());

        std::string topic_pub_local_state = node->declare_parameter<std::string>("plan.topic_pub_local_state", "/nav_core_local_state");
        plan_.pub_local_state_ = node->create_publisher<rm_interface::msg::LocalState>(topic_pub_local_state, 10);

        plan_.run_local.sleep_run = node->declare_parameter<int>("plan.local.sleep_time_ms", 50);

        plan_.run_global.sleep_run = node->declare_parameter<int>("plan.global.sleep_time_ms", 500);

        std::string topic_pub_cmd = node->declare_parameter<std::string>("motion.topic_pub_cmd", "/motion_cmd");
        plan_.pub_cmd_ = node->create_publisher<rm_interface::msg::Command>(topic_pub_cmd, 10);

        plan_.group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); //同一时间只允许一个回调执行
        plan_.options.callback_group = plan_.group;

        plan_.topic_sub_pos_2d = node->declare_parameter<std::string>("plan.topic_pos_2d", "/rm_nav_goal");

        plan_.topic_sub_click = node->declare_parameter<std::string>("plan.topic_sub_click", "/clicked_point");

        plan_.topic_service_nav = node->declare_parameter<std::string>("plan.service_nav", "/rm_nav");

        plan_.topic_service_bot_mode = node->declare_parameter<std::string>("plan.service_bot_mode", "/rm_bot_mode");
    }

    /**
     * @brief 加载sensor相关的参数
     */
    inline void loadParamsOfSensor(rclcpp::Node* node){
        auto param_FOV = node->declare_parameter<std::vector<double>>("sensor.FOV", {-7, 52, -180.0, 0.0});
        for(auto& v : param_FOV) v = v/180.0*M_PI; //角度转弧度
        vecToEigen(param_FOV, sensor_.FOV);

        sensor_.min_p_dis = node->declare_parameter<double>("sensor.min_p_dis", 0.3);

        sensor_.group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant); //允许多个回调同时执行
        sensor_.options.callback_group = sensor_.group;

        sensor_.topic_sub_icp = node->declare_parameter<std::string>("sensor.topic_sub_icp", "/icp_odom");

        std::string topic_sub_cloud = node->declare_parameter<std::string>("sensor.topic_sub_cloud", "/cloud_registered_body");
        sensor_.sub_cloud = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
        sensor_.sub_cloud->subscribe(node, topic_sub_cloud, rmw_qos_profile_sensor_data, sensor_.options);

        std::string topic_sub_odom = node->declare_parameter<std::string>("sensor.topic_sub_odom", "/fast_lio_odom");
        sensor_.sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
        sensor_.sub_odom->subscribe(node, topic_sub_odom, rmw_qos_profile_sensor_data, sensor_.options);

        sensor_.syn_chronizer = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>(
                *sensor_.sub_cloud, *sensor_.sub_odom, 10);
    }

    /**
     * @brief 加载visual相关的参数
     */
    inline void loadParamsOfVisual(rclcpp::Node* node){
        std::string topic_pub_cloud_local = node->declare_parameter<std::string>("visual.topic_pub_cloud_local", "/nav_core_cloud_l");
        visual_.pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_cloud_local, 10);

        std::string topic_pub_cloud_global = node->declare_parameter<std::string>("visual.topic_pub_cloud_global", "/nav_core_cloud_g");
        visual_.pub_cloud_g = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_cloud_global, 10);

        visual_.sleep_pub_cloud_local = node->declare_parameter<int>("visual.sleep_pub_cloud_local_ms", 50);
        visual_.sleep_pub_cloud_g = node->declare_parameter<int>("visual.sleep_pub_cloud_global_ms", 1000);

        std::string topic_pub_safe_p = node->declare_parameter<std::string>("visual.topic_pub_safe_p", "/nav_core_safe_p");
        visual_.pub_safe_p = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic_pub_safe_p, 10);

        std::string topic_pub_safe_p_g = node->declare_parameter<std::string>("visual.topic_pub_safe_p_g", "/nav_core_safe_p_g");
        visual_.pub_safe_p_g = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic_pub_safe_p_g, 10);

        std::string topic_pub_for_test = node->declare_parameter<std::string>("visual.topic_pub_for_test", "/nav_core_for_test");
        visual_.pub_for_test = node->create_publisher<visualization_msgs::msg::MarkerArray>(topic_pub_for_test, 10);

        std::string topic_pub_robot = node->declare_parameter<std::string>("visual.topic_pub_robot", "/nav_core_robot");
        visual_.pub_robot = node->create_publisher<visualization_msgs::msg::Marker>(topic_pub_robot, 10);

        std::string topic_pub_path_local = node->declare_parameter<std::string>("visual.topic_pub_path_local", "/nav_core_path_l");
        visual_.pub_path = node->create_publisher<nav_msgs::msg::Path>(topic_pub_path_local, 10);

        std::string topic_pub_path_global = node->declare_parameter<std::string>("visual.topic_pub_path_global", "/nav_core_path_g");
        visual_.pub_line= node->create_publisher<nav_msgs::msg::Path>(topic_pub_path_global, 10);

        visual_.tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }

    /**
     * @brief 发布机器人中心的里程计，包含x,y的速度
     */
    template<typename T> inline void pubNavOdom(T nav_plan, rclcpp::Node* node){
        auto [p, q, v] = nav_plan->getRobotState();
        visual_.robot_z = p.z();
    
        nav_msgs::msg::Odometry msg_odom;
        msg_odom.header.frame_id = "map";
        msg_odom.header.stamp = node->now();
        msg_odom.child_frame_id = "robot";
    
        msg_odom.pose.pose.position.x = p.x();
        msg_odom.pose.pose.position.y = p.y();
        msg_odom.pose.pose.position.z = p.z();
    
        msg_odom.pose.pose.orientation.w = q.w();
        msg_odom.pose.pose.orientation.x = q.x();
        msg_odom.pose.pose.orientation.y = q.y();
        msg_odom.pose.pose.orientation.z = q.z();
    
        msg_odom.twist.twist.linear.x = v.x();
        msg_odom.twist.twist.linear.y = v.y();
    
        plan_.pub_odom->publish(msg_odom);

        // geometry_msgs::msg::TransformStamped msg_tf;
        // msg_tf.header.stamp = node->now();
        // msg_tf.header.frame_id = "map";
        // msg_tf.child_frame_id = "robot";

        // msg_tf.transform.translation.x = p.x();
        // msg_tf.transform.translation.y = p.y();
        // msg_tf.transform.translation.z = p.z();
        // msg_tf.transform.rotation.x = q.x();
        // msg_tf.transform.rotation.y = q.y();
        // msg_tf.transform.rotation.z = q.z();
        // msg_tf.transform.rotation.w = q.w();

        // visual_.tf_broadcaster->sendTransform(msg_tf);
    }

    /**
     * @brief 可视化 local_map
     */
    template<typename T> inline void visCloud(const T& nav_plan, rclcpp::Node* node){
        while(rclcpp::ok()){
            if(visual_.pub_cloud->get_subscription_count() >= 1 && sensor_.init_icp && sensor_.init_odom){
                auto cloud = nav_plan->getCloud();
                if(!cloud->empty()) util::VisRos::pubCloudVisual(visual_.pub_cloud, node, *cloud);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(visual_.sleep_pub_cloud_local));
        }
    }

    /**
     * @brief 发送控制命令
     */
    template<typename T> inline void sendCommand(const T& nav_plan, rclcpp::Node*){
        while(rclcpp::ok()){
            Eigen::Vector2d robot_map_spd;
            auto cmd = nav_plan->getCommand(robot_map_spd);
            rm_interface::msg::Command msg_cmd;
            msg_cmd.available = true;
            msg_cmd.x = cmd.x();
            msg_cmd.y = cmd.y();
            msg_cmd.r = cmd.z();
            msg_cmd.x_m = robot_map_spd.x();
            msg_cmd.y_m = robot_map_spd.y();
            plan_.pub_cmd_->publish(msg_cmd);

            std::this_thread::sleep_for(std::chrono::milliseconds(nav_plan->l_cfg_.ctl_cfg.control_interval_ms));
        }
    }

    /**
     * @brief 接收雷达里程计和点云，更新状态
     */
    template<typename T> inline void updateOdomAndCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
            const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom, T nav_plan, rclcpp::Node* node){
        const rclcpp::Time cur_time = msg_odom->header.stamp;
        if(sensor_.last_time.nanoseconds() == 0){
            sensor_.last_time = cur_time; 
            return;
        } 
        auto odom_p = Eigen::Vector3d(msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);
        auto odom_q = Eigen::Quaterniond(msg_odom->pose.pose.orientation.w, msg_odom->pose.pose.orientation.x, 
                msg_odom->pose.pose.orientation.y, msg_odom->pose.pose.orientation.z).matrix();
        auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*msg_cloud, *cloud_in);
        if(cloud_in->points.size() <= 0) return;
        std::for_each(std::execution::par, cloud_in->points.begin(), cloud_in->points.end(), [&](pcl::PointXYZI& p){
            Eigen::Vector3d point_origin(p.x, p.y, p.z);
            // 计算水平角和俯仰角（弧度）
            double theta = std::atan2(point_origin.y(), point_origin.x());
            double phi = std::atan2(point_origin.z(), point_origin.head<2>().norm());
            double distance = point_origin.norm();
            if(phi < sensor_.FOV[0] || phi > sensor_.FOV[1] || theta < sensor_.FOV[2] || theta > sensor_.FOV[3] || distance < sensor_.min_p_dis){
                p.x = p.y = p.z = NAN;
            }
            else if(p.intensity < 2) p.x = p.y = p.z = NAN;
            else{
                Eigen::Vector3d point_world = sensor_.pose_icp.second * (odom_q * point_origin + odom_p) + sensor_.pose_icp.first;
                p.x = point_world.x();
                p.y = point_world.y();
                p.z = point_world.z();
            }
        });
        // 删除标记为NaN的点
        cloud_in->points.erase(
            std::remove_if(cloud_in->points.begin(), cloud_in->points.end(),
                [](const pcl::PointXYZI& p) {
                    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
                }),
            cloud_in->points.end()
        );
        Eigen::Vector3d map_p = sensor_.pose_icp.second * odom_p + sensor_.pose_icp.first;
        auto map_q = sensor_.pose_icp.second * odom_q;
        const double dt = (cur_time-sensor_.last_time).seconds();
        sensor_.last_time = cur_time;
        nav_plan->updateMap(cloud_in, map_p, map_q, dt);
        sensor_.init_odom = true;
        pubNavOdom(nav_plan, node);
    }
}; //NavLocalBase
} //namespace nav_core
} //namespace path_planning

#endif