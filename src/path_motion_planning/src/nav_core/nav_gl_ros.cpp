#include "path_motion_planning/nav_core/nav_gl_ros.hpp"

namespace path_motion_planning{
namespace nav_core{

void NavGLRos::callBackCloudOdom(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom){
    debug_tools::Timer t_begin;
    updateOdomAndCloud(msg_cloud, msg_odom, nav_plan_, this);
    debug_tools::Timer t_end(t_begin); t_end.log("callBackCloudOdom", "ms");
}

inline void NavGLRos::loadParamsOfPlanG(NavGL::Ptr& nav_plan){
    auto bound_min = declare_parameter<std::vector<double>>("plan.global.bound_min", {-10.0, -10.0, -1});
    util::Tools::vecToEigen(bound_min, nav_plan->nav_cfg_g_.m_cfg.bound_min);

    auto bound_max = declare_parameter<std::vector<double>>("plan.global.bound_max", {10, 10, 1});
    util::Tools::vecToEigen(bound_max, nav_plan->nav_cfg_g_.m_cfg.bound_max);

    nav_plan->nav_cfg_g_.m_cfg.resolution = declare_parameter<double>("plan.global.resolution", 0.05);
    
    nav_plan->nav_cfg_g_.m_cfg.safe_cost = declare_parameter<double>("plan.global.safe_cost", 0.3);
    
    nav_plan->nav_cfg_g_.m_cfg.safe_radius = declare_parameter<int>("plan.global.safe_radius", 3);

    nav_plan->nav_cfg_g_.m_cfg.pcd_file = declare_parameter<std::string>(
            "plan.global.pcd_map", "/home/qwelyflyf/Documents/point_cloud/flower_2.pcd");

    nav_plan->nav_cfg_g_.alti_cfg.trun_radius = declare_parameter<double>("plan.global.altitude.trun_radius", 0.4);

    nav_plan->nav_cfg_g_.alti_cfg.down_spread_dis = declare_parameter<int>("plan.global.altitude.down_spread_dis", 2);

    nav_plan->nav_cfg_g_.sec_cfg.v_cost = declare_parameter<double>("plan.global.search.v_cost", 30.0);
    
    nav_plan->nav_cfg_g_.sec_cfg.dead_cost = declare_parameter<double>("plan.global.search.dead_cost", 0.9);
    debug_tools::Debug().print("G search dead_cost:", nav_plan->nav_cfg_g_.sec_cfg.dead_cost);
    
    nav_plan->nav_cfg_g_.sec_cfg.num_limit = declare_parameter<int>("plan.global.search.num_limit", 200000);
}

inline void NavGLRos::tryGlobalNav(){
    result_path_global_ = 0;

    std::vector<Eigen::Vector3d> safe_p;
    auto result = nav_plan_->selectPosG(goal_global_, safe_p);
    util::VisRos::pubBallVisual(visual_.pub_safe_p_g, this, safe_p, {0.0f, 0.0f, 1.0f});
    if(!result){
        debug_tools::Debug().print("selectPos G failed");
        result_path_global_ = 1;
        return;
    }

    std::vector<Eigen::Vector4d> path;
    if(!nav_plan_->searchPathG(path, is_new_task_global_)){
        debug_tools::Debug().print("searchPath G failed");
        result_path_global_ = 3;
        return;
    }
    is_new_task_global_ = false;

    util::VisRos::pubPathVisual(visual_.pub_line, this, path);
    plan_.run_local.setPauseState(true);
    nav_plan_->bufferGlobalPath(path);
    plan_.run_local.setPauseState(false);
}

void NavGLRos::callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg){
    goal_global_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    debug_tools::Debug().print("---------------------GlobalNav[", 
            goal_global_.x(), goal_global_.y(), goal_global_.z(), "]-------------------------");
    is_new_task_global_ = true;
    is_reach_goal_ = false;
    plan_.run_global.setPauseState(true);
    tryGlobalNav();
    plan_.run_global.setPauseState(false);
}

void NavGLRos::callBackNavServer(const rm_interface::srv::NavCore::Request::SharedPtr& req,
        const rm_interface::srv::NavCore::Response::SharedPtr& res){
    // plan_.plan->setDiedCostG(req->died_cost);
    goal_global_ = Eigen::Vector3d(req->x, req->y, req->z);
    debug_tools::Debug().print("---------------------GlobalNav[", 
        goal_global_.x(), goal_global_.y(), goal_global_.z(), "]-------------------------");
    is_new_task_global_ = true;
    is_reach_goal_ = false;
    plan_.run_global.setPauseState(true);
    tryGlobalNav();
    plan_.run_global.setPauseState(false);
    res->result = result_path_global_;
}

void NavGLRos::callBackBotMode(const rm_interface::srv::SetRobotMode::Request::SharedPtr& req,
        const rm_interface::srv::SetRobotMode::Response::SharedPtr& res){
    auto spd_rate = req->robot_speed_rate;
    debug_tools::Debug().print("---------------------GlobalBotMode[", spd_rate, "]-------------------------");
    nav_plan_->setSpeedRate(spd_rate);
    res->result = true;
}

inline void NavGLRos::tryLocalNav(){
    std::vector<Eigen::Vector3d> safe_p;
    rm_interface::msg::LocalState msg_l_state;

    auto result_select_pos = nav_plan_->selectPosFromGPath(safe_p, msg_l_state.safe_state);
    if(!result_select_pos){
        debug_tools::Debug().print("selectPos L failed");
        msg_l_state.result = 1;
    }
    else util::VisRos::pubBallVisual(visual_.pub_safe_p, this, safe_p, {0.0f, 1.0f, 0.0f});

    std::vector<Eigen::Vector3d> path;
    if(result_select_pos){
        if(nav_plan_->seachPath(path, msg_l_state.safe_state)) msg_l_state.result = 0; 
        else{
            debug_tools::Debug().print("searchPath L failed");
            msg_l_state.result = 3;
        }
    }

    nav_plan_->updatePathLForControl(path);
    plan_.pub_local_state_->publish(msg_l_state);
    util::VisRos::pubPathVisual(visual_.pub_path, this, path);
}

inline void NavGLRos::visCloudGlocal(){
    while(rclcpp::ok()){
        if(visual_.pub_cloud_g->get_subscription_count() >= 1){
            auto cloud = nav_plan_->getCloudG();
            if(!cloud->empty()) util::VisRos::pubCloudVisual(visual_.pub_cloud_g, this, *cloud);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(visual_.sleep_pub_cloud_g));
    }
}

NavGLRos::NavGLRos(): Node("nav_gl_ros_node", rclcpp::NodeOptions()){
    nav_plan_ = std::make_shared<NavGL>();
    loadParamsOfPlan(nav_plan_, this);
    loadParamsOfPlanG(nav_plan_);
    // nav_plan_->setDefaultParams();
    nav_plan_->init();

    plan_.run_local.work = std::bind(&NavGLRos::tryLocalNav, this);
    plan_.run_local.setPauseState(true);  //未接收到全局路径，关闭局部搜索

    loadParamsOfSensor(this);
    sensor_.sub_odom_icp = this->create_subscription<nav_msgs::msg::Odometry>(
        sensor_.topic_sub_icp, 10, std::bind(&NavGLRos::callBackIcp, this, std::placeholders::_1), sensor_.options);
    sensor_.syn_chronizer->registerCallback(
        std::bind(&NavGLRos::callBackCloudOdom, this, std::placeholders::_1, std::placeholders::_2));

    loadParamsOfVisual(this);

    visual_.td_show_cloud = std::thread(&NavGLRos::visCloud<NavGL::Ptr>, this, std::ref(nav_plan_), this);

    visual_.td_show_cloud_g = std::thread(&NavGLRos::visCloudGlocal, this);

    plan_.thread_cmd_ = std::thread(&NavGLRos::sendCommand<NavGL::Ptr>, this, std::ref(nav_plan_), this);

    plan_.sub_click = this->create_subscription<geometry_msgs::msg::PointStamped>(
        plan_.topic_sub_click, 10, 
        std::bind(&NavGLRos::callBackClick, this, std::placeholders::_1), plan_.options);

    plan_.sev_nav = this->create_service<rm_interface::srv::NavCore>(
        plan_.topic_service_nav, 
        std::bind(&NavGLRos::callBackNavServer, this, std::placeholders::_1, std::placeholders::_2), 
        rmw_qos_profile_services_default, plan_.group);

    plan_.sev_bot_mode = this->create_service<rm_interface::srv::SetRobotMode>(
        plan_.topic_service_bot_mode,
        std::bind(&NavGLRos::callBackBotMode, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, plan_.group);

    plan_.run_local.run();

    plan_.run_global.work = std::bind(&NavGLRos::tryGlobalNav, this);
    plan_.run_global.setPauseState(true);  //未接收到全局路径，关闭局部搜索
    plan_.run_global.run();
}

NavGLRos::~NavGLRos(){
    plan_.run_local.stop();
    plan_.run_global.stop();
    if(visual_.td_show_cloud.joinable()) visual_.td_show_cloud.join();
    if(visual_.td_show_cloud_g.joinable()) visual_.td_show_cloud_g.join();
    if(plan_.thread_cmd_.joinable()) plan_.thread_cmd_.join();
}
} //namespace nav_core
} //namespace path_planning

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_motion_planning::nav_core::NavGLRos>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}