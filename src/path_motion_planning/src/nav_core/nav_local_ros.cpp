#include "path_motion_planning/nav_core/nav_local_ros.hpp"

namespace path_motion_planning{
namespace nav_core{
void NavLocalRos::callBackCloudOdom(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odom){
    debug_tools::Timer t_begin;
    updateOdomAndCloud(msg_cloud, msg_odom, nav_plan_, this);
    debug_tools::Timer t_end(t_begin); t_end.log("callBackCloudOdom", "ms");
}

inline void NavLocalRos::tryLocalNav(){
    std::vector<Eigen::Vector3d> safe_p;
    rm_interface::msg::LocalState msg_l_state;

    if(!nav_plan_->selectPosFromLine(plan_.line.path_2d, safe_p)){
        debug_tools::Debug().print("selectPos failed");
        msg_l_state.result = 1;
        msg_l_state.result = true;
        plan_.pub_local_state_->publish(msg_l_state);
        return;
    }
    util::VisRos::pubBallVisual(visual_.pub_safe_p, this, safe_p, {1.0f, 0.0f, 0.0f});
    std::vector<Eigen::Vector3d> path;
    if(!nav_plan_->seachPath(path)){
        debug_tools::Debug().print("seachPath failed");
        msg_l_state.result = 3;
        msg_l_state.result = true;
        plan_.pub_local_state_->publish(msg_l_state);
    }
    util::VisRos::pubPathVisual(visual_.pub_path, this, path); //如果局部搜索失败，将发布空路径
}

inline void NavLocalRos::getLocalReferedLine(const Eigen::Vector3d& target_pose){
    plan_.line.path_2d = nav_plan_->getReferLine(target_pose);
    std::vector<Eigen::Vector3d> line_3d; //仅用于可视化 
    for(const auto& p : plan_.line.path_2d) line_3d.push_back(Eigen::Vector3d(p.x(), p.y(), visual_.robot_z));
    nav_plan_->updatePathG(line_3d);
    util::VisRos::pubPathVisual(visual_.pub_line, this, line_3d); //重复发布会影响motion校准global_pose
}

void NavLocalRos::callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg){
    plan_.run_local.setPauseState(true);
    auto target_pose = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    getLocalReferedLine(target_pose);
    plan_.run_local.setPauseState(false);
}

void NavLocalRos::callBackNavServer(const rm_interface::srv::NavCore::Request::SharedPtr& req,
        const rm_interface::srv::NavCore::Response::SharedPtr& res){
    plan_.run_local.setPauseState(true);
    auto target_pose = Eigen::Vector3d(req->x, req->y, req->z);
    getLocalReferedLine(target_pose);
    plan_.run_local.setPauseState(false);
    res->result = 0;
}

NavLocalRos::NavLocalRos(): Node("nav_local_ros_node", rclcpp::NodeOptions()){
    nav_plan_ = std::make_shared<NavLocal>();
    // nav_plan_->setDefaultParams();
    loadParamsOfPlan(nav_plan_, this);
    nav_plan_->init();

    plan_.run_local.work = std::bind(&NavLocalRos::tryLocalNav, this);
    plan_.run_local.setPauseState(true);  //未接收到全局路径，暂时关闭局部搜索

    loadParamsOfSensor(this);
    sensor_.sub_odom_icp = this->create_subscription<nav_msgs::msg::Odometry>(
        sensor_.topic_sub_icp, 10, std::bind(&NavLocalRos::callBackIcp, this, std::placeholders::_1), sensor_.options);
    sensor_.syn_chronizer->registerCallback(
        std::bind(&NavLocalRos::callBackCloudOdom, this, std::placeholders::_1, std::placeholders::_2));

    loadParamsOfVisual(this);

    visual_.td_show_cloud = std::thread(&NavLocalRos::visCloud<NavLocal::Ptr>, this, std::ref(nav_plan_), this);

    plan_.thread_cmd_ = std::thread(&NavLocalRos::sendCommand<NavLocal::Ptr>, this, std::ref(nav_plan_), this);

    plan_.sub_click = this->create_subscription<geometry_msgs::msg::PointStamped>(
            plan_.topic_sub_pos_2d, 10, std::bind(&NavLocalRos::callBackClick, this, std::placeholders::_1), plan_.options);

    plan_.sev_nav = this->create_service<rm_interface::srv::NavCore>(plan_.topic_service_nav, 
            std::bind(&NavLocalRos::callBackNavServer, this, std::placeholders::_1, std::placeholders::_2), 
            rmw_qos_profile_services_default, plan_.group);

    plan_.run_local.run(); //开始局部搜索的线程
}

NavLocalRos::~NavLocalRos(){
    plan_.run_local.stop();
    if(visual_.td_show_cloud.joinable()) visual_.td_show_cloud.join();
    if(plan_.thread_cmd_.joinable()) plan_.thread_cmd_.join();
    debug_tools::Debug().print("~NavLocalRos");
}

} //namespace nav_core
} //namespace path_motion_planning

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_motion_planning::nav_core::NavLocalRos>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}