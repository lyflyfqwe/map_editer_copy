#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "path_motion_planning/nav_core/nav_fixed_global.h"
#include "path_motion_planning/util/vis_ros.h"
#include "path_motion_planning/nav_core/nav_fixed_global_ros.h"

namespace path_motion_planning{
namespace nav_core{
NavFixedGlobalRos::NavFixedGlobalRos(): Node("nav_global_ros_node", rclcpp::NodeOptions()),
util::VisRos(),
whatMode(0),
whatCubePoint(0)
{
    plan_ = std::make_shared<NavFixedGlobal>();
    save_ = std::make_shared<altitude_map::FixedGlobalMap>();
    plan_->setDefaultParams();
    std::string path_pcd = declare_parameter<std::string>(
            "path_global_pcd", "/home/qwelyflyf/Documents/point_cloud/flower_2.pcd");
    // "/home/simpletime/Project/lab_tf_4_23.pcd"
    // "/home/simpletime/nav/out.pcd"
    //将点云入传入nav核心进行初始化
    if (plan_){
        plan_->init(path_pcd);
        save_=plan_->getMapPtr();
    } 


    rosRelated();
    thread_vis_ = std::thread([this](){
        while(rclcpp::ok()){
            // t = 1;
            // if(t) continue;
            auto cloud = plan_->getMap();
            util::VisRos::pubCloudVisual(pub_cloud_, this, *cloud);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        } 
    }); 
}

NavFixedGlobalRos::~NavFixedGlobalRos(){
    if(thread_vis_.joinable()) thread_vis_.join();
}

// bool t = false;

//2Dpose插件点击切换状态
void NavFixedGlobalRos::callBackSwich(const geometry_msgs::msg::PoseStamped& msg){
    whatMode++;
    if(whatMode > 3) whatMode = 0;
    debug_tools::Debug().print("Switch to", whatMode);
    if(whatMode == 3){
        debug_tools::Debug().print("cubes",cubes.size());
        debug_tools::Debug().print("cubes_del",cubes_del.size());
        
        if (save_) save_->saveCloudToPcd(cubes, cubes_del);
        debug_tools::Debug().print("保存点云完成");
    }
}

    //点击回调函数
void NavFixedGlobalRos::callBackClick(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg){
    //获取点击位置
    auto pos = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    debug_tools::Debug().print("click: [", pos.x(), pos.y(), pos.z(), "]");
    
    if(whatMode == 0){
        // plan_->showPoint2Map(pos);
        whatCubePoint++;
        if (whatCubePoint == 1){
            cube.first = pos;
            debug_tools::Debug().print("第一个点",cube.first.x(),cube.first.y(),cube.first.z());
        }
        if (whatCubePoint == 2){
            whatCubePoint = 0;
            cube.second = pos;
            debug_tools::Debug().print("第二个点",cube.second.x(),cube.second.y(),cube.second.z());
            plan_->addCubePoint2Map(cube, cubes);
            if(cubes.size() > 0 ){
                for (auto &c : cubes) {
                    debug_tools::Debug().print("cube11111:", 
                        c.first.x(), c.first.y(), c.first.z(),
                        c.second.x(), c.second.y(), c.second.z());
                    }
            }
        }
    }else if(whatMode == 1){ 
        whatCubePoint++;
        if (whatCubePoint == 1){
            cube_del.first = pos;
            debug_tools::Debug().print("第一个点", cube.first.x(), cube.first.y(), cube.first.z());
        }
        if (whatCubePoint == 2){
            whatCubePoint = 0;
            cube_del.second = pos;
            debug_tools::Debug().print("第二个点", cube.second.x(), cube.second.y(), cube.second.z());
            plan_->delCubePoint2Map(cube_del, cubes_del);
            
        }
    }else if(whatMode == 2){
        Eigen::Vector3d p_safe;
        auto size = plan_->selectPos(pos, p_safe);
        if(ps_safe_.size() > size || ps_safe_.size() >= 2) ps_safe_.clear();
        ps_safe_.push_back(p_safe);
        debug_tools::Debug().print("启动导航");
        std::vector<Eigen::Vector3d> path;
        if(size != 2 || !plan_->seachPath(path)) return;
        util::VisRos::pubPathVisual(pub_path_, this, path);
        //发布点 发布者容器/发布节点/点队列/颜色
        util::VisRos::pubBallVisual(pub_pos_, this, ps_safe_, {1.0f, 0.0f, 0.0f});
    }
        
}

//ros话题初始化
inline void NavFixedGlobalRos::rosRelated(){
    std::string topic_pub_cloud = declare_parameter<std::string>("visual.topic_pub_cloud_global", "/nav_core_cloud_g");
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_cloud, 10);

    std::string topic_pub_safe_p = declare_parameter<std::string>("visual.topic_pub_safe_p", "/nav_core_safe_p");
    pub_pos_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_pub_safe_p, 10);

    std::string topic_pub_path = declare_parameter<std::string>("visual.topic_pub_path_global", "/nav_core_path_g");
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_pub_path, 10);
    
    sub_click_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&NavFixedGlobalRos::callBackClick, this, std::placeholders::_1));

    sub_pos_click_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,std::bind(&NavFixedGlobalRos::callBackSwich,this,std::placeholders::_1));
}

    
}
} //namespace nav_core
//namespace path_motion_planning

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_motion_planning::nav_core::NavFixedGlobalRos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

