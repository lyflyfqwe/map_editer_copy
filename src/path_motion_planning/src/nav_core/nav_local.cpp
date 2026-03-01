#include "path_motion_planning/nav_core/nav_local.h"

namespace path_motion_planning{
namespace nav_core{
std::vector<Eigen::Vector2d> 
NavLocal::interpolatePoints(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double resolution){
    std::vector<Eigen::Vector2d> points;  
    double distance = (end - start).norm();  
    int p_num = static_cast<int>(distance / resolution);  

    for(int i{0}; i <= p_num; ++i) {  
        double t = static_cast<double>(i) / p_num;  // 从0到1的插值因子  
        Eigen::Vector2d point = (1 - t) * start + t * end;  // 线性插值  
        points.push_back(point);  
    }  
    return points; 
}

bool NavLocal::selectTargetFromReferedLine(
        const Eigen::Vector3d& safe_start, const std::vector<Eigen::Vector2d>::iterator begin, 
        const std::vector<Eigen::Vector2d>::iterator end, Eigen::Vector3d& target){
    auto Func = [&](const Eigen::Vector3d& p) ->bool { //insidebox && can neighbour && sdf<safe_cost
        auto pos_local = p - lidar_p_;
        if(!map_->isInMap(pos_local)) return false;
        auto index = map_->getGridIndex(pos_local);
        return map_->canBeNeighbour(index) && map_->getSDFValue(index) < nav_cfg_.safe_cost;
    };
    if(!Func(safe_start)){
        debug_tools::Debug().print("selectTargetFromReferedLine failed start");
        return false;
    } 
    if(begin == end) return false;

    target = safe_start;
    if(map_->isInMap((end-1)->x()-lidar_p_.x(), (end-1)->y()-lidar_p_.y())){ //终点在local-map内
        auto neighbours = std::vector<double>{0, nav_cfg_.resolution, -nav_cfg_.resolution};
        bool flag{false};
        for(auto it = begin; it != end; it++){
            for(auto n : neighbours){
                auto pos = Eigen::Vector3d(it->x(), it->y(), target.z()+n);
                if(Func(pos)){
                    target = pos;
                    flag = true;
                } 
            }
        }
        return flag;
    }
    //线段终点在local-map 外 
    for(auto it{end}; it != begin;){
        --it;
        double l_x = it->x() - lidar_p_.x();
        double l_y = it->y() - lidar_p_.y();
        if(!map_->isInMap(l_x, l_y)){
            target.x() = it->x(); 
            target.y() = it->y();
        }
        else break;
    }
    return true;
}

bool NavLocal::selectTargetFromReferedPath(
        const Eigen::Vector3d& safe_start, const std::vector<Eigen::Vector4d>::iterator begin, 
        const std::vector<Eigen::Vector4d>::iterator end, Eigen::Vector3d& target, bool& safe_state){
    auto Func = [&](const Eigen::Vector3d& p) ->bool { //insidebox && can neighbour && sdf<safe_cost
        auto pos_local = p - lidar_p_;
        if(!map_->isInMap(pos_local)) return false;
        auto index = map_->getGridIndex(pos_local);
        return map_->canBeNeighbour(index) && map_->getSDFValue(index) < nav_cfg_.safe_cost;
    };
    if(!Func(safe_start)){
        debug_tools::Debug().print("selectTargetFromReferedLine failed start");
        return false;
    } 
    if(begin == end) return false;
    std::vector<Eigen::Vector4d>::iterator it_target; //用于判断是否需要进入跟随模式 

    if(map_->isInMap(Eigen::Vector3d((end-1)->x(), (end-1)->y(), (end-1)->z()) - lidar_p_)){  //终点在local-map内
        debug_tools::Debug().print("in map");
        auto neighbours = std::vector<double>{0, nav_cfg_.resolution, -nav_cfg_.resolution};
        bool flag{false};
        target = safe_start;
        it_target = begin;
        for(auto it{begin}; it != end; it++){
            for(auto n : neighbours){
                auto pos = Eigen::Vector3d(it->x(), it->y(), target.z()+n);
                if(Func(pos)){
                    target = pos;
                    it_target = it;
                    flag = true;
                    break;
                }
            }
        }
        for(auto it{it_target}; it != end; it++){ //仅用于判断是否存在sdf值超过预设值
            if(it->w() > 0.5){
                safe_state = false;
                debug_tools::Debug().print("sdf value > 0.5");
                break;
            }
        }
        return flag;
    }
    debug_tools::Debug().print("not in map");
    for(auto it{end}; it != begin;){ //线段终点在local-map 外 
        --it;
        auto pos = Eigen::Vector3d(it->x(), it->y(), it->z());
        if(!map_->isInMap(pos - lidar_p_)){
            target = pos;
            it_target = it;
        } 
        else break;
    }
    for(auto it{it_target}; it != begin;){ //仅用于判断是否存在sdf值超过预设值
        --it;
        if(it->w() > 0.5){
            safe_state = false;
            debug_tools::Debug().print("sdf value > 0.5");
            break;
        }
    }
    // target.z() += 4;
    return true;  
}

void NavLocal::updateMap(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &cloud_in, 
        const Eigen::Vector3d &odom_p, const Eigen::Matrix3d& odom_q, double update_t){
    map_->updateMap(cloud_in, odom_p);
    lidar_p_ = map_->getLidarOdom();
    robot_.pos = odom_q * l_cfg_.offset_robot_p + lidar_p_;
    robot_.mat =  odom_q * l_cfg_.offset_robot_m;
    robot_.ind = ((odom_q * l_cfg_.offset_robot_p - nav_cfg_.bound_min)/nav_cfg_.resolution).cast<int>();
    vel_estimator_->update(update_t, odom_p); //估计速度
    auto robot_speed = vel_estimator_->getVelocity();
    robot_.spd = Eigen::Vector2d(robot_speed.x(), robot_speed.y()); 
    controller_->updateOdom(robot_.pos, robot_.mat, robot_.spd);
    // debug_tools::Debug().print("robot_.spd.norm():", robot_.spd.norm());
}

std::vector<Eigen::Vector2d> NavLocal::getReferLine(const Eigen::Vector3d& p_end){
    return interpolatePoints(Eigen::Vector2d(robot_.pos.x(), robot_.pos.y()), 
            Eigen::Vector2d(p_end.x(), p_end.y()), nav_cfg_.resolution);
}

bool NavLocal::selectPosFromLine(
        std::vector<Eigen::Vector2d>& line, std::vector<Eigen::Vector3d>& safe_p){
    Eigen::Vector3i safe_start_i;
    if(!map_->findSafeClosed(robot_.ind, nav_cfg_.safe_radius, nav_cfg_.safe_cost, safe_start_i, 
            [&](int a, int b){return a<b; })){
        debug_tools::Debug().print("failed in findSafeClosed() start");
        return false;
    }
    auto safe_start_p = map_->getGridCubeCenterIgnoreAvailable(safe_start_i) + lidar_p_;
    safe_p.push_back(safe_start_p);
    if(line.empty()) return false;

    Eigen::Vector3d target{0, 0, 0};
    if(!selectTargetFromReferedLine(safe_start_p, line.begin(), line.end(), target)){
        debug_tools::Debug().print("failed in selectTargetFromReferedLine()");
        return false;
    }
    safe_p.push_back(target);
    safe_i_.main.start = safe_start_i;
    safe_i_.end = map_->getGridIndexIgnoreAvailable(target-lidar_p_);
    return true;
}

bool NavLocal::selectPosFromGPath(std::vector<Eigen::Vector3d>& safe_p, bool& safe_state){
    Eigen::Vector3i safe_start_i;
    if(!map_->findSafeClosed(robot_.ind, nav_cfg_.safe_radius, nav_cfg_.safe_cost, safe_start_i, 
            [&](int a, int b){return a<b; })){
        debug_tools::Debug().print("failed in findSafeClosed() start");
        return false;
    }
    safe_p.clear();
    auto safe_start_p = map_->getGridCubeCenterIgnoreAvailable(safe_start_i) + lidar_p_;
    safe_p.push_back(safe_start_p);

    path_from_g_.lock();
    auto path_g = path_from_g_.getData();
    path_from_g_.unlock();
    if(path_g.empty()) return false;

    Eigen::Vector3d target{0, 0, 0};
    if(!selectTargetFromReferedPath(safe_start_p, path_g.begin(), path_g.end(), target, safe_state)){
        debug_tools::Debug().print("failed in selectTargetFromReferedLine()");
        return false;
    }
    safe_p.push_back(target);
    safe_i_.main.start = safe_start_i;
    safe_i_.end = map_->getGridIndexIgnoreAvailable(target-lidar_p_);
    return true;
}

bool NavLocal::seachPath(std::vector<Eigen::Vector3d>& path){
    std::vector<Eigen::Vector3i> path_i;
    if(!searcher_->plan(safe_i_.main.start, safe_i_.end, path_i)) return false;
    debug_tools::Debug().print("searcher_ plan success");
    std::vector<Eigen::Vector3d> path_res;
    for(auto p_i : path_i) path_res.push_back(map_->getGridCubeCenterIgnoreAvailable(p_i) + lidar_p_);
    path = interpolate(path_res, nav_cfg_.resolution); //输出的原始只有关键点，故均匀插入点 
    controller_->updatePathL(path);
    return true;
}

void NavLocal::setDefaultParams(){
    nav_cfg_.bound_min = Eigen::Vector3d{-2.5, -2.5, -1};
    nav_cfg_.bound_max = Eigen::Vector3d{2.5, 2.5, 0.1};
    nav_cfg_.resolution = 0.05;
    nav_cfg_.safe_cost = 0.3;
    nav_cfg_.safe_radius = 3;

    l_cfg_.alti_cfg.trun_radius = 0.4;
    l_cfg_.alti_cfg.down_spread_dis = 2;

    l_cfg_.offset_robot_p = Eigen::Vector3d{0, 0, 0};
    l_cfg_.offset_robot_m = eulerToQuaternion(0, 0, 0).matrix();

    l_cfg_.p_cfg.map_size_d = Eigen::Vector3d(7, 7, 2);

    l_cfg_.sec_cfg.v_cost = 30.0;
    l_cfg_.sec_cfg.dead_cost = 0.8;
    l_cfg_.sec_cfg.num_limit = 20000;
    l_cfg_.sec_cfg.z_error_thres = 10;

    l_cfg_.ctl_cfg.traj_ahead_num = 5;
    l_cfg_.ctl_cfg.max_accel_add = 3.0;
    l_cfg_.ctl_cfg.max_accel_del = 12.0;
    l_cfg_.ctl_cfg.max_speed = 2.0;
    l_cfg_.ctl_cfg.speed_diff_rate = 2.39;
    l_cfg_.ctl_cfg.speed_yaw_diff_rate = 0.8;
    l_cfg_.ctl_cfg.control_interval_ms = 10;
    l_cfg_.ctl_cfg.stop_dis = 0.3;
    l_cfg_.ctl_cfg.stop_yaw = 0.15;
}

void NavLocal::init(){
    map_ = std::make_shared<altitude_map::LocalMap>();
    altitude_map::AltitudeMap::Ptr map_ptr = map_;
    searcher_ = std::make_shared<path_searcher::ThetaStarExplored>(map_ptr, l_cfg_.sec_cfg.v_cost, l_cfg_.sec_cfg.num_limit);
    vel_estimator_ = std::make_shared<util::VelocityEstimator>();
    controller_ = std::make_shared<motion::Controller>();

    map_->setAltitudeMapParams(l_cfg_.alti_cfg.trun_radius, l_cfg_.alti_cfg.down_spread_dis);
    map_->setProbMapParams(l_cfg_.p_cfg.map_size_d, nav_cfg_.resolution);
 if (nav_cfg_.bound_max.x() <= nav_cfg_.bound_min.x() ||
    nav_cfg_.bound_max.y() <= nav_cfg_.bound_min.y() ||
    nav_cfg_.bound_max.z() <=nav_cfg_.bound_min.z()) {
    std::cerr << "Error: max boundary must be greater than min boundary!" << std::endl;
    return;  // 或者抛出异常或终止程序
}
    map_->init(nav_cfg_.bound_min, nav_cfg_.bound_max, nav_cfg_.resolution);
    searcher_->setDelThres(l_cfg_.sec_cfg.dead_cost);
    vel_estimator_->initialize(Eigen::Vector3d::Zero(), 0.1, 20.0, 5.0, 0.01);

    auto& ctl_cfg = controller_->trackerConfig(); //引用可以直接修改参数
    ctl_cfg.traj_ahead_num = l_cfg_.ctl_cfg.traj_ahead_num;
    debug_tools::Debug().print("ctl_cfg.traj_ahead_num:", ctl_cfg.traj_ahead_num);
    ctl_cfg.max_accel_add = l_cfg_.ctl_cfg.max_accel_add;
    ctl_cfg.max_accel_del = l_cfg_.ctl_cfg.max_accel_del;
    ctl_cfg.max_speed = l_cfg_.ctl_cfg.max_speed;
    ctl_cfg.speed_diff_rate = l_cfg_.ctl_cfg.speed_diff_rate;
    ctl_cfg.speed_yaw_diff_rate = l_cfg_.ctl_cfg.speed_yaw_diff_rate;
    ctl_cfg.control_interval_s = l_cfg_.ctl_cfg.control_interval_ms / 1000.0;
    ctl_cfg.stop_dis = l_cfg_.ctl_cfg.stop_dis;
    ctl_cfg.stop_yaw = l_cfg_.ctl_cfg.stop_yaw;

    robot_.ctl.default_accel_add = ctl_cfg.max_accel_add;
    robot_.ctl.default_accel_del = ctl_cfg.max_accel_del;
    robot_.ctl.default_spd_max = ctl_cfg.max_speed;

    debug_tools::Debug().print("NavLocal::init");
}

} //namespace nav_core
} //namespace path_motion_planning