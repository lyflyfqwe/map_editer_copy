#include "path_motion_planning/nav_core/nav_global.h"

namespace path_motion_planning{
namespace nav_core{
void NavGlobal::setDefaultParams(){
    nav_cfg_.bound_min = Eigen::Vector3d{-15, -15, -1}; 
    nav_cfg_.bound_max =Eigen::Vector3d{15, 15, 1};
    nav_cfg_.resolution = 0.05;
    nav_cfg_.safe_cost = 0.9;
    nav_cfg_.safe_radius = 3;

    g_cfg_.search_v_cost = 50.0;
    g_cfg_.search_num_limit = 200000;
}

void NavGlobal::init(std::string pcd_file){
    map_ = std::make_shared<altitude_map::GlobalMap>();
    altitude_map::AltitudeMap::Ptr map_ptr = map_;
    searcher_ = std::make_shared<path_searcher::ThetaStar>(map_ptr, g_cfg_.search_v_cost, g_cfg_.search_num_limit);
 if (nav_cfg_.bound_max.x() <= nav_cfg_.bound_min.x() ||
    nav_cfg_.bound_max.y() <= nav_cfg_.bound_min.y() ||
    nav_cfg_.bound_max.z() <=nav_cfg_.bound_min.z()) {
    std::cerr << "Error: max boundary must be greater than min boundary!" << std::endl;
    return;  // 或者抛出异常或终止程序
}
    map_->init(nav_cfg_.bound_min, nav_cfg_.bound_max, nav_cfg_.resolution, pcd_file);
    searcher_->setDelThres(nav_cfg_.safe_cost);
}

size_t NavGlobal::selectPos(const Eigen::Vector3d& pos, Eigen::Vector3d& p_safe){
    auto tuple_size = p_safe_.size();
    if(!map_->isInMap(pos)) return tuple_size;
    auto pos_i = map_->getGridIndex(pos);
    if(!map_->isIndexVaild(pos_i)) return tuple_size;

    Eigen::Vector3i safe_i;
    if(!map_->findSafeClosed(pos_i, nav_cfg_.safe_radius, nav_cfg_.safe_cost, safe_i, 
            [&](int a, int b){return tuple_size == 0? a < b : true;}))  return tuple_size;
    p_safe_.push(safe_i);
    p_safe = map_->getGridCubeCenter(safe_i);
    return p_safe_.size();
}

bool NavGlobal::seachPath(const PosI2& pos_safe, std::vector<Eigen::Vector3d>& path){
    std::vector<Eigen::Vector3i> path_i;
    if(!searcher_->plan(pos_safe.main.start, pos_safe.end, path_i)){
        debug_tools::Debug().print("global search path failed...");
        return false;
    }
    path.clear();
    for(const auto& p : path_i){
        auto pos = map_->getGridCubeCenter(p);
        path.push_back(pos);
    }
    return true;
}

bool NavGlobal::seachPath(std::vector<Eigen::Vector3d>& path){
    if(p_safe_.size() != 2) return false;
    auto end_i = p_safe_.top();
    p_safe_.pop();
    auto start_i = p_safe_.top();
    p_safe_.pop();
    PosI2 pos2(start_i, Eigen::Vector3i::Zero(), end_i);
    return seachPath(pos2, path);
}

} //namespace nav_core
} //namespace path_motion_planning