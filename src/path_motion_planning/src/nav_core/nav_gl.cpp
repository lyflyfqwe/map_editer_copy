#include "path_motion_planning/nav_core/nav_gl.h"

namespace path_motion_planning{
namespace nav_core{
void NavGL::setDefaultParams(){
    NavLocal::setDefaultParams();

    nav_cfg_g_.m_cfg.bound_min = Eigen::Vector3d{-10.0, -10.0, -1};
    nav_cfg_g_.m_cfg.bound_max = Eigen::Vector3d{10.0, 10.0, 1};
    nav_cfg_g_.m_cfg.resolution = 0.05;
    nav_cfg_g_.m_cfg.safe_cost = 0.3;
    nav_cfg_g_.m_cfg.safe_radius = 3;
    nav_cfg_g_.m_cfg.pcd_file = "/home/qwelyflyf/Documents/point_cloud/flower_2.pcd";

    nav_cfg_g_.alti_cfg.trun_radius = 0.4;
    nav_cfg_g_.alti_cfg.down_spread_dis = 2;

    nav_cfg_g_.sec_cfg.v_cost = 30.0;
    nav_cfg_g_.sec_cfg.dead_cost = 0.5;
    nav_cfg_g_.sec_cfg.num_limit = 200000;
}

void NavGL::init(){
    NavLocalTrack::init();

    map_g_ = std::make_shared<altitude_map::GlobalMap>();
    altitude_map::AltitudeMap::Ptr map_g_ptr = map_g_;
    searcher_g_ = std::make_shared<path_searcher::ThetaStar>(
            map_g_ptr, map_, nav_cfg_g_.sec_cfg.v_cost, nav_cfg_g_.sec_cfg.num_limit);
    
    map_g_->setAltitudeMapParams(nav_cfg_g_.alti_cfg.trun_radius, nav_cfg_g_.alti_cfg.down_spread_dis);
    map_g_->init(nav_cfg_g_.m_cfg.bound_min, nav_cfg_g_.m_cfg.bound_max, 
            nav_cfg_g_.m_cfg.resolution, nav_cfg_g_.m_cfg.pcd_file);
    debug_tools::Debug().print("pcd path: ", nav_cfg_g_.m_cfg.pcd_file);

    searcher_g_->setDelThres(nav_cfg_g_.sec_cfg.dead_cost);
}

bool NavGL::selectPosG(const Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& safe_p){
    safe_p.clear();
    if(!map_g_->isInMap(robot_.pos)) return false;
    Eigen::Vector3i start_i = map_g_->getGridIndex(robot_.pos);
    if(!map_g_->isIndexVaild(start_i)) return false;
    if(!map_g_->findSafeClosed(start_i, nav_cfg_g_.m_cfg.safe_radius, nav_cfg_g_.m_cfg.safe_cost, 
        safe_g_i_.main.start, [&](int a, int b){return a<b;})) return false;
    safe_p.push_back(map_g_->getGridCubeCenter(safe_g_i_.main.start));

    if(!map_g_->isInMap(pos)) return false;
    auto end_i = map_g_->getGridIndex(pos);
    if(!map_g_->findSafeClosed(end_i, nav_cfg_g_.m_cfg.safe_radius, nav_cfg_g_.m_cfg.safe_cost,
        safe_g_i_.end, [&](int a, int b){return a<=b; })) return false;
    safe_p.push_back(map_g_->getGridCubeCenter(safe_g_i_.end));
    return true;
}

bool NavGL::searchPathG(std::vector<Eigen::Vector4d>& path, bool is_update_init_yaw){
    std::vector<Eigen::Vector3i> path_i;
    if(!searcher_g_->plan(safe_g_i_.main.start, safe_g_i_.end, path_i)) return false;
    std::vector<Eigen::Vector3d> path_3d;
    for(const auto& i : path_i) path_3d.push_back(map_g_->getGridCubeCenter(i));
    path_3d = interpolate(path_3d, nav_cfg_g_.m_cfg.resolution);
    if(is_update_init_yaw) updatePathG(path_3d);

    path.clear();
    for(const auto& p : path_3d){
        auto index = map_g_->getGridIndex(p);
        path.push_back(Eigen::Vector4d(p.x(), p.y(), p.z(), map_g_->getSDFValue(index)));
    }
    return true;
}

} //namespace nav_core
} //namespace path_motion_planning