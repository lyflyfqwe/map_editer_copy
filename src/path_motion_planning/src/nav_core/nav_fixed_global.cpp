#include "path_motion_planning/nav_core/nav_fixed_global.h"

namespace path_motion_planning{
namespace nav_core{
void NavFixedGlobal::setDefaultParams(){
    nav_cfg_.bound_min = Eigen::Vector3d{-30, -30, -1}; 
    nav_cfg_.bound_max =Eigen::Vector3d{30, 30, 3};
    nav_cfg_.resolution = 0.05;
    nav_cfg_.safe_cost = 0.9;
    nav_cfg_.safe_radius = 3;

    g_cfg_.search_v_cost = 50.0;
    g_cfg_.search_num_limit = 200000;
}

//nav核心传入点云文件
void NavFixedGlobal::init(std::string pcd_file){
    map_ = std::make_shared<altitude_map::FixedGlobalMap>();
    //海拔地图指针
    altitude_map::AltitudeMap::Ptr map_ptr = map_;
    //两份地图，一份地图转为点云，一份地图作为纯数据给寻路
    //寻路模块 传入海拔地图及其参数


    searcher_ = std::make_shared<path_searcher::ThetaStar>(map_ptr, g_cfg_.search_v_cost, g_cfg_.search_num_limit);
        if (nav_cfg_.bound_max.x() <= nav_cfg_.bound_min.x() ||
    nav_cfg_.bound_max.y() <= nav_cfg_.bound_min.y() ||
    nav_cfg_.bound_max.z() <=nav_cfg_.bound_min.z()) {
    std::cerr << "Error: max boundary must be greater than min boundary!" << std::endl;
    return;  // 或者抛出异常或终止程序
}
    std::cout << "NavFixedGlobal::size:" << pcd_file.size() << std::endl;
    std::cout << "bound_min: " << nav_cfg_.bound_min.transpose() << std::endl;
    std::cout << "bound_max: " << nav_cfg_.bound_max.transpose() << std::endl;
    map_->init(nav_cfg_.bound_min, nav_cfg_.bound_max, nav_cfg_.resolution, pcd_file);

    searcher_->setDelThres(nav_cfg_.safe_cost);
}


void NavFixedGlobal::showPoint2Map(const Eigen::Vector3d& pos){
    if(map_->isInMap(pos)){
        auto index = map_->getGridIndex(pos);
        if (map_->isIndexVaild(index)){
            debug_tools::Debug().print("当前坐标：",index.x(),index.y(),index.z());
        }
    }
}

//添加单点
void NavFixedGlobal::addPoint2Map(const Eigen::Vector3d& pos_add){
    if(map_->isInMap(pos_add)){
        auto index_add = map_->getGridIndex(pos_add);
        if (map_->isIndexVaild(index_add)){
            map_->addPoint(index_add);
        }
    }
}

//区域添加
void NavFixedGlobal::addCubePoint2Map(const std::pair<Eigen::Vector3d,Eigen::Vector3d>& cube, std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>>& cubes){
    if (map_->isInMap(cube.first) && map_->isInMap(cube.second)){
        std::pair<Eigen::Vector3i,Eigen::Vector3i> index_cube,index_cube_fix;
        index_cube.first = map_->getGridIndex(cube.first);
        index_cube.second = map_->getGridIndex(cube.second);

        index_cube_fix.first.x() = (index_cube.first.x() < index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.first.y() = (index_cube.first.y() < index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.first.z() = (index_cube.first.z() < index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        index_cube_fix.second.x() = (index_cube.first.x() > index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.second.y() = (index_cube.first.y() > index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.second.z() = (index_cube.first.z() > index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        debug_tools::Debug().print("Cube :",index_cube.first.x(),index_cube.first.y(),index_cube.first.z(),index_cube.second.x(),index_cube.second.y(),index_cube.second.z());
        // debug_tools::Debug().print("Cube 2 :",index_cube.second.x(),index_cube.second.y(),index_cube.second.z());

        if(map_->isIndexVaild(index_cube_fix.first) && map_->isIndexVaild(index_cube_fix.second)){
            cubes.push_back(index_cube_fix);
            //三目操作符，从当前坐标轴较小的点遍历到最大的点
            for(int x = index_cube_fix.first.x();x <= index_cube_fix.second.x();x++){
                for(int y = index_cube_fix.first.y();y <= index_cube_fix.second.y();y++){
                    for(int z = index_cube_fix.first.z();z <= index_cube_fix.second.z();z++){
                        map_->addPoint(x,y,z);
                    }
                }
            }
        }else{
            debug_tools::Debug().print("参数无效");
        }
    }else{
        debug_tools::Debug().print("参数不在地图中");
    }
}

//区域删除
void NavFixedGlobal::delCubePoint2Map(const std::pair<Eigen::Vector3d,Eigen::Vector3d>& cube,std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>>& cubes_del){
    if (map_->isInMap(cube.first) && map_->isInMap(cube.second)){
        std::pair<Eigen::Vector3i,Eigen::Vector3i> index_cube,index_cube_fix;
        index_cube.first = map_->getGridIndex(cube.first);
        index_cube.second = map_->getGridIndex(cube.second);

        index_cube_fix.first.x() = (index_cube.first.x() < index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.first.y() = (index_cube.first.y() < index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.first.z() = (index_cube.first.z() < index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        index_cube_fix.second.x() = (index_cube.first.x() > index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.second.y() = (index_cube.first.y() > index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.second.z() = (index_cube.first.z() > index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        debug_tools::Debug().print("Cube :",index_cube.first.x(),index_cube.first.y(),index_cube.first.z(),index_cube.second.x(),index_cube.second.y(),index_cube.second.z());
        // debug_tools::Debug().print("Cube 2 :",index_cube.second.x(),index_cube.second.y(),index_cube.second.z());

        if(map_->isIndexVaild(index_cube_fix.first) && map_->isIndexVaild(index_cube_fix.second)){
            cubes_del.push_back(index_cube_fix);
            //三目操作符，从当前坐标轴较小的点遍历到最大的点
            for(int x = index_cube_fix.first.x();x <= index_cube_fix.second.x();x++){
                for(int y = index_cube_fix.first.y();y <= index_cube_fix.second.y();y++){
                    for(int z = index_cube_fix.first.z();z <= index_cube_fix.second.z();z++){
                        map_->delPoint(x,y,z);
                    }
                }
            }
        }else{
            debug_tools::Debug().print("参数无效");
        }
    }else{
        debug_tools::Debug().print("参数不在地图中");
    }
}

//删除单点
void NavFixedGlobal::delPoint2Map(const Eigen::Vector3d& pos_del){
    if(map_->isInMap(pos_del)){
        auto index_del = map_->getGridIndex(pos_del);
        //z轴自减取脚下的点
        index_del.z() -= 1;
        if(map_->isIndexVaild(index_del) && map_->isIndexOccupied(index_del)){
            map_->delPoint(index_del);
        }else{
            debug_tools::Debug().print("当前删除失败点为",index_del.x(),index_del.y(),index_del.z());
        }
    }
}

size_t NavFixedGlobal::selectPos(const Eigen::Vector3d& pos, Eigen::Vector3d& p_safe){
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

//寻路 点+路
bool NavFixedGlobal::seachPath(const PosI2& pos_safe, std::vector<Eigen::Vector3d>& path){
    //启动导航时再传入地图
    // altitude_map::AltitudeMap::Ptr map_ptr_ = map_;
    // searcher_ = std::make_shared<path_searcher::ThetaStar>(map_ptr_, g_cfg_.search_v_cost, g_cfg_.search_num_limit);
    // searcher_->setDelThres(nav_cfg_.safe_cost);

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

//寻路
bool NavFixedGlobal::seachPath(std::vector<Eigen::Vector3d>& path){


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