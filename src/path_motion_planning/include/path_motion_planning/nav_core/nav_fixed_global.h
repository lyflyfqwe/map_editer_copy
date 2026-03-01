#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_GLOBAL_H
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_GLOBAL_H

#include <stack>

#include "path_motion_planning/nav_core/nav_core.h"
#include "path_motion_planning/path_searcher/theta_star.h"
#include "altitude_map/fixed_global_map.h"

namespace path_motion_planning{
namespace nav_core{
class NavFixedGlobal: public NavCore{
private:
    altitude_map::FixedGlobalMap::Ptr map_;
    std::stack<Eigen::Vector3i> p_safe_;

public:
    altitude_map::FixedGlobalMap::Ptr getMapPtr() {
    return map_;
}

    typedef std::shared_ptr<NavFixedGlobal> Ptr;

    struct GConfig{
        double search_v_cost{30};
        int search_num_limit{100000};
    }g_cfg_;

    /**
     * @brief 设置默认参数，仅用于测试
     */
    void setDefaultParams();

    void init(std::string pcd_file);

    size_t selectPos(const Eigen::Vector3d& pos, Eigen::Vector3d& p_safe);

    bool seachPath(const PosI2& pos_safe, std::vector<Eigen::Vector3d>& path);

    bool seachPath(std::vector<Eigen::Vector3d>& path);

    //显示坐标
    void showPoint2Map(const Eigen::Vector3d& pos);

    //添加单点
    void addPoint2Map(const Eigen::Vector3d& pos_add);

    //区域添加
    void addCubePoint2Map(const std::pair<Eigen::Vector3d,Eigen::Vector3d>& cube, 
        std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>>& cubes);

    //区域删除
    void delCubePoint2Map(const std::pair<Eigen::Vector3d,Eigen::Vector3d>& cube,
        std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>>& cubes_del);

    //删除单点
    void delPoint2Map(const Eigen::Vector3d& pos_del);

    

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMap(){
        return map_->getMapOccupied();
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr getMap(){
    //     return map_->getMapTSDF();
    // }

}; //class NavLocal
} //namespace nav_core
} //namespace path_motion_planning

#endif