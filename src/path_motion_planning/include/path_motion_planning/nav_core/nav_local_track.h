#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_TRACK_H
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_LOCAL_TRACK_H

#include "path_motion_planning/nav_core/nav_local.h"
#include "path_motion_planning/path_searcher/theta_star_track.h"

namespace path_motion_planning{
namespace nav_core{
class NavLocalTrack: public NavLocal{
private:
    path_searcher::ThetaStarTrack::Ptr searcher_t_;

    std::vector<Eigen::Vector3i> path_3i_; // path'pos based on l-map

    double cur_spd_rate_{2.0}; //安全模式下的档位, 默认2.0

public:
    /**
     * @brief 判断是否存在安全的起点
     */
    bool selectPosFromGPath(std::vector<Eigen::Vector3d>& safe_p, bool& safe_state) override;

    bool seachPath(std::vector<Eigen::Vector3d>& path, bool safe_state);

    void init() override;

    /**
     * @brief 更新控制器的参考路径
     * @param path 局部搜索的路径
     */
    void updatePathLForControl(const std::vector<Eigen::Vector3d>& path);

    /**
     * @brief 设置安全模式下的速度档位
     * @param rate 安全模式下输出的速度将乘于该值
     */
    void setSpeedRate(double rate){
        assert(rate > 0 && "[NavLocalTrack] (setSpeedRate) rate <= 0");
        cur_spd_rate_ = rate;
    }
}; //class NavLocalTrack
} //namespace nav_core
} //namespace path_motion_planning

#endif