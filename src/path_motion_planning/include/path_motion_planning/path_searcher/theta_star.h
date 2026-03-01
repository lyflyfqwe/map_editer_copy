#ifndef PATH_MOTION_PLANNING_PATH_SEARCHER_THETA_STAR_H
#define PATH_MOTION_PLANNING_PATH_SEARCHER_THETA_STAR_H

#include "path_motion_planning/path_searcher/path_searcher.h"

namespace path_motion_planning{
namespace path_searcher{
class ThetaStar: public PathSearcher{
protected:
    bool lineOfSight(const GridNode& parent, const GridNode& child);

    void updateG(const GridNode& parent, GridNode& child);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ThetaStar> Ptr;

    ThetaStar(altitude_map::AltitudeMap::Ptr& map, double w_travel, long long max_cal_num_limit);

    ThetaStar(altitude_map::AltitudeMap::Ptr& map_g, altitude_map::LocalMap::Ptr& map_l, 
        double w_travel, long long max_cal_num_limit);

    bool plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end, 
            std::vector<Eigen::Vector3i>& path) override;
}; //class ThetaStar
} //namespace path_searcher
} //namespace path_motion_planning

#endif