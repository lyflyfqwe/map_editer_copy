#ifndef PATH_MOTION_PLANNING_PATH_SEARCHER_THETA_STAR_EXPLORED_H
#define PATH_MOTION_PLANNING_PATH_SEARCHER_THETA_STAR_EXPLORED_H

#include "path_motion_planning/path_searcher/path_searcher.h"

namespace path_motion_planning{
namespace path_searcher{
class ThetaStarExplored: public PathSearcher{
private:
    bool lineOfSight(const GridNode& parent, const GridNode& child);

    void updateG(const GridNode& parent, GridNode& child);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ThetaStarExplored> Ptr;

    ThetaStarExplored(altitude_map::AltitudeMap::Ptr& map, double w_travel, long long max_cal_num_limit);

    bool plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end, 
            std::vector<Eigen::Vector3i>& path) override;
}; //class ThetaStarExplored

} //namespace path_searcher
} //namespace path_motion_planning
#endif