#ifndef ALTITUDE_MAP_PROB_MAP_H
#define ALTITUDE_MAP_PROB_MAP_H

#include <queue>
#include <mutex>
#include <execution>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "altitude_map/prob/prob_config.hpp"
#include "altitude_map/prob/sliding_map.h"
#include "altitude_map/prob/raycaster.h"

namespace altitude_map{
class ProbMap: public SlidingMap{
private:
    bool init_once{false};

protected:
    std::vector<float> occupancy_buffer_;
    bool map_empty_{true};

    struct RaycastData{
        raycaster::RayCaster raycaster;
        std::queue<Eigen::Vector3i> update_cache_id_g;
        std::vector<uint16_t> operation_cnt;
        std::vector<uint16_t> hit_cnt;
        Eigen::Vector3d cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
        int batch_update_counter{0};
        std::mutex raycast_range_mtx;
    } raycast_data_;

    ProbConfig prob_cfg_;

    // standardization query
    // Known free < l_free
    // occupied >= l_occ
    bool isKnownFree(const double &prob) const{
        return prob < prob_cfg_.l_free;
    }
    bool isOccupied(const double &prob) const{
        return prob >= prob_cfg_.l_occ;
    }
    bool isUnknown(const double &prob) const{
        return prob >= prob_cfg_.l_free && prob < prob_cfg_.l_occ;
    }

    void resetCell(const int &hash_id) override;

    void resetLocalMap() override;

    bool isOccupied(const Eigen::Vector3i &id_g) const;

    bool isUnknown(const Eigen::Vector3i &id_g) const;

    bool isKnownFree(const Eigen::Vector3i &id_g) const;

    void hitPointUpdate(const int &hash_id, const int &hit_num);

    void missPointUpdate(const int &hash_id, const int &hit_num);

    void probabilisticMapFromCache();

    void insertUpdateCandidate(const Eigen::Vector3i &id_g, bool is_hit);

    void raycastProcess(const pcl::PointCloud<pcl::PointXYZI> &input_cloud, const Eigen::Vector3d &cur_odom);

    void updateLocalBox(const Eigen::Vector3d &cur_odom);

    Eigen::Vector3d lineBoxIntersectPoint(const Eigen::Vector3d& pt, const Eigen::Vector3d& pos,
            const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ProbMap> Ptr;

    ProbMap() = default;
    ~ProbMap() override = default;

    Eigen::Vector3d getLocalMapOrigin() const { return local_map_origin_d_; }

    Eigen::Vector3d getLocalMapSize() const { return prob_cfg_.map_size_d; }

    double getResolution() const{ return sli_cfg_.resolution; }
    
    //这里会初始化 prob_cfg_.initForTest();
    void initProbMap();

    bool isOccupied(const Eigen::Vector3d &pos) const;

    bool isUnknown(const Eigen::Vector3d &pos) const;

    bool isKnownFree(const Eigen::Vector3d &pos) const;

    GridType getGridType(Eigen::Vector3i &id_g) const;

    GridType getGridType(const Eigen::Vector3d &pos) const;

    double getMapValue(const Eigen::Vector3d &pos) const;

    void boundBoxByLocalMap(Eigen::Vector3d &box_min, Eigen::Vector3d &box_max) const;

    void boxSearch(const Eigen::Vector3d &_box_min, const Eigen::Vector3d &_box_max,
        const GridType &gt, std::vector<Eigen::Vector3d> &out_points) const;

    Eigen::Vector3i boxSearch(const Eigen::Vector3d& odom_pos, int*** voxel_map, 
        int size_x, int size_y, int size_z, Eigen::Vector3i bound_zr_min, int* iter_z);

    void updateProbMap(const pcl::PointCloud<pcl::PointXYZI> &cloud, const Eigen::Vector3d &pos);

    void setProbMapParams(const Eigen::Vector3d& size_map_d, double resolution){
        prob_cfg_.map_size_d = size_map_d;
        prob_cfg_.resolution = resolution;
    }
};
}//namespace altitude_map

#endif