#ifndef ALTITUDE_MAP_SLIDING_MAP_H
#define ALTITUDE_MAP_SLIDING_MAP_H

#include <memory>
#include <vector>
#include <Eigen/Eigen>

#define SIGN(x) ((x > 0) - (x < 0))

namespace altitude_map{

enum GridType {
    UNDEFINED = 0,
    UNKNOWN = 1,
    OUT_OF_MAP,
    OCCUPIED,
    KNOWN_FREE
};

class SlidingMap{
private:
    bool has_been_initialized_{false};

protected:
    struct SlidingMapConfig{
        double resolution{0};
        double resolution_inv{0};
        double sliding_thresh{0};
        bool map_sliding_en{false};
        Eigen::Vector3d fix_map_origin{};
        Eigen::Vector3i visualization_range_i{};
        Eigen::Vector3i map_size_i{};
        Eigen::Vector3i half_map_size_i{};
        int virtual_ceil_height_id_g{0};
        int virtual_ground_height_id_g{0};
        int safe_margin_i{0};
        int map_vox_num{0};
    }sli_cfg_;

    Eigen::Vector3d local_map_origin_d_, local_map_bound_min_d_, local_map_bound_max_d_;
    Eigen::Vector3i local_map_origin_i_, local_map_bound_min_i_, local_map_bound_max_i_;

    virtual void resetLocalMap() = 0;
    virtual void resetCell(const int & hash_id) = 0;

    int getLocalIndexHash(const Eigen::Vector3i &id_in) const;

    void clearMemoryOutOfMap(const std::vector<int> &clear_id, const int &i);

    void posToGlobalIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) const;

    void posToGlobalIndex(const double &pos, int &id) const;

    //全局索引转全局坐标
    void globalIndexToPos(const Eigen::Vector3i &id_g, Eigen::Vector3d &pos) const;

    void globalIndexToLocalIndex(const Eigen::Vector3i &id_g, Eigen::Vector3i &id_l) const;

    void localIndexToGlobalIndex(const Eigen::Vector3i &id_l, Eigen::Vector3i &id_g) const;

    //本地索引转全局坐标
    void localIndexToPos(const Eigen::Vector3i &id_l, Eigen::Vector3d &pos) const;

    void hashIdToLocalIndex(const int &hash_id, Eigen::Vector3i &id) const;

    void hashIdToPos(const int &hash_id, Eigen::Vector3d &pos) const;

    void hashIdToGlobalIndex(const int &hash_id, Eigen::Vector3i &id_g) const;

    int getHashIndexFromPos(const Eigen::Vector3d &pos) const;

    int getHashIndexFromGlobalIndex(const Eigen::Vector3i &id_g) const;

    void updateLocalMapOriginAndBound(const Eigen::Vector3d &new_origin_d, const Eigen::Vector3i &new_origin_i);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SlidingMap() = default;
    virtual ~SlidingMap() = default;

    void initSlidingMap(const Eigen::Vector3i &half_map_size_i, const double &resolution,
        const bool &map_sliding_en, const double &sliding_thresh, const Eigen::Vector3d &fix_map_origin);

    SlidingMap(const Eigen::Vector3i &half_map_size_i, const double &resolution,
        const bool &map_sliding_en, const double &sliding_thresh, const Eigen::Vector3d &fix_map_origin);

    void mapSliding(const Eigen::Vector3d &odom);

    bool insideLocalMap(const Eigen::Vector3d &pos) const;

    bool insideLocalMap(const Eigen::Vector3i &id_g) const;
};
}// namespace altitude_map

#endif