#ifndef PATH_MOTION_PLANNING_PATH_SEARCHER_THETA_STAR_TRACK_H
#define PATH_MOTION_PLANNING_PATH_SEARCHER_THETA_STAR_TRACK_H

#include "path_motion_planning/path_searcher/theta_star.h"

namespace path_motion_planning{
namespace path_searcher{
class Vec3iMap{
private:
    struct Vec3iHash{
        std::size_t operator()(const Eigen::Vector3i& v) const {  
            return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y());
        }
    }; //struct Vec3iHash

    struct Vec3iValue{
        int value, z;

        Vec3iValue(){}

        Vec3iValue(int v, int z){
            this->value = v; this->z = z;
        }
    }; //struct Vec3iValue

    std::unordered_map<Eigen::Vector3i, Vec3iValue, Vec3iHash> map_; 

public:
    void insert(const Eigen::Vector3i& key, int value){
        map_[key] = Vec3iValue(value, key.z());
    }

    bool get(const Eigen::Vector3i& key, int& value, int& z) const{
        auto it = map_.find(key);  
        if(it != map_.end()) {  
            auto vec3i_value = it->second;
            value = vec3i_value.value;
            z = vec3i_value.z;
            return true;  
        }  
        return false; // 如果未找到键  
    }

    bool contains(const Eigen::Vector3i& key) const{  
        return map_.find(key) != map_.end();  
    }

    size_t size(){
        return map_.size();
    }
}; //class Vec3iMap

class ThetaStarTrack: public ThetaStar{
private:
    int error_z_thres_{0};

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<ThetaStarTrack> Ptr;

    ThetaStarTrack(altitude_map::AltitudeMap::Ptr& map, double w_travel, long long max_cal_num_limit);

    bool plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end,
            const std::vector<Eigen::Vector3i>& path_in, std::vector<Eigen::Vector3i>& path_out);

    void setErrorZThres(int z){
        error_z_thres_ = z;
    }

}; //class ThetaStarTrack
} //namespace path_searcher
} //namespace path_motion_planning

#endif