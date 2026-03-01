#ifndef ALTITUDE_MAP_LOCAL_MAP_H
#define ALTITUDE_MAP_LOCAL_MAP_H

#include "altitude_map/altitude_map.h"
#include "altitude_map/prob/prob_map.h"
#include "debug_tools/debug_tools.h"

namespace altitude_map{
class LocalMap: public AltitudeMap{
private:
    ProbMap::Ptr prob_map_;
    Eigen::Vector3i lidar_odom_; //lidar global odom‘s index
    Eigen::Vector3d lidar_odom_p_; //lidar global odom

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<LocalMap> Ptr;

    LocalMap(): AltitudeMap(true){
        prob_map_ = std::make_shared<ProbMap>();
    }

    void init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, double resolution);

    void updateMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, const Eigen::Vector3d &odom);

    Eigen::Vector3d getLidarOdom();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapOccupied();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapSurfel();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapObs();

    pcl::PointCloud<pcl::PointXYZI>::Ptr getMapTSDF(bool show_surfel=false);

    /**
     * @brief 输入全局地图的索引，获取对应位置的，局部地图的sdf值
     */
    double getSDF2Global(const Eigen::Vector3i& index_g);

    double getSDF2GlobalFromPos(const Eigen::Vector3d& pos_g);

    /**
     * @brief 设置prob-map的大小,地图尺寸必须大于局部地图 
     */
    void setProbMapParams(const Eigen::Vector3d& size_map_d, double resolution);
};
}//namespace altitude_map

#endif