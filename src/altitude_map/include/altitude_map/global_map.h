#ifndef ALTITUDE_MAP_GLOBAL_MAP_H
#define ALTITUDE_MAP_GLOBAL_MAP_H

#include "altitude_map/altitude_map.h"
#include "debug_tools/debug_tools.h"

namespace altitude_map{
class GlobalMap: public AltitudeMap{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g_;

    /**
     * @param bound_min, bound_max 点云限制的最大尺寸
     * @param cloud_bound_min, cloud_bound_max 引用返回点云的实际尺寸 
     */
    bool loadCloudFromPcd(const std::string pcd_path, 
        const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, 
        Eigen::Vector3d& cloud_bound_min, Eigen::Vector3d& cloud_bound_max);

    void updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<GlobalMap> Ptr;

    GlobalMap(): AltitudeMap(false){}

    void init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, 
            double resolution, std::string pcd_path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapOccupied();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapSurfel();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapObs();

    pcl::PointCloud<pcl::PointXYZI>::Ptr getMapTSDF(bool show_surfel=false);
};
} //namespace altitude_map

#endif