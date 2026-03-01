#ifndef ALTITUDE_MAP_GLOBAL_MAP_H
#define ALTITUDE_MAP_GLOBAL_MAP_H

#include "altitude_map/altitude_map.h"
#include "debug_tools/debug_tools.h"
#include <fstream>

namespace altitude_map{
class FixedGlobalMap: public AltitudeMap{
private:
    //原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g_;
    //修改后需要输出的原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_g_out_;
    // 原始点云添加点储存数组
    std::vector<Eigen::Vector3d> point_s_;
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
    typedef std::shared_ptr<FixedGlobalMap> Ptr;

    FixedGlobalMap(): AltitudeMap(false){}

    void init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, 
            double resolution, std::string pcd_path);

    //添加单点
    void addPoint(const Eigen::Vector3i& pos);
    void addPoint(int x,int y,int z);
    void addCubePoint2Map(const std::pair<Eigen::Vector3i,Eigen::Vector3i> cube);

    //删除单点
    void delPoint(const Eigen::Vector3i& pos);
    void delPoint(int x,int y,int z);
    void delCubePoint2Map(const std::pair<Eigen::Vector3i,Eigen::Vector3i> cube);
    void saveCloudToPcd(
        const std::vector<std::pair<Eigen::Vector3i,Eigen::Vector3i>>& Cubes, 
        const std::vector<std::pair<Eigen::Vector3i,Eigen::Vector3i>>& Cubes_del);

    std::vector<Eigen::Vector3d> densifyWithNoise(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double num_points);

    void delCloudCube(const Eigen::Vector3i& p1, const Eigen::Vector3i& p2, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2del);

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapOccupied();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapSurfel();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapObs();

    pcl::PointCloud<pcl::PointXYZI>::Ptr getMapTSDF(bool show_surfel=false);
};
} //namespace altitude_map

#endif