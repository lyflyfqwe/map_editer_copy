#include "altitude_map/global_map.h"
#include <pcl/io/pcd_io.h>

namespace altitude_map{

bool GlobalMap::loadCloudFromPcd(const std::string pcd_path, 
        const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, 
        Eigen::Vector3d& cloud_bound_min, Eigen::Vector3d& cloud_bound_max){
    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud_in) == -1){
        debug_tools::Debug().print("[PathPlanning] loadGlobalMap: load pcd failed...");
        return false;
    }
    auto max_double = std::numeric_limits<double>::max();
    cloud_bound_min = Eigen::Vector3d(max_double, max_double, max_double);
    cloud_bound_max = Eigen::Vector3d(-max_double, -max_double, -max_double);

    cloud_g_->clear();
    for(const auto& p : cloud_in->points){
        if(p.x < bound_min.x() || p.x >= bound_max.x()) continue;
        if(p.y < bound_min.y() || p.y >= bound_max.y()) continue;
        if(p.z < bound_min.z() || p.z >= bound_max.z()) continue;

        if(p.x < cloud_bound_min.x()) cloud_bound_min.x() = p.x;
        if(p.y < cloud_bound_min.y()) cloud_bound_min.y() = p.y;
        if(p.z < cloud_bound_min.z()) cloud_bound_min.z() = p.z;

        if(p.x > cloud_bound_max.x()) cloud_bound_max.x() = p.x;
        if(p.y > cloud_bound_max.y()) cloud_bound_max.y() = p.y;
        if(p.z > cloud_bound_max.z()) cloud_bound_max.z() = p.z;

        cloud_g_->push_back(p);
    }
    return true;
}

void GlobalMap::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in){
    debug_tools::Timer t_0;
    downSampleCloud(cloud_in, cloud_down_, 0.01);
    cloud2Voxel<pcl::PointXYZ, int>(cloud_down_, map_occupied_, 5);
    debug_tools::Timer t_1(t_0); t_1.log("---cloud_in -> map_occupied_", "ms");
    
    // normalizeMap(map_occupied_, 30, 1);
    calAltitude(map_a_);
    altitudeToObs(map_a_, 1.5, 0.8, map_tsdf_);
    downSpread(map_tsdf_, alti_cfg_.sdf.down_spread_dis);
    calTSDF(map_tsdf_, alti_cfg_.sdf.trun_radius);
    debug_tools::Timer t_2(t_1); t_2.log("---cal TSDF", "ms");

    retainOnlySurfel(map_occupied_, map_surfel_);
    // fillHoles(map_surfel_, 4, 8);
    debug_tools::Timer t_3(t_2); t_3.log("---process surfel", "ms");
    debug_tools::Timer t_end(t_0); t_end.log("time update global-map", "ms");
}

void GlobalMap::init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, 
        double resolution, std::string pcd_path){
    Eigen::Vector3d pcd_bound_min, pcd_bound_max;
    cloud_g_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    loadCloudFromPcd(pcd_path, bound_min, bound_max, pcd_bound_min, pcd_bound_max);
      debug_tools::Debug().print("GlobalMap_bound: [", bound_min.x(), bound_min.y(), bound_min.z(), "], [",
            bound_max.x(), bound_max.y(), bound_max.z(), "]");
    AltitudeMap::init(pcd_bound_min, pcd_bound_max, resolution);
    updateMap(cloud_g_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalMap::getMapOccupied(){
    return voxel2Cloud<pcl::PointXYZ, int>(map_occupied_, [&](int value){
        return value != 0;
    });
}
pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalMap::getMapSurfel(){
    return voxel2Cloud<pcl::PointXYZ, int>(map_surfel_, [&](int value){
        return value != 0;
    });
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalMap::getMapObs(){
    return voxel2Cloud<pcl::PointXYZ, double>(map_tsdf_, [&](double value){
        return value >= 1;
    });
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalMap::getMapTSDF(bool show_surfel){
    auto getTSDF = [&](int*** voxel_s, double*** voxel_t){
        auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::PointXYZI p;
        Eigen::Vector3d p_center;
        for(int z{0}; z < size_map_z_; z++){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(voxel_s[z][y][x] == 0) continue;
                    p_center = getGridCubeCenter(x, y, z);
                    p.x = p_center.x();
                    p.y = p_center.y();
                    p.z = p_center.z();
                    p.intensity = voxel_t[z][y][x];
                    cloud_out->push_back(p);
                }
            }
        }
        return cloud_out;
    };
    if(show_surfel) return getTSDF(map_surfel_, map_tsdf_);
    return getTSDF(map_occupied_, map_tsdf_);
}
} //namespace altitude_map