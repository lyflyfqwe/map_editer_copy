#include "altitude_map/local_map.h"

namespace altitude_map{
void LocalMap::init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, double resolution){
      debug_tools::Debug().print("map_bound: [", bound_min.x(), bound_min.y(), bound_min.z(), "], [",
            bound_max.x(), bound_max.y(), bound_max.z(), "]");
    debug_tools::Debug().print("map_size: [", size_map_x_, size_map_y_, size_map_z_, "]");
    AltitudeMap::init(bound_min, bound_max, resolution);
  
    prob_map_->initProbMap();
}

void LocalMap::updateMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, const Eigen::Vector3d &odom){
    debug_tools::Timer t_0;
    prob_map_->updateProbMap(*cloud_in, odom);
    debug_tools::Timer t_1(t_0); t_1.log("---update prob_map", "ms");

    auto bound_index_min = (boundary_min_/resolution_).cast<int>();
    lidar_odom_ = prob_map_->boxSearch(
        odom, map_occupied_, size_map_x_, size_map_y_, size_map_z_, bound_index_min, iter_map_z_);
    lidar_odom_p_ = lidar_odom_.cast<double>()*resolution_;
    normalizeMap(map_occupied_, 20, 2);
    debug_tools::Timer t_2(t_1); t_2.log("---normalize map_occupied", "ms");

    // reSetMap(map_surfel_buffer_1, 0);
    // processSurfel(cloud_down_, alti_cfg_.surfel.seach_radius, alti_cfg_.surfel.search_gradient, map_surfel_buffer_1);
    // retainOnlySurfel(map_occupied_, map_surfel_buffer_1);

    // fillHoles(map_surfel_buffer_1, 4, 8);
    // normalizeMap(map_surfel_buffer_1, 15, 2);
    reSetMap(map_surfel_buffer_2, 0);
    retainOnlySurfel(map_occupied_, map_surfel_buffer_2);
    flushMap(map_surfel_buffer_2, map_surfel_);
    debug_tools::Timer t_3(t_2); t_3.log("---process surfel", "ms");

    calAltitude(map_a_);
    altitudeToObs(map_a_, 1.5, 0.8, map_tsdf_buffer_);
    downSpread(map_tsdf_buffer_, alti_cfg_.sdf.down_spread_dis);
    calTSDF(map_tsdf_buffer_, alti_cfg_.sdf.trun_radius);
    flushMap(map_tsdf_buffer_, map_tsdf_);
    debug_tools::Timer t_4(t_3); t_4.log("---cal TSDF", "ms");
    debug_tools::Timer t_end(t_0); t_end.log("time update local-map", "ms");
}

Eigen::Vector3d LocalMap::getLidarOdom(){
    return lidar_odom_p_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getMapOccupied(){
    return voxel2Cloud<pcl::PointXYZ, int>(map_occupied_, lidar_odom_.cast<double>()*resolution_, [&](int value){
        return value != 0;
    });
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getMapSurfel(){
    return voxel2Cloud<pcl::PointXYZ, int>(map_surfel_, lidar_odom_.cast<double>()*resolution_, [&](int value){
        return value != 0;
    });
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getMapObs(){
    return voxel2Cloud<pcl::PointXYZ, double>(
        map_tsdf_, lidar_odom_.cast<double>()*resolution_, [&](double value){
            return value >= 1;
    });
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LocalMap::getMapTSDF(bool show_surfel){
    auto getTSDF = [&](int*** voxel_s, double*** voxel_t){
        auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::PointXYZI p;
        Eigen::Vector3d p_center;
        for(int z{0}; z < size_map_z_; z++){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(voxel_s[z][y][x] == 0) continue;
                    p_center = getGridCubeCenter(x, y, z);
                    p.x = p_center.x() + lidar_odom_p_.x();
                    p.y = p_center.y() + lidar_odom_p_.y();
                    p.z = p_center.z() + lidar_odom_p_.z();
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

double LocalMap::getSDF2Global(const Eigen::Vector3i& index_g){
    auto index_l = index_g - lidar_odom_;
    if(!isIndexVaild(index_l)) return 0; //超出local-map的范围，返回0
    return getSDFValue(index_l);
}

double LocalMap::getSDF2GlobalFromPos(const Eigen::Vector3d& pos_g){
    auto pos_l = pos_g - lidar_odom_p_;
    if(!isInMap(pos_l)) return 0;
    return getSDFValue(getGridIndex(pos_l));
}

void LocalMap::setProbMapParams(const Eigen::Vector3d& size_map_d, double resolution){
    prob_map_->setProbMapParams(size_map_d, resolution);
    debug_tools::Debug().print("prob-map size: [", size_map_d.x(), size_map_d.y(), size_map_d.z(), "]");
    debug_tools::Debug().print("prob-map resolution: ", resolution);
}
}//namespace altitude_map