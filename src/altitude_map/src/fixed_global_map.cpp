#include "altitude_map/fixed_global_map.h"
#include <pcl/io/pcd_io.h>
#include <random>

namespace altitude_map{

bool FixedGlobalMap::loadCloudFromPcd(const std::string pcd_path, 
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

    double min_x = 1e9, min_y = 1e9, min_z = 1e9;
    double max_x = -1e9, max_y = -1e9, max_z = -1e9;
    for (const auto& p : cloud_in->points) {
        min_x = std::min(min_x, (double)p.x);
        min_y = std::min(min_y, (double)p.y);
        min_z = std::min(min_z, (double)p.z);
        max_x = std::max(max_x, (double)p.x);
        max_y = std::max(max_y, (double)p.y);
        max_z = std::max(max_z, (double)p.z);
    }
    double span_x = max_x - min_x;
    double span_y = max_y - min_y;
    double span_z = max_z - min_z;

    // 自动检测单位
    double scale = 1.0;  // 默认：米
    if (span_x > 1000.0 || span_y > 1000.0) {
        scale = 0.001;  // 原图是毫米 → 转米
        std::cout << "[MapLoader] Detected mm-scale point cloud, converting to meters..." << std::endl;
    } else if (span_x < 0.1 && span_y < 0.1) {
        scale = 1000.0; // 原图是米制但太小，可按需调整（通常不需要）
    }

    // 应用缩放（统一到米）
    for (auto& p : cloud_in->points) {
        p.x *= scale;
        p.y *= scale;
        p.z *= scale;
    }

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
       std::cout << "cloud_bound_min: " << cloud_bound_min.transpose() << std::endl;
    std::cout << "cloud_bound_max: " << cloud_bound_max.transpose() << std::endl;
    std::cout << "cloud size: " << cloud_in->size() << std::endl;
        return true;
}

//更新地图
void FixedGlobalMap::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in){


    debug_tools::Timer t_0;
    //传入点云
    //降采样
    downSampleCloud(cloud_in, cloud_down_, 0.01);
    //点云转网格，
    cloud2Voxel<pcl::PointXYZ, int>(cloud_down_, map_occupied_, 5);
    debug_tools::Timer t_1(t_0); t_1.log("---cloud_in -> map_occupied_", "ms");
    
    //障碍物处理 
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


void FixedGlobalMap::saveCloudToPcd(const std::vector<std::pair<Eigen::Vector3i,Eigen::Vector3i>>& Cubes, 
    const std::vector<std::pair<Eigen::Vector3i,Eigen::Vector3i>>& Cubes_del){

    for(auto &p : Cubes){
        addCubePoint2Map(p);
    }

    // 原始点云添加点
    for(auto &p : point_s_){
        pcl::PointXYZ point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        cloud_g_out_->push_back(point);
    }
    for(auto &p : Cubes_del){
        delCubePoint2Map(p);
        delCloudCube(p.first,p.second,cloud_g_out_);
    }
    pcl::io::savePCDFile("cloud_out_1.pcd",*cloud_g_out_);

    }


//手动加单点
void FixedGlobalMap::addPoint(const Eigen::Vector3i& pos){
    map_occupied_[pos.z()][pos.y()][pos.x()] = 1;
}

void FixedGlobalMap::addPoint(int x,int y,int z){
    map_occupied_[z][y][x] = 1;
}

void FixedGlobalMap::delPoint(const Eigen::Vector3i& pos){
    //z轴-1修正坐标点
    map_occupied_[pos.z()][pos.y()][pos.x()] = 0;
}

void FixedGlobalMap::delPoint(int x,int y,int z){
    //z轴-1修正坐标点
    map_occupied_[z][y][x] = 0;
}

//手动加点版，占据地图+原始点云同时修改
void FixedGlobalMap::addCubePoint2Map(const std::pair<Eigen::Vector3i,Eigen::Vector3i> cube){
    // if (isInMap(cube.first) && isInMap(cube.second)){
        std::pair<Eigen::Vector3i,Eigen::Vector3i> index_cube,index_cube_fix;
        index_cube.first = cube.first;
        index_cube.second = cube.second;

        index_cube_fix.first.x() = (index_cube.first.x() < index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.first.y() = (index_cube.first.y() < index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.first.z() = (index_cube.first.z() < index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        index_cube_fix.second.x() = (index_cube.first.x() > index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.second.y() = (index_cube.first.y() > index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.second.z() = (index_cube.first.z() > index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        // debug_tools::Debug().print("Cube 1 :",index_cube.first.x(),index_cube.first.y(),index_cube.first.z());
        // debug_tools::Debug().print("Cube 2 :",index_cube.second.x(),index_cube.second.y(),index_cube.second.z());

        if(isIndexVaild(index_cube_fix.first) && isIndexVaild(index_cube_fix.second)){
            //三目操作符，从当前坐标轴较小的点遍历到最大的点
            for(int x = index_cube_fix.first.x();x <= index_cube_fix.second.x();x+=10){
                for(int y = index_cube_fix.first.y();y <= index_cube_fix.second.y();y+=10){
                    for(int z = index_cube_fix.first.z();z <= index_cube_fix.second.z();z+=10){
                        addPoint(x,y,z);

                        // auto pos_ = getGridCubeCenter(x,y,z);
                        // point_s_.push_back(pos_);
                        // debug_tools::Debug().print("point:",pos_.x(),pos_.y(),pos_.z());

                    }
                }
            }
            //原始点云点添加  densifyWithNoise为填充对角线内的点云 参数:1点，2点，每单位轴多少点   注意是每单位轴，如果转换成每单位多少个则需要^3
            auto point_temp = densifyWithNoise(getGridCubeCenter(index_cube_fix.first),getGridCubeCenter(index_cube_fix.second),4);
            for(auto& p : point_temp){
                //point_s_为点云添加队列
                point_s_.push_back(p);
            }
            debug_tools::Debug().print("Add Cube :",index_cube.first.x(),index_cube.first.y(),index_cube.first.z(),index_cube.second.x(),index_cube.second.y(),index_cube.second.z(),"points :",point_temp.size());
            
        }else{
            debug_tools::Debug().print("参数无效");
        }
}

void FixedGlobalMap::delCubePoint2Map(const std::pair<Eigen::Vector3i,Eigen::Vector3i> cube){
    // if (isInMap(cube.first) && isInMap(cube.second)){
        std::pair<Eigen::Vector3i,Eigen::Vector3i> index_cube,index_cube_fix;
        index_cube.first = cube.first;
        index_cube.second = cube.second;

        index_cube_fix.first.x() = (index_cube.first.x() < index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.first.y() = (index_cube.first.y() < index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.first.z() = (index_cube.first.z() < index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        index_cube_fix.second.x() = (index_cube.first.x() > index_cube.second.x()) ? index_cube.first.x() : index_cube.second.x();
        index_cube_fix.second.y() = (index_cube.first.y() > index_cube.second.y()) ? index_cube.first.y() : index_cube.second.y();
        index_cube_fix.second.z() = (index_cube.first.z() > index_cube.second.z()) ? index_cube.first.z() : index_cube.second.z();

        debug_tools::Debug().print("Del Cube :",index_cube.first.x(),index_cube.first.y(),index_cube.first.z(),index_cube.second.x(),index_cube.second.y(),index_cube.second.z());

        if(isIndexVaild(index_cube_fix.first) && isIndexVaild(index_cube_fix.second)){
            //三目操作符，从当前坐标轴较小的点遍历到最大的点
            for(int x = index_cube_fix.first.x();x <= index_cube_fix.second.x();x++){
                for(int y = index_cube_fix.first.y();y <= index_cube_fix.second.y();y++){
                    for(int z = index_cube_fix.first.z();z <= index_cube_fix.second.z();z++){
                        delPoint(x,y,z);
                    }
                }
            }
        }else{
            debug_tools::Debug().print("参数无效");
        }
}

//填充原始点云
std::vector<Eigen::Vector3d> FixedGlobalMap::densifyWithNoise(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double num_points){
    std::vector<Eigen::Vector3d> points;
    
    // 计算轴对齐包围盒
    const double min_x = std::min(p1.x(), p2.x());
    const double max_x = std::max(p1.x(), p2.x());
    const double min_y = std::min(p1.y(), p2.y());
    const double max_y = std::max(p1.y(), p2.y());
    const double min_z = std::min(p1.z(), p2.z());
    const double max_z = std::max(p1.z(), p2.z());

    if (num_points <= 0) return points;

    double bate = 1.0 / num_points;
    double dd = resolution_ * bate;
    for(double x = min_x; x <= max_x; x += dd){
        for(double y = min_y; y <= max_y; y += dd){
            for(double z = min_z; z <= max_z; z += dd){
                points.push_back(Eigen::Vector3d(x,y,z));
            }
        }
    }

    return points;
}

//区域删除原始点云
void FixedGlobalMap::delCloudCube(const Eigen::Vector3i& p1, const Eigen::Vector3i& p2, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2del){
    Eigen::Vector3d p1_d(getGridCubeCenter(p1));
    Eigen::Vector3d p2_d(getGridCubeCenter(p2));
    double min_x = std::min(p1_d.x(),p2_d.x());
    double max_x = std::max(p1_d.x(),p2_d.x());
    double min_y = std::min(p1_d.y(),p2_d.y());
    double max_y = std::max(p1_d.y(),p2_d.y());
    double min_z = std::min(p1_d.z(),p2_d.z());
    double max_z = std::max(p1_d.z(),p2_d.z());

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(auto &p : *cloud2del){
        if(p.x > min_x && p.x < max_x &&
           p.y > min_y && p.y < max_y &&
           p.z > min_z && p.z < max_z){
            // null
        }else{
            cloud.push_back(p);
        }
    }
    *cloud2del = cloud;
}

//点云变网格地图
void FixedGlobalMap::init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, 
        double resolution, std::string pcd_path){
    Eigen::Vector3d pcd_bound_min, pcd_bound_max;
    cloud_g_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cloud_g_out_ = cloud_g_;

    //传入点云文件及限制参数，网格限制，实际坐标限制，更新cloud_g_
    if(loadCloudFromPcd(pcd_path, bound_min, bound_max, pcd_bound_min, pcd_bound_max) == false)return;
debug_tools::Debug().print("FixedGlobalMap_bound: [", bound_min.x(), bound_min.y(), bound_min.z(), "], [",
            bound_max.x(), bound_max.y(), bound_max.z(), "]");
            debug_tools::Debug().print("FixedGlobalMap_bound: [", pcd_bound_min.x(), pcd_bound_min.y(), pcd_bound_min.z(), "], [",
            pcd_bound_max.x(), pcd_bound_max.y(), pcd_bound_max.z(), "]");
    if (bound_max.x() <= bound_min.x() ||
    bound_max.y() <= bound_min.y() ||
    bound_max.z() <=bound_min.z()) {
    std::cerr << "Error: max boundary must be greater than min boundary!" << std::endl;
    return;  // 或者抛出异常或终止程序
}

    //海拔地图初始化，传入实际坐标点
    AltitudeMap::init(pcd_bound_min, pcd_bound_max, resolution);
    //传入cloud_g_，进行过滤处理
    updateMap(cloud_g_);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr FixedGlobalMap::getMapOccupied(){
    //网格转点云
    return voxel2Cloud<pcl::PointXYZ, int>(map_occupied_, [&](int value){
        return value != 0;
    });
}
pcl::PointCloud<pcl::PointXYZ>::Ptr FixedGlobalMap::getMapSurfel(){
    return voxel2Cloud<pcl::PointXYZ, int>(map_surfel_, [&](int value){
        return value != 0;
    });
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FixedGlobalMap::getMapObs(){
    return voxel2Cloud<pcl::PointXYZ, double>(map_tsdf_, [&](double value){
        return value >= 1;
    });
}

pcl::PointCloud<pcl::PointXYZI>::Ptr FixedGlobalMap::getMapTSDF(bool show_surfel){
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