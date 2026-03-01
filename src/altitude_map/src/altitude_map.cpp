#include "altitude_map/altitude_map.h"
#include <pcl/filters/voxel_grid.h>
#include <iostream>

namespace altitude_map{
std::vector<std::pair<int, int>> AltitudeMap::getCoveredCells(int R){
    std::vector<std::pair<int, int>> coveredCells; // 存储被覆盖的格子坐标  
    // 遍历可能的格子中心坐标  
    for(int x = -R; x <= R; x++) {  
        for(int y = -R; y <= R; y++) {  
            // 计算格子中心是否在圆内  
            if (x * x + y * y <= R * R) coveredCells.emplace_back(x, y); // 如果在圆内，添加到结果  
        }  
    } 
    return coveredCells;  
}

void AltitudeMap::preHandleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, 
        Eigen::Vector3d& bound_min, Eigen::Vector3d& bound_max){
    cloud_out->clear();
    for(auto& p : cloud_in->points){
        if(p.x < bound_min(0) || p.x > bound_max(0)) continue;
        if(p.y < bound_min(1) || p.y > bound_max(1)) continue;
        if(p.z < bound_min(2) || p.z > bound_max(2)) continue;
        cloud_out->push_back(p);
    }
}

void AltitudeMap::downSampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float resolution){
    auto filter = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
    filter->setInputCloud(cloud_in);
    filter->setLeafSize(resolution, resolution, resolution);
    filter->filter(*cloud_out); 
}

void AltitudeMap::processSurfel(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_in, 
        double radius, double angle, int*** map_surf){
    std::vector<bool> state(cloud_in->size(), true);  // 初始化每个点的状态为 true  
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  
    kdtree.setInputCloud(cloud_in);  

    std::vector<int> indices(cloud_in->size());  
    std::iota(indices.begin(), indices.end(), 0);  // 填充索引  

    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i){
        const pcl::PointXYZ& pos = cloud_in->points[i];  
        Eigen::Vector3d c_point(pos.x, pos.y, pos.z);  

        std::vector<int> points_index;  
        std::vector<float> points_radius;  

        if(kdtree.radiusSearch(pos, radius, points_index, points_radius) > 0){
            Eigen::Vector3d n_point;  
            for(size_t j{0}; j < points_index.size(); j++){
                n_point(0) = cloud_in->points[points_index[j]].x;  
                n_point(1) = cloud_in->points[points_index[j]].y;  
                n_point(2) = cloud_in->points[points_index[j]].z;   

                if (n_point(2) > c_point(2)) {  
                    double dis_xy = (n_point.head(2) - c_point.head(2)).norm();  
                    double ang = std::atan2(n_point(2) - c_point(2), dis_xy);   
                    if(ang > angle){  
                        state[i] = false;  
                        break;  
                    }   
                }  
            }  
        }  
    });  
    splitFromState(cloud_in, indices, state, map_surf);
}

void AltitudeMap::calAltitude(double*** voxel){
    std::for_each(std::execution::par, iter_map_y_, iter_map_y_+size_map_y_, [&](int y){
        for(int x{0}; x < size_map_x_; x++){
            bool flag{false};
            int index_scan{0}; int index_surfel{-1}, index_back{0};
            if(isIndexOccupied(x, y, index_scan)){
                flag = true;
                index_back = index_scan;
            }
            while(index_scan < size_map_z_-1){
                if(isIndexOccupied(x, y, index_scan) && !isIndexOccupied(x, y, index_scan+1)){
                    index_surfel = index_scan;
                    if(flag){
                        for(int z{index_back}; z < index_scan; z++) voxel[z][y][x] = index_surfel;
                        flag = false;
                    }
                }
                else if(!isIndexOccupied(x, y, index_scan) && isIndexOccupied(x, y, index_scan+1)){
                    flag = true;
                    index_back = index_scan+1;
                }
                voxel[index_scan++][y][x] = index_surfel;
            }
            voxel[index_scan][y][x] = index_surfel;
            if(flag){
                for(int z{index_back}; z <= index_scan; z++) voxel[z][y][x] = index_scan;
            }
        }
    });
}

double AltitudeMap::calculateVariance(const std::vector<double>& data){
    if(data.empty()) throw std::invalid_argument("vector is empty");
    // 计算平均值  
    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    double variance = 0.0;  
    for (const double& value : data) {  
        variance += (value - mean) * (value - mean);  
    }  
    variance /= data.size();
    return variance;
}

void AltitudeMap::altitudeToObs(double ***voxel_i, double obs_thres, double variance, double ***voxel_o){
    auto absValue =[](double num){ return num>0? num : -num; };
    auto neibours = {std::make_pair<int, int>(-1, 0), std::make_pair<int, int>(1, 0), std::make_pair<int, int>(0, -1),
        std::make_pair<int, int>(0, 1)};
    std::for_each(std::execution::par, iter_map_z_, iter_map_z_+size_map_z_, [&](int z){
        //重置voxel_o边缘的四条边为0
        for(int b_x{0}; b_x < size_map_x_; b_x++){
            voxel_o[z][0][b_x] = 0;
            voxel_o[z][size_map_y_-1][b_x] = 0;
        }
        for(int b_y{0}; b_y < size_map_y_; b_y++){
            voxel_o[z][b_y][0] = 0;
            voxel_o[z][b_y][size_map_x_-1] = 0;
        }
        
        for(int y{1}; y < size_map_y_-1; y++){
            for(int x{1}; x < size_map_x_-1; x++){
                if(isIndexOccupied(x, y, z)){
                    voxel_o[z][y][x] = 0;
                    continue;
                }
                bool flag{true};
                double cost_max_diff{0};
                std::vector<double> costs;
                for(auto [dx, dy] : neibours){
                    double value = voxel_i[z][y+dy][x+dx];
                    if(value < 0){
                        flag = false;
                        break;
                    }
                    double cost = value-voxel_i[z][y][x];
                    costs.push_back(cost);
                    if(absValue(cost_max_diff) < absValue(cost)) cost_max_diff = cost;
                } 
                voxel_o[z][y][x] = flag && calculateVariance(costs) > variance
                        && absValue(cost_max_diff)>obs_thres? 1 : 0; 
            }
        }
    });
}

void AltitudeMap::calTSDF(double*** voxel, double truncation_dis){
    int tur_size = std::ceil(truncation_dis / resolution_);
    std::for_each(std::execution::par, iter_map_z_, iter_map_z_ + size_map_z_, [&](int z){
        int ix, iy;
        for(int y{1}; y < size_map_y_-1; y++){
            for(int x{1}; x < size_map_x_-1; x++){
                if(voxel[z][y][x] < 1) continue;

                for(int dy{-tur_size}; dy <= tur_size; dy++){
                    for(int dx{-tur_size}; dx <= tur_size; dx++){
                        ix = dx + x, iy = dy + y;
                        if(!(dx == 0 && dy == 0) && isIndexVaild(ix, iy, z)){
                            double dis = resolution_ * std::sqrt(dx*dx + dy*dy);
                            voxel[z][iy][ix] = std::max(voxel[z][iy][ix], voxel[z][y][x]*(truncation_dis - dis)/truncation_dis);
                        }
                    }
                }
            }
        }
    });
}

AltitudeMap::~AltitudeMap(){
    if(is_init_maps_){
        delMapPtr(&map_surfel_);
        delMapPtr(&map_surfel_buffer_1);
        delMapPtr(&map_a_);
        delMapPtr(&map_tsdf_);
        if(is_local_map_){
            delMapPtr(&map_surfel_buffer_2);
            delMapPtr(&map_tsdf_buffer_);
        }
    }
    std::cout << "~AltitudeMap\n";
}

std::vector<std::pair<std::string, double>> 
AltitudeMap::checkMemoryMB(Eigen::Vector3d& bound_min, Eigen::Vector3d& bound_max, double resolution){
    int x = std::ceil((bound_max(0) - bound_min(0)) / resolution);
    int y = std::ceil((bound_max(1) - bound_min(1)) / resolution);
    int z = std::ceil((bound_max(2) - bound_min(2)) / resolution);
    double grid_num{x*y*z*1.0}, rate{1024.0*1024.0};

    std::vector<std::pair<std::string, double>> m_list;
    m_list.push_back(std::make_pair("map_occupied", grid_num * sizeof(int) / rate));
    m_list.push_back(std::make_pair("map_altitude", grid_num * sizeof(int) / rate));
    int num_map_surfel = is_local_map_? 3 : 2;
    m_list.push_back(std::make_pair("map_surfel", num_map_surfel*grid_num * sizeof(int) / rate));
    int num_map_tsdf = is_local_map_? 2 : 1;
    m_list.push_back(std::make_pair("map_tsdf", num_map_tsdf*grid_num * sizeof(double) / rate));
    double m_totall{0};
    for(auto m : m_list) m_totall += m.second;
    m_list.push_back(std::make_pair("m totall", m_totall));

    return m_list;
}

//海拔地图初始化
void AltitudeMap::init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, double resolution){
    if(is_init_maps_){
        std::cout << "[AltitudeMap] repeated init map\n";
        return;
    }
    std::cout << "GridMapbound_max: " << bound_max(0) << bound_max(1) << bound_max(2) << std::endl;
    //网格地图初始化
    GridMap::init(bound_min, bound_max, resolution);
    newMapPtr(&map_surfel_);
    newMapPtr(&map_surfel_buffer_1);
    newMapPtr(&map_a_);
    newMapPtr(&map_tsdf_);

    reSetMap<int>(map_surfel_, 0);
    reSetMap<int>(map_surfel_buffer_1, 0);
    reSetMap<double>(map_a_, 0.0);
    reSetMap<double>(map_tsdf_, 0.0);

    if(is_local_map_){
        newMapPtr(&map_surfel_buffer_2);
        newMapPtr(&map_tsdf_buffer_);

        reSetMap<int>(map_surfel_buffer_2, 0);
        reSetMap<double>(map_tsdf_buffer_, 0.0);
    }
    cloud_down_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    is_init_maps_ = true;
}

std::tuple<int, int, int> AltitudeMap::getMapSize(){
    return {size_map_x_, size_map_y_, size_map_z_};
}

bool AltitudeMap::canBeNeighbour(Eigen::Vector3i index){
    if(!isIndexVaild(index)) return false;
    return map_surfel_[index.z()][index.y()][index.x()] == 1;
}

double AltitudeMap::getSDFValue(Eigen::Vector3i index){
    if(!isIndexVaild(index)) return 1;
    return map_tsdf_[index.z()][index.y()][index.x()];
}

bool AltitudeMap::findSafeClosed(const Eigen::Vector3i& index, int radius, double safe_cost, 
        Eigen::Vector3i& safe_i, std::function<bool(int, int)> func){
    bool is_find{false};
    double dis_closed = std::numeric_limits<double>::max();  
    std::queue<Eigen::Vector3i> search_queue;  
    search_queue.push(index); // 从中心开始搜索  
    std::set<Eigen::Vector3i, util::Vector3iLess> visited;  // 记录已访问的索引以防止重复访问  

    for(int dist{0}; dist <= radius; ++dist){
        for(size_t i{0}; i < search_queue.size(); ++i){
            Eigen::Vector3i cur_index = search_queue.front();  
            search_queue.pop();  

            // 检查当前元素是否安全
            if(canBeNeighbour(cur_index) && func(cur_index.z(), index.z()) && getSDFValue(cur_index) < safe_cost){
                double distance = (cur_index - index).norm();   //计算距离  

                if(distance < dis_closed){  
                    dis_closed = distance;  
                    safe_i = cur_index;  
                    is_find = true;
                }
            }

            // 继续搜索该索引附近的元素  
            for(int dx{-1}; dx <= 1; ++dx){  
                for(int dy{-1}; dy <= 1; ++dy){  
                    for(int dz{-1}; dz <= 1; ++dz){  
                        // 忽略 (0, 0, 0) 移动  
                        if(dx == 0 && dy == 0 && dz == 0) continue;  
                        Eigen::Vector3i neighbour_i = cur_index + Eigen::Vector3i(dx, dy, dz);  

                        // 确保在边界内且未访问过  
                        if(visited.find(neighbour_i) == visited.end()){  
                            visited.insert(neighbour_i);  
                            search_queue.push(neighbour_i);  
                        }  
                    }  
                }  
            }
        }
    }
    return is_find;
}

bool AltitudeMap::findSafeClosed(const Eigen::Vector3i& index, int radius, Eigen::Vector3i& safe_i){
    auto func = [&](int z)->bool { return z >= 0 && z < size_map_z_; };
    auto cur_i = index;
    for(int i{0}; i < radius; i++){
        cur_i.z() = index.z() + i;
        if(!func(cur_i.z())) break;
        if(canBeNeighbour(cur_i)){
            safe_i = cur_i;
            return true;
        }
    }
    for(int i{0}; 0 > -radius; i--){
        cur_i.z() = index.z() + i;
        if(!func(cur_i.z())) break;
        if(canBeNeighbour(cur_i)){
            safe_i = cur_i;
            return true;
        }
    }
    return false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr 
AltitudeMap::getTSDF(int*** voxel_o, double*** voxel_t){
    auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointXYZI p;
    Eigen::Vector3d p_center;
    for(int z{1}; z < size_map_z_; z++){
        for(int y{0}; y < size_map_y_; y++){
            for(int x{0}; x < size_map_x_; x++){
                if(voxel_o[z-1][y][x] == 0) continue;
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
}

void AltitudeMap::setAltitudeMapParams(double trun_radius, int down_spread_dis){
    alti_cfg_.sdf.trun_radius = trun_radius;
    alti_cfg_.sdf.down_spread_dis = down_spread_dis;
    std::cout << "[altitude-map] trun_radius: " << alti_cfg_.sdf.trun_radius << " "
            << "down_spread_dis: " << alti_cfg_.sdf.down_spread_dis << "\n";
}

}// namespace altitude_map