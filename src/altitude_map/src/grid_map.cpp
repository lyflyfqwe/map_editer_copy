#include "altitude_map/grid_map.h"
#include <iostream>

namespace altitude_map{
GridMap::~GridMap(){
    std::cout << "~GridMap\n";
    if(is_init_iter_){
        if(iter_map_x_) delete[] iter_map_x_;
        if(iter_map_y_) delete[] iter_map_y_;
        if(iter_map_z_) delete[] iter_map_z_;
    };
    if(is_init_map_occupied_) delMapPtr(&map_occupied_);
}

void GridMap::init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, double resolution){
    if (resolution <= 0) {
        throw std::invalid_argument("Resolution must be greater than 0.");
    }
    
    // 检查边界的合法性
    if (bound_max(0) <= bound_min(0) || bound_max(1) <= bound_min(1) || bound_max(2) <= bound_min(2)) {
            std::cout << "cloud_bound_max: " << bound_max(0) << bound_max(1) << bound_max(2) << std::endl;
    std::cout << "cloud_bound_min: " << bound_min(0) << bound_min(1) << bound_min(2) << std::endl;
        throw std::invalid_argument("Invalid bounds: max boundary must be greater than min boundary.");
    }
    boundary_min_ = bound_min;
    boundary_max_ = bound_max;
    resolution_ = resolution;
    size_map_x_ = std::ceil((boundary_max_(0) - boundary_min_(0)) / resolution_);
    size_map_y_ = std::ceil((boundary_max_(1) - boundary_min_(1)) / resolution_);
    size_map_z_ = std::ceil((boundary_max_(2) - boundary_min_(2)) / resolution_);
    if(!is_init_iter_){
        iter_map_x_ = new int[size_map_x_];
        for(int x{0}; x < size_map_x_; x++) iter_map_x_[x] = x;
        iter_map_y_ = new int[size_map_y_];
        for(int y{0}; y < size_map_y_; y++) iter_map_y_[y] = y;
        iter_map_z_ = new int[size_map_z_];
        for(int z{0}; z < size_map_z_; z++) iter_map_z_[z] = z;
        is_init_iter_ = true; 
    }
    newMapPtr(&map_occupied_);
    reSetMap<int>(map_occupied_, 0);
    is_init_map_occupied_ = true;
}

bool GridMap::isInMap(Eigen::Vector3d pos){
    if(pos(0) < boundary_min_(0) || pos(1) < boundary_min_(1) || pos(2) < boundary_min_(2)) return false;
    if(pos(0) > boundary_max_(0) || pos(1) > boundary_max_(1) || pos(2) > boundary_max_(2)) return false;
    return true;
}

bool GridMap::isInMap(double x, double y){
    if(x < boundary_min_(0) || y < boundary_min_(1)) return false;
    if(x > boundary_max_(0) || y > boundary_max_(1)) return false;
    return true;
}

bool GridMap::isIndexVaild(Eigen::Vector3i index){
    if(index(0) < 0 || index(1) < 0 || index(2) < 0) return false;
    if(index(0) >= size_map_x_ || index(1) >= size_map_y_ || index(2) >= size_map_z_) {
        std::cout << "Map size << " << size_map_x_ << ", " << size_map_y_ << ", " << size_map_z_ << " >>, "
                  << "Index << " << index(0) << ", " << index(1) << ", " << index(2) << " >>\n";
        return false;
    }
    return true;
}

bool GridMap::isIndexVaild(int x, int y, int z){
    if(x < 0 || y < 0 || z < 0) return false;
    if(x >= size_map_x_ || y >= size_map_y_ || z >= size_map_z_) return false;
    return true;
}

bool GridMap::isIndexVaild(int x, int y){
    return x >= 0 && y >= 0 && x < size_map_x_ && y < size_map_y_;
}

bool GridMap::isIndexVaildWithBound(Eigen::Vector3i index){
    return index.x() >= -1 && index.y() >= -1 && index.z() >= -1
        && index.x() < size_map_x_+1 && index.y() < size_map_y_+1 && index.z() < size_map_z_+1;
}

bool GridMap::isIndexOccupied(Eigen::Vector3i index){
    if(!isIndexVaild(index)) return true;
    if(map_occupied_[index(2)][index(1)][index(0)] == 0) return false;
    return true;
}

bool GridMap::isIndexOccupied(int x, int y, int z){
    if(!isIndexVaild(x, y, z)) return true;
    if(map_occupied_[z][y][x] == 0) return false;
    return true;
}

Eigen::Vector3i GridMap::getGridIndexIgnoreAvailable(Eigen::Vector3d pos){
    int index_x = std::floor( (pos(0) - boundary_min_(0)) / resolution_);
    int index_y = std::floor( (pos(1) - boundary_min_(1)) / resolution_);
    int index_z = std::floor( (pos(2) - boundary_min_(2)) / resolution_);
    return Eigen::Vector3i(index_x, index_y, index_z);
}

Eigen::Vector3i GridMap::getGridIndex(Eigen::Vector3d pos){
    if(!isInMap(pos)) return Eigen::Vector3i(0, 0, 0);
    return getGridIndexIgnoreAvailable(pos);
}

Eigen::Vector3d GridMap::getGridCubeCenterIgnoreAvailable(int x, int y, int z){
    double dx = (x + 0.5) * resolution_;
    double dy = (y + 0.5) * resolution_;
    double dz = (z + 0.5) * resolution_;

    return Eigen::Vector3d(dx, dy, dz) + boundary_min_;
}

Eigen::Vector3d GridMap::getGridCubeCenterIgnoreAvailable(Eigen::Vector3i index){
    return getGridCubeCenterIgnoreAvailable(index(0), index(1), index(2));
}

Eigen::Vector3d GridMap::getGridCubeCenter(int x, int y, int z){
    if(!isIndexVaild(x, y, z)) return Eigen::Vector3d(0, 0, 0);
    return getGridCubeCenterIgnoreAvailable(x, y, z);
}

Eigen::Vector3d GridMap::getGridCubeCenter(Eigen::Vector3i index){
    return getGridCubeCenter(index(0), index(1), index(2));
}

bool GridMap::isCoordOccupied(Eigen::Vector3d pos){
    if(!isInMap(pos)) return true;
    return isIndexOccupied(getGridIndex(pos));
}

}//namespace altitude_map