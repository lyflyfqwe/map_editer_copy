#ifndef ALTITUDE_MAP_GRID_MAP_H
#define ALTITUDE_MAP_GRID_MAP_H

#include <memory>
#include <execution>
#include <Eigen/Eigen>

namespace altitude_map{
class GridMap{
private:
    bool is_init_iter_{false}, is_init_map_occupied_{false};

protected:
    double resolution_;
    Eigen::Vector3d boundary_min_, boundary_max_;
    int size_map_x_, size_map_y_, size_map_z_;
    int *iter_map_x_, *iter_map_y_, *iter_map_z_;
    int ***map_occupied_;

    template<typename T> void newMapPtr(T**** map){
        *map = new T**[size_map_z_];
        for(int z{0}; z < size_map_z_; z++){
            (*map)[z] = new T*[size_map_y_];
            for(int y{0}; y < size_map_y_; y++) (*map)[z][y] = new T[size_map_x_];
        }
    }

    template<typename T> void delMapPtr(T**** map){
        if(*map){
            for(int z{0}; z < size_map_z_; z++){
                for(int y{0}; y < size_map_y_; y++){
                    if((*map)[z][y]) delete[] (*map)[z][y];
                }
            }
            for(int z{0}; z < size_map_z_; z++)
                if((*map)[z]) delete[] (*map)[z];
            delete[] *map;
        }
    }

    template<typename T> void reSetMap(T*** map, T value){
        std::for_each(std::execution::par, iter_map_z_, iter_map_z_+size_map_z_, [&](int z){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++) map[z][y][x] = value;
            }
        });
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<GridMap> Ptr;

    virtual ~GridMap();

    void init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, double resolution);

    bool isInMap(Eigen::Vector3d pos);

    bool isInMap(double x, double y);

    bool isIndexVaild(Eigen::Vector3i index);

    bool isIndexVaild(int x, int y, int z);

    bool isIndexVaild(int x, int y);

    /**
     * return index within [-1, size+1)
     */
    bool isIndexVaildWithBound(Eigen::Vector3i index);

    bool isIndexOccupied(Eigen::Vector3i index);

    bool isIndexOccupied(int x, int y, int z);

    /**
     * @brief 获取点坐标对应的索引,如果点坐标位于map外，可能返回负数索引
     */
    Eigen::Vector3i getGridIndexIgnoreAvailable(Eigen::Vector3d pos);

    /**
     * @brief 获取点坐标对应的索引,如点坐标在map外，返回Eigen::Vector3i(0, 0, 0)
     */
    Eigen::Vector3i getGridIndex(Eigen::Vector3d pos);

    Eigen::Vector3d getGridCubeCenterIgnoreAvailable(int x, int y, int z);

    Eigen::Vector3d getGridCubeCenterIgnoreAvailable(Eigen::Vector3i index);

    Eigen::Vector3d getGridCubeCenter(int x, int y, int z);

    Eigen::Vector3d getGridCubeCenter(Eigen::Vector3i index);

    bool isCoordOccupied(Eigen::Vector3d pos);
};
}//namespace altitude_map

#endif