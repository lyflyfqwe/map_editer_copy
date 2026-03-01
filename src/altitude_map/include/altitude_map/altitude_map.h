#ifndef ALTITUDE_MAP_ALTITUDE_MAP_H
#define ALTITUDE_MAP_ALTITUDE_MAP_H

#include <limits>
#include <algorithm>
#include <execution>
#include <type_traits> 
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/impl/io.hpp>

#include "altitude_map/grid_map.h"

namespace altitude_map{
namespace util{
struct Vector3iLess{  
    bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const{  
        if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();  
        if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();  
        return lhs.z() < rhs.z();  
    }  
}; 
} // namespace util

class AltitudeMap: public GridMap{
private:
    bool is_init_maps_{false}, is_local_map_{false};

    std::vector<std::pair<int, int>> getCoveredCells(int R);

protected:
    int ***map_surfel_, ***map_surfel_buffer_1, ***map_surfel_buffer_2;
    double ***map_a_, ***map_tsdf_, ***map_tsdf_buffer_; 
    std::mutex mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down_;

    /**
    * @param fill_thres 八个方向存在多少个占据，则置1
    * @param clear_thres 八个方向存在多少个为空，则置0  
    */
    template<typename T> void fillHoles(T*** voxel, double fill_thres, double clear_thres){
        auto neibours = {std::make_pair<int, int>(0, 1), std::make_pair<int, int>(0, -1), std::make_pair<int, int>(1, 0), std::make_pair<int, int>(-1, 0),
                std::make_pair<int, int>(1, 1), std::make_pair<int, int>(1, -1), std::make_pair<int, int>(-1, 1), std::make_pair<int, int>(-1, -1)};
        int neibour_size = static_cast<int>(neibours.size());
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T** value_z){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(value_z[y][x] <= 0 || value_z[y][x] == neibour_size+1) continue;
                    int empty_num{0};
                    for(auto [dx, dy] : neibours){
                        if(empty_num >= clear_thres){
                            value_z[y][x] = neibour_size+1;
                            break;
                        }
                        int nx = x + dx, ny = y + dy;
                        if(isIndexVaild(nx, ny) && value_z[ny][nx] <= 0) empty_num++;
                    }
                }
            }
        });
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T** value_z){
            std::for_each(std::execution::par, value_z, value_z+size_map_y_, [&](T* value_y){
                for(int x{0}; x < size_map_x_; x++){
                    if(value_y[x] != neibour_size+1) continue;
                    value_y[x] = 0;
                }
            });
        });   
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T** value_z){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(value_z[y][x] <= 0) continue;
                    for(auto [dx, dy] : neibours){
                        int nx = x + dx, ny = y + dy;
                        if(isIndexVaild(nx, ny) && value_z[ny][nx] <= 0) value_z[ny][nx] -= 1;
                    }
                }
            }
        });
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T** value_z){
            std::for_each(std::execution::par, value_z, value_z+size_map_y_, [&](T* value_y){
                for(int x{0}; x < size_map_x_; x++){
                    if(value_y[x] > 0) continue;
                    value_y[x] = value_y[x] <= -fill_thres? 1 : 0;
                }
            });
        });
    }

    /**
     * @brief 仅针对tsdf-map, 处理voxel中voxel.value >= 1的体素， 以下down_dis范围内的voxel将被赋值1
     */
    template<typename T> void downSpread(T*** voxel, int down_dis){
        if(down_dis >= size_map_z_-1) return; 
        for(int z{down_dis}; z < size_map_z_; z++){
            std::for_each(std::execution::par, iter_map_y_, iter_map_y_+size_map_y_, [&](int y){
                for(int x{0}; x < size_map_x_; x++){
                    if(voxel[z][y][x] < 1) continue;
                    for(int k{z-1}; k >= z-down_dis; k--) 
                        if(voxel[k][y][x] < 1) voxel[k][y][x] = 1;
                }
            });
        }
    }

    template<typename T> void flushMap(T*** voxel_in, T*** voxel_out){
        std::for_each(std::execution::par, iter_map_z_, iter_map_z_+size_map_z_, [&](int z){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if constexpr (std::is_same<T, int>::value) if(voxel_in[z][y][x] != voxel_out[z][y][x]) voxel_out[z][y][x] = voxel_in[z][y][x];
                    if constexpr (std::is_same<T, double>::value) if(std::abs(voxel_in[z][y][x] - voxel_out[z][y][x]) > 0.01) voxel_out[z][y][x] = voxel_in[z][y][x];
                }
            }
        });
    }

    template<typename T> inline void splitFromState(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
            std::vector<int>& indices, std::vector<bool>& state, T*** map_surf){
        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i){
            Eigen::Vector3i index = getGridIndex(Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
            if(!isIndexVaild(index)) return;
            if(state[i]) map_surf[index(2)][index(1)][index(0)] = 1;
        });
    }

    template<typename T> void normalizeMap(T*** voxel, double fill_thres, int R){
        auto neibours = getCoveredCells(R);
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T** value_z){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(value_z[y][x] <= 0) continue;
                    for(auto [dx, dy] : neibours){
                        int ny{y+dy}, nx{x+dx};
                        if(!isIndexVaild(nx, ny) || value_z[ny][nx] > 0 || value_z[ny][nx] <= -fill_thres) continue;
                        value_z[ny][nx] -= 1;
                    }
                }
            }
        });
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T** value_z){
            std::for_each(std::execution::par, value_z, value_z+size_map_y_, [&](T* value_y){
                for(int x{0}; x < size_map_x_; x++){
                    if(value_y[x] > 0) continue;
                    if(value_y[x] <= -fill_thres) value_y[x] = 1; 
                }
            });
        });   
    }

    template<typename T1, typename T2> void retainOnlySurfel(T1*** voxel_in, T2*** voxel_out){
        std::for_each(std::execution::par, iter_map_z_+1, iter_map_z_+size_map_z_, [&](int z){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    //逐一遍历，如果当前格为0，且脚底下一格不为空，则将脚底下那一格判断为1，即地板
                    if(voxel_in[z][y][x] == 0 && voxel_in[z-1][y][x] != 0) voxel_out[z-1][y][x] = 1; 
                }
            }
        });
    }

    template<typename T1, typename T2> void cloud2Voxel(typename pcl::PointCloud<T1>::Ptr& cloud_in, T2*** voxel, int density_thres){

        //将点云坐标转化成地图索引格
        std::for_each(std::execution::par, cloud_in->begin(), cloud_in->end(), [&](const T1& p){
            Eigen::Vector3i index = getGridIndex(Eigen::Vector3d(p.x, p.y, p.z));
            //判断是否有效，如果有效则将占据地图对应点赋值为1
            if(isIndexVaild(index)) voxel[index(2)][index(1)][index(0)]++;
        });

        
        std::for_each(std::execution::par, voxel, voxel+size_map_z_, [&](T2** value_z){
            std::for_each(std::execution::par, value_z, value_z+size_map_y_, [&](T2* value_y){
                for(int x{0}; x < size_map_x_; x++)
                    value_y[x] = value_y[x] >= density_thres? 1 : 0;
            });
        });
    }

    template<typename T1=pcl::PointXYZI, typename T2=double, typename T3> typename pcl::PointCloud<T1>::Ptr voxel2Cloud(T2*** voxel, T3 Func){
        auto cloud_out = std::make_shared<pcl::PointCloud<T1>>();
        T1 p;
        Eigen::Vector3d p_center;
        for(int z{0}; z < size_map_z_; z++){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(!Func(voxel[z][y][x])) continue;
                    p_center = getGridCubeCenter(x, y, z);
                    p.x = p_center.x();
                    p.y = p_center.y();
                    p.z = p_center.z();
                    if constexpr (std::is_same<T1, pcl::PointXYZI>::value) p.intensity = voxel[z][y][x];
                    cloud_out->push_back(p);
                }
            }
        }
        return cloud_out;
    }

    template<typename T1=pcl::PointXYZI, typename T2=double, typename T3> typename pcl::PointCloud<T1>::Ptr voxel2Cloud(T2*** voxel, Eigen::Vector3d odom_tf, T3 Func){
        auto cloud_out = std::make_shared<pcl::PointCloud<T1>>();
        T1 p;
        Eigen::Vector3d p_center;
        for(int z{0}; z < size_map_z_; z++){
            for(int y{0}; y < size_map_y_; y++){
                for(int x{0}; x < size_map_x_; x++){
                    if(!Func(voxel[z][y][x])) continue;
                    p_center = getGridCubeCenter(x, y, z);
                    p.x = p_center.x() + odom_tf.x();
                    p.y = p_center.y() + odom_tf.y();
                    p.z = p_center.z() + odom_tf.z();
                    if constexpr (std::is_same<T1, pcl::PointXYZI>::value) p.intensity = voxel[z][y][x];
                    cloud_out->push_back(p);
                }
            }
        }
        return cloud_out;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZ2XYZI(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, Eigen::Vector3d odom_tf=Eigen::Vector3d::Zero()){
        auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_out->reserve(cloud_in->size());
        for(auto& p : cloud_in->points){
            pcl::PointXYZI p_i;
            p_i.x = p.x + odom_tf.x();
            p_i.y = p.y + odom_tf.y();
            p_i.z = p.z + odom_tf.z();
            cloud_out->push_back(p_i);
        }
        return cloud_out;
    }

    void preHandleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, 
        Eigen::Vector3d& bound_min, Eigen::Vector3d& bound_max);

    void downSampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float resolution);

    void processSurfel(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_in, double radius, double angle, int*** map_surf);

    void calAltitude(double*** voxel);

    // 计算方差
    double calculateVariance(const std::vector<double>& data);
    
    /**
     * @brief 从altitude中提取差异，将voxel_o中对应的点的值设置为1
     * @param voxel_i altitude-map 
     * @param obs_thres 满足垂直差异大于obs_thres,才允许被视为障碍点
     * @param variance 邻域四个点的方差大于variance, 才允许被视为障碍点
     * @param voxel_o 输出的，只包含障碍点的tsdf-map
     */
    void altitudeToObs(double ***voxel_i, double obs_thres, double variance, double ***voxel_o);

    void calTSDF(double*** voxel, double truncation_dis);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<AltitudeMap> Ptr;

    struct AltitudeMapConfig{
        struct Surfel{
            double seach_radius{0.15};
            double search_gradient = 50/180.0*M_PI; //倾斜角度 弧度制
        }surfel; //表面搜索相关参数 （）已废弃
        struct SDF{
            double trun_radius{0.4}; // tsdf截断距离（类似于膨胀半径）默认0.4m
            int down_spread_dis{2}; // 障碍物向下传播的距离 默认2格
        }sdf;
    } alti_cfg_;

    AltitudeMap(bool is_local){ is_local_map_ = is_local; }

    virtual ~AltitudeMap();

    std::vector<std::pair<std::string, double>> 
    checkMemoryMB(Eigen::Vector3d& bound_min, Eigen::Vector3d& bound_max, double resolution);

    void init(const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max, double resolution);

    std::tuple<int, int, int> getMapSize();

    //if index unavailable return false; else return map_surfel == 1;
    bool canBeNeighbour(Eigen::Vector3i index);
    
    //if index unavailable return 1; else return map_tsdf's value
    double getSDFValue(Eigen::Vector3i index);

    /**
     * @param index 查找开始的索引
     * @param radius 查找半径，跨越多少个voxel单位长度
     * @param safe_cost tsdf值小于该值，即意味着安全
     * @param safe_i 查找到的安全位置的索引
     * @param func lambda: 比较搜索过程的 seach_index.z() 和 index.z()
     * @return 查找成功
     */
    bool findSafeClosed(const Eigen::Vector3i& index, int radius, double safe_cost, 
            Eigen::Vector3i& safe_i, std::function<bool(int, int)> func);

    /**
     * @brief 搜索索引z轴限定区域内是否存在surfel
     * @param index 搜索起点
     * @param radius 搜索半径
     * @param safe_i 如果存在，则引用返回
     */
    bool findSafeClosed(const Eigen::Vector3i& index, int radius, Eigen::Vector3i& safe_i);

    /**
     * @param voxel_o 判断的依据
     * @param voxel_t tsdf来源
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getTSDF(int*** voxel_o, double*** voxel_t);

    /**
     * @brief 设置altitude-map参数
     * @param trun_radius tsdf截断距离（类似于膨胀半径）
     * @param down_spread_dis 障碍物向下传播的距离 默认是2格
     */
    void setAltitudeMapParams(double trun_radius=0.4, int down_spread_dis=2);
};

}// namespace altitude_map

#endif