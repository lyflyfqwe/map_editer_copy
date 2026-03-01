#ifndef PATH_MOTION_PLANNING_NAV_CORE_NAV_CORE_H
#define PATH_MOTION_PLANNING_NAV_CORE_NAV_CORE_H

#include <condition_variable>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <path_motion_planning/path_searcher/path_searcher.h>

namespace path_motion_planning{
namespace nav_core{

template<typename T> class PathData{
private:
    std::vector<T> data_;
    std::condition_variable cv_;
    mutable std::mutex mutex_;
    bool is_working_{false};   

public:
    void lock(bool state){
        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_working_ = state;
        }
        cv_.notify_all();
    }

    void setData(std::vector<T>& data){
        std::unique_lock<std::mutex> lock(mutex_);  
        cv_.wait(lock, [this]{ return !is_working_; });
        data_ = data;
    }

    std::vector<T> getData(){
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }

    void lock(){lock(true);}

    void unlock(){lock(false);}
}; //class PathData

struct PosI2{
    Eigen::Vector3i end{};

    struct Temp{
        bool is_set{false};
        Eigen::Vector3i start{};
    }main, tmp;

    PosI2(){}

    PosI2(const Eigen::Vector3i& s, const Eigen::Vector3i& s_temp, const Eigen::Vector3i& e){
        main.is_set = false;
        main.start = s;
        tmp.is_set = false;
        tmp.start = s_temp;
        end = e;
    }
}; //struct PosI2

class NavCore{
protected:
    path_searcher::PathSearcher::Ptr searcher_;

    /**
     * @brief 将欧拉角转换为四元数 注意：Eigen 使用弧度而非角度
     */
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);

    bool insideBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const Eigen::Vector3d& pos);

    bool insideBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const Eigen::Vector2d& pos);

    /**
     * @brief 找到路径中在距离robot_pos距离最近的点
     */
    std::vector<Eigen::Vector3d>::iterator findClosedPosOfPath(std::vector<Eigen::Vector3d>::iterator begin, 
        std::vector<Eigen::Vector3d>::iterator end, const Eigen::Vector3d& robot_pos);

    /**
     * @brief 输入离散点集，按照间隔interval均匀插点，返回插值后的点集
     */
    std::vector<Eigen::Vector3d> interpolate(const std::vector<Eigen::Vector3d>& points, double interval); 

public:
    struct NavCoreConfig{
        Eigen::Vector3d bound_min{}, bound_max{};
        double resolution{};
        int safe_radius{};
        int safe_radius_find_l_start{10}; //从全局路径中查找点的最远阈值
        double safe_cost{};
    }nav_cfg_;

    virtual ~NavCore() = default;
}; //class NavCore

} //namespace nav_core
} //namespace path_motion_planning

#endif