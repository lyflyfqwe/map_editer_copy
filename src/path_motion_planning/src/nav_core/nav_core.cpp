#include "path_motion_planning/nav_core/nav_core.h"

namespace path_motion_planning{
namespace nav_core{
Eigen::Quaterniond NavCore::eulerToQuaternion(double roll, double pitch, double yaw){
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());  

    // 合成四元数，顺序为先 yaw、后 pitch、再 roll  
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;  
    return q;  
}

bool NavCore::insideBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const Eigen::Vector3d& pos){
    return pos.x() >= min.x() && pos.y() >= min.y() && pos.z() >= min.z()
        && pos.x() < max.x() && pos.y() < max.y() && pos.z() < max.z();
}

bool NavCore::insideBox(const Eigen::Vector3d& min, const Eigen::Vector3d& max, const Eigen::Vector2d& pos){
    return pos.x() >= min.x() && pos.y() >= min.y() && pos.x() < max.x() && pos.y() < max.y();
}

std::vector<Eigen::Vector3d>::iterator 
NavCore::findClosedPosOfPath(std::vector<Eigen::Vector3d>::iterator begin, 
        std::vector<Eigen::Vector3d>::iterator end, const Eigen::Vector3d& robot_pos){
    auto min_dis = (*begin - robot_pos).norm(); 
    if(begin == end) return end;
    auto closed_iter = begin;
    for(auto it = ++begin; it != end; it++){
        auto dis = (*it - robot_pos).norm();
        if(dis <= min_dis){
            min_dis = dis;
            closed_iter = it;
        }
    }
    return closed_iter;
}

std::vector<Eigen::Vector3d> 
NavCore::interpolate(const std::vector<Eigen::Vector3d>& points, double interval){
    auto distance = [&](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)->double {
        return (p1 - p2).norm();
    };
    std::vector<Eigen::Vector3d> result;  
    // 遍历给定的点  
    for(size_t i{0}; i < points.size()-1; ++i){  
        const Eigen::Vector3d& p1 = points[i];  
        const Eigen::Vector3d& p2 = points[i + 1];  
        
        // 计算两点之间的距离  
        double dist = distance(p1, p2);  
        result.push_back(p1);  // 保存起始点  
        
        // 如果距离大于间隔，则插入中间点  
        if(dist > interval){  
            int numIntervals = static_cast<int>(std::floor(dist / interval)); // 计算需要插入的间隔数  
            // 计算单位方向向量并乘以间隔  
            Eigen::Vector3d direction = (p2 - p1).normalized() * interval;  

            for(int j{1}; j <= numIntervals; ++j) {  
                Eigen::Vector3d newPoint = p1 + j * direction;  
                result.push_back(newPoint);  // 保存插值点  
            }  
        }  
    }  
    
    result.push_back(points.back());  // 保存最后一个点  
    return result;  
}

} //namespace nav_core
} //namespace path_motion_planning