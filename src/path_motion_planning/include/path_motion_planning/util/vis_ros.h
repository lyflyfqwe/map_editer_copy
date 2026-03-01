#ifndef PATH_MOTION_PLANNING_UTIL_VIS_ROS_H
#define PATH_MOTION_PLANNING_UTIL_VIS_ROS_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

namespace path_motion_planning{
namespace util{
class Tools{
protected:
    template<typename T> void 
    vecToEigen(std::vector<double>& vec, T& e){
        if constexpr (std::is_same_v<T, Eigen::Vector3d>){
            e = Eigen::Vector3d(vec.at(0), vec.at(1), vec.at(2));
        }
        else if constexpr (std::is_same_v<T, Eigen::Vector<double, 4>>){
            e = Eigen::Vector<double, 4>(vec.at(0), vec.at(1), vec.at(2), vec.at(3));
        }
    }
    
    /**
     * @brief 输入弧度制,输出四元数
     */
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw){    
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());  
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());  
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());  
    
        // 合成四元数，顺序为先 yaw、后 pitch、再 roll  
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;  
        return q;  
    }
}; //class Tools

class VisRos{
private:
    inline void pubCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
            sensor_msgs::msg::PointCloud2& msg, rclcpp::Node* node){
        msg.header.frame_id = "map";
        msg.header.stamp = node->now();
        pub->publish(msg);
    }

protected:
    /**
     * @brief 从容器内取出点坐标，以小球的形式发布可视化
     */
    void pubBallVisual(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub, rclcpp::Node* node,
            const std::vector<Eigen::Vector3d> points, std::initializer_list<float> color_rgb){
        if(pub->get_subscription_count() < 1) return;
        if(color_rgb.size() != 3) return;  
        auto c = color_rgb.begin();  
        float rgb[3]{*(c), *(c+1), *(c+2)};
        visualization_msgs::msg::MarkerArray marker_array; // 使用 MarkerArray 来存储多个小球标记  
        int id{0}; // 使用不同的 ID 来区分不同的小球  
        for(const auto& p : points){  
            visualization_msgs::msg::Marker marker;  
            marker.header.frame_id = "map"; // 使用地图框架  
            marker.header.stamp = node->now();  
            marker.ns = "balls"; // 命名空间  
            marker.id = id++; // 设置唯一 ID  
            marker.type = visualization_msgs::msg::Marker::SPHERE; // 设置为小球标记  
            marker.action = visualization_msgs::msg::Marker::ADD; // 添加标记  

            // 设置小球的颜色  
            marker.color.r = rgb[0]; // 红色  
            marker.color.g = rgb[1]; // 绿色  
            marker.color.b = rgb[2]; // 蓝色  
            marker.color.a = 1.0; // 透明度  

            marker.scale.x = 0.1; // 小球的直径（宽度）  
            marker.scale.y = 0.1; // 小球的直径（高度）  
            marker.scale.z = 0.1; // 小球的直径（深度）  

            // 设置小球的位置  
            marker.pose.position.x = p.x();  
            marker.pose.position.y = p.y();  
            marker.pose.position.z = p.z();  
            marker.pose.orientation.w = 1.0; // 默认的四元数（无旋转）  

            marker_array.markers.push_back(marker); // 将每个小球的标记添加到 MarkerArray 中  
        }
        pub->publish(marker_array); // 发布所有的小球标记
    }

    /**
     * @brief 发布路径可视化
     */
    void pubPathVisual(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub, rclcpp::Node* node,
            const std::vector<Eigen::Vector3d>& path){
        if(pub->get_subscription_count() < 1) return;
        nav_msgs::msg::Path msg_path;
        msg_path.header.frame_id = "map";
        msg_path.header.stamp = node->now();
        for(const auto& p : path){
            geometry_msgs::msg::PoseStamped pos;
            pos.pose.position.x = p.x();
            pos.pose.position.y = p.y();
            pos.pose.position.z = p.z() + 0.2;
            msg_path.poses.push_back(pos);
        }
        pub->publish(msg_path);
    }

    void pubPathVisual(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub, rclcpp::Node* node,
            const std::vector<Eigen::Vector4d>& path){
        std::vector<Eigen::Vector3d> path_3d;
        for(const auto& p : path) path_3d.push_back(Eigen::Vector3d(p.x(), p.y(), p.z()));
        return pubPathVisual(pub, node, path_3d);
    }

    void pubPathVisual(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, rclcpp::Node* node,
            std::vector<Eigen::Vector3d>& path, std::initializer_list<float> color_rgb){
        if(pub->get_subscription_count() < 1) return;
        visualization_msgs::msg::Marker marker;  
        marker.header.frame_id = "map"; // 使用地图框架  
        marker.header.stamp = node->now(); // 当前时间戳  
        marker.ns = "path"; // 命名空间  
        marker.id = 0; // ID  
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 设置为小球标记   
        marker.action = visualization_msgs::msg::Marker::ADD; // 添加标记  
        if(color_rgb.size() != 3) return;
        auto c = color_rgb.begin();
        // 设置点的颜色  
        marker.color.r = *c++; // 红色  
        marker.color.g = *c++; // 绿色  
        marker.color.b = *c; // 蓝色  
        marker.color.a = 1.0; // 透明度  
        marker.scale.x = 0.04; // 点的宽度  
        marker.scale.y = 0.04; // 点的高度 
        marker.scale.z = 0.06+0.2; 

        std::for_each(path.begin(), path.end(), [&](Eigen::Vector3d p){
            geometry_msgs::msg::Point point;  
            point.x = p.x();
            point.y = p.y();
            point.z = p.z()+0.2;
            marker.points.push_back(point);  
        }); 
        pub->publish(marker);
    }

    void pubArray(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, rclcpp::Node* node, std::string frame,
            const Eigen::Vector3d& p, const Eigen::Quaterniond& q){
        visualization_msgs::msg::Marker marker;  
        marker.header.frame_id = frame;
        marker.header.stamp = node->now(); // 时间戳  
        marker.ns = "points";  
        marker.id = 0; // 标记的 ID  
        marker.type = visualization_msgs::msg::Marker::ARROW; // 使用箭头类型来表示朝向  
        marker.action = visualization_msgs::msg::Marker::ADD;  
        // 设置位置  
        marker.pose.position.x = p.x(); // 三维点的 x, y, z 坐标  
        marker.pose.position.y = p.y();  
        marker.pose.position.z = p.z();  
        // 设置朝向（使用四元数表示）  
        marker.pose.orientation.x = q.x(); // 这里设置为单位四元数（没有旋转）  
        marker.pose.orientation.y = q.y();  
        marker.pose.orientation.z = q.z();  
        marker.pose.orientation.w = q.w();  
        // 设置标记的尺寸和颜色  
        marker.scale.x = 0.15; // 尺寸  
        marker.scale.y = 0.05;  
        marker.scale.z = 0.05;  
            
        // 颜色设置  
        marker.color.r = 1.0; // 红色  
        marker.color.g = 1.0;  
        marker.color.b = 0.0;  
        marker.color.a = 1.0; // 不透明  
        pub->publish(marker);
    }

    /**
     * @brief 发布点云可视化
     */
    void pubCloudVisual(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub, rclcpp::Node* node,
            const pcl::PointCloud<pcl::PointXYZ>& cloud){
        if(pub->get_subscription_count() < 1) return;
        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        pubCloud(pub, msg_cloud, node);
    }

    void pubCloudVisual(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub, rclcpp::Node* node,
            const pcl::PointCloud<pcl::PointXYZI>& cloud){
        if(pub->get_subscription_count() < 1) return;
        sensor_msgs::msg::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        pubCloud(pub, msg_cloud, node);
    }
};
} //namespace util
} //namespace path_motion_planning
#endif