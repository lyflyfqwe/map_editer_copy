#ifndef ALTITUDE_MAP_CLOUD_GENERATOR_H
#define ALTITUDE_MAP_CLOUD_GENERATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>

namespace altitude_map{
class CloudGenerator{
private:
    inline Eigen::Vector3f computeNormal(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, 
            const Eigen::Vector3f& p3) {
        Eigen::Vector3f u = p2 - p1;
        Eigen::Vector3f v = p3 - p1;
        return u.cross(v).normalized();
    }

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
    cylinder(std::initializer_list<double> center, double radius, double height, double resolution){
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if(center.size() != 3) return cloud; 
        auto iter = center.begin();
        Eigen::Vector3d center_p;
        center_p.x() = *iter++;
        center_p.y() = *iter++;
        center_p.z() = *iter;
        // 根据侧面积和分辨率计算侧面上的点的数量
        int num_points_per_circle = static_cast<int>(std::ceil(2.0 * M_PI * radius / resolution));
        int num_circles = static_cast<int>(std::ceil(height / resolution));
        int num_lateral_points = num_points_per_circle * num_circles;

        // 考虑到顶部和底部，我们为每个圆添加resolution长度上的点（即沿着圆的周长）
        // 这里的处理是简化的，它假设顶部和底部的点分布与侧面上的圆环相同
        int num_top_bottom_points = 2 * num_points_per_circle; // 顶部和底部各一圈点

        // 创建点云对象
        cloud->reserve(num_lateral_points + num_top_bottom_points); // 预留空间以提高效率

        // 随机数生成器
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis01(0.0, 1.0);

        // 生成圆柱体的侧面
        for (int i = 0; i < num_circles; ++i) {
            double current_height = i * (height / (num_circles - 1));
            for (int j = 0; j < num_points_per_circle; ++j) {
                double angle = 2.0 * M_PI * j / num_points_per_circle;
                double x = center_p.x() + radius * cos(angle);
                double y = center_p.y() + radius * sin(angle);
                double z = center_p.z() + current_height;
                cloud->points.push_back(pcl::PointXYZ(x, y, z));
            }
        }

        // 生成圆柱体的顶部和底部（可选，根据需求来决定是否生成）
        for (int j = 0; j < num_points_per_circle; ++j) {
            double angle = 2.0 * M_PI * j / num_points_per_circle;
            double x = center_p.x() + radius * cos(angle);
            double y = center_p.y() + radius * sin(angle);

            // 底部
            cloud->points.push_back(pcl::PointXYZ(x, y, center_p.z()));

            // 顶部
            cloud->points.push_back(pcl::PointXYZ(x, y, center_p.z() + height));
        }

        // 设置点云属性
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr 
    triangle(std::initializer_list<float> p1, std::initializer_list<float> p2, 
            std::initializer_list<float> p3, double resolution){
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();  
        if(p1.size() != 3 || p2.size() != 3 || p3.size() != 3) return cloud;
        Eigen::Vector3d A, B, C;
        auto it_p1 = p1.begin(); A.x() = *it_p1++; A.y() = *it_p1++; A.z() = *it_p1;
        auto it_p2 = p2.begin(); B.x() = *it_p2++; B.y() = *it_p2++; B.z() = *it_p2;
        auto it_p3 = p3.begin(); C.x() = *it_p3++; C.y() = *it_p3++; C.z() = *it_p3;   
        // 计算三角形的面积  
        Eigen::Vector3d AB = B - A;  
        Eigen::Vector3d AC = C - A;  
        double area = 0.5 * AB.cross(AC).norm();  
            
        // 根据密度计算生成的点个数  
        int numPoints = static_cast<int>(area/(resolution*resolution));  
        cloud->width = numPoints;  
        cloud->height = 1;  
        cloud->points.resize(numPoints);  

        for(int i{0}; i < numPoints; ++i) {  
            double r1 = static_cast<double>(rand()) / RAND_MAX;  
            double r2 = static_cast<double>(rand()) / RAND_MAX;  
            // 使用重心坐标来生成点  
            double s = sqrt(r1); // 使用平方根以保证均匀性  
            double t = r2;  

            // 重心坐标法生成点  
            Eigen::Vector3d point = (1 - s) * A + s * (1 - t) * B + s * t * C;  
            cloud->points[i].x = point(0);  
            cloud->points[i].y = point(1);  
            cloud->points[i].z = point(2);  
        }
        return cloud;
    }
};
} //namespace altitude_map

#endif