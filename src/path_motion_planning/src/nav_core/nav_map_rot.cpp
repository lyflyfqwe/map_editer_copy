#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <string>

// 定义旋转函数
pcl::PointCloud<pcl::PointXYZ>::Ptr rotatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::string &axis,
    double angle_deg) {

    // 计算旋转角度
    double angle_rad = angle_deg * M_PI / 180.0;

    // 创建旋转矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    if (axis == "x") {
        transform.rotate(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitX()));
    } else if (axis == "y") {
        transform.rotate(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitY()));
    } else if (axis == "z") {
        transform.rotate(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitZ()));
    } else {
        throw std::invalid_argument("Invalid axis. Use 'x', 'y' or 'z'.");
    }

    // 旋转点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *rotated_cloud, transform);

    return rotated_cloud;
}

class PointCloudRotator : public rclcpp::Node {
public:
    PointCloudRotator()
        : Node("rotate_pointcloud_node") {

        // 直接在代码中指定输入路径、旋转轴、角度和输出路径
        std::string path = "/home/simpletime/code/rm/rm.pcd";  // 输入点云文件路径
        std::string axis = "x";                     // 旋转轴，可以是 "x", "y", 或 "z"
        double angle_deg = 7.4;                    // 旋转角度，单位是度
        std::string output_pcd = "rotated_map.pcd"; // 输出点云文件路径

        // 加载点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法读取点云文件：%s", path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "成功读取点云，点数：%ld", cloud->points.size());

        // 调用旋转函数
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud = rotatePointCloud(cloud, axis, angle_deg);

        // 保存旋转后的点云
        if (pcl::io::savePCDFileASCII(output_pcd, *rotated_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法保存点云文件：%s", output_pcd.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "已保存旋转后的点云到：%s", output_pcd.c_str());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRotator>());
    rclcpp::shutdown();
    return 0;
}
