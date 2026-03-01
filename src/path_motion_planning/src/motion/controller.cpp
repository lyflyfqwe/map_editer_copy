#include "path_motion_planning/motion/controller.h"

namespace path_motion_planning{
namespace motion{
void 
Controller::updateOdom(const Eigen::Vector3d& pos, const Eigen::Matrix3d& mat, const Eigen::Vector2d& spd){
    robot_p_ = pos;
    auto getYaw = [&](const Eigen::Matrix3d& mat)->double {
        return std::atan2(mat(1, 0), mat(0, 0));
    };
    auto normYaw =[](double& yaw)->void {
        if(yaw > M_PI) yaw -= 2*M_PI;
        else if(yaw < -M_PI) yaw += 2*M_PI;
    };
    robot_yaw_ = getYaw(mat);
    normYaw(robot_yaw_);
    real_spd_ = spd;
}

void Controller::updatePathL(const std::vector<Eigen::Vector3d>& path){
    data_path_.setData(path);
}

void Controller::updatePathG(const std::vector<Eigen::Vector3d>& path, int traj_ahead_num){
    if(path.empty()) return;
    std::cout << "uuuuuuuuuuuuuuuuuuuuuuuuu [updatePathG]:" << static_cast<int>(path.size()) << "\n";
    global_.is_new_data = true;
    auto closed_i = Tools::findClosedIndexOfPath(path, robot_p_);
    auto path_size = static_cast<int>(path.size());
    int traj_index = path_size > traj_ahead_num + closed_i? traj_ahead_num + closed_i : path_size - 1;
    std::cout << "traj_index: " << traj_index << "\n";
    global_.target_pose = path.at(traj_index) - robot_p_; //全局路径大致的朝向
    auto traj_p = path.at(traj_index);
    std::cout << "traj_p: " << traj_p.x() << " " << traj_p.y() << " " << traj_p.z() << "\n";
    std::cout << "robot_p_: " << robot_p_.x() << " " << robot_p_.y() << " " << robot_p_.z() << "\n";
    global_.target_pos = path.back();
}

Eigen::Vector3d Controller::getCommand(){
    if(global_.is_new_data){ //接受到全局路径的标志位，需要提前旋转yaw至global-path的大致朝向
        Eigen::Vector3d command;
        if(!tracker_->getRobotCmdSuitGPose(global_.target_pose, robot_spd_, robot_yaw_, command)) return command;
        global_.is_new_data = false;
    }

    data_path_.lock(true);
    auto path = data_path_.getData();
    data_path_.lock(false);

    return tracker_->getRobotCmd(path, global_.target_pos, robot_p_, robot_spd_, robot_yaw_);
}

Eigen::Vector3d Controller::getCommand(Eigen::Vector2d& spd){
    if(global_.is_new_data){ //接受到全局路径的标志位，需要提前旋转yaw至global-path的大致朝向
        Eigen::Vector3d command;
        if(!tracker_->getRobotCmdSuitGPose(global_.target_pose, robot_spd_, robot_yaw_, command)) return command;
        global_.is_new_data = false;
    }

    data_path_.lock(true);
    auto path = data_path_.getData();
    data_path_.lock(false);

    auto cmd = tracker_->getRobotCmd(path, global_.target_pos, robot_p_, robot_spd_, robot_yaw_);
    spd = robot_spd_;
    return cmd;
}

} //namespace motion
} // namespace path_motion_planning