#include "path_motion_planning/path_searcher/theta_star_explored.h"

namespace path_motion_planning{
namespace path_searcher{

bool ThetaStarExplored::lineOfSight(const GridNode& parent, const GridNode& child){
    // 计算 x y z 方向上的步进值
    Eigen::Vector3i value = parent.index - child.index;
    int s_x = (value.x() == 0) ? 0 : value.x() / std::abs(value.x());
    int s_y = (value.y() == 0) ? 0 : value.y() / std::abs(value.y());  
    int s_z = (value.z() == 0) ? 0 : value.z() / std::abs(value.z());  

    int d_x = std::abs(parent.index.x() - child.index.x());  
    int d_y = std::abs(parent.index.y() - child.index.y());  
    int d_z = std::abs(parent.index.z() - child.index.z());

    if(d_x >= d_y && d_x >= d_z){  
        // x 方向主导  
        int tau_y = d_y - d_x;  
        int tau_z = d_z - d_x;  
        int x = child.index.x(), y = child.index.y(), z = child.index.z();  
        int e_y{0}, e_z{0};  

        while(x != parent.index.x()){  
            if (e_y * 2 > tau_y) {  
                y += s_y;  
                e_y -= d_x;  
            }  
            if (e_z * 2 > tau_z) {  
                z += s_z;  
                e_z -= d_x;  
            }  

            x += s_x;   

            if(getCostIgnoreBound(Eigen::Vector3i(x, y, z)) >= del_thres_ 
                    || getCostIgnoreBound(Eigen::Vector3i(x, y, z)) > getCostIgnoreBound(parent.index)) 
                return false; // 检测到障碍物  
        }
    }
    else if (d_y >= d_x && d_y >= d_z) {  
        // y 方向主导  
        int tau_x = d_x - d_y;  
        int tau_z = d_z - d_y;  
        int x = child.index.x(), y = child.index.y(), z = child.index.z();  
        int e_x{0}, e_z{0};  

        while (y != parent.index.y()) {  
            if (e_x * 2 > tau_x) {  
                x += s_x;  
                e_x -= d_y;  
            }  
            if (e_z * 2 > tau_z) {  
                z += s_z;  
                e_z -= d_y;  
            }  

            y += s_y;  
            
            if(getCostIgnoreBound(Eigen::Vector3i(x, y, z)) >= del_thres_ 
                || getCostIgnoreBound(Eigen::Vector3i(x, y, z)) > getCostIgnoreBound(parent.index)) 
            return false; // 检测到障碍物  
        }
    }
    else{  // z 方向主导  
        int tau_x = d_x - d_z;  
        int tau_y = d_y - d_z;  
        int x = child.index.x(), y = child.index.y(), z = child.index.z();  
        int e_x{0}, e_y{0};  

        while(z != parent.index.z()){  
            if (e_x * 2 > tau_x) {  
                x += s_x;  
                e_x -= d_z;  
            }  
            if (e_y * 2 > tau_y) {  
                y += s_y;  
                e_y -= d_z;  
            }  

            z += s_z;  

            if(getCostIgnoreBound(Eigen::Vector3i(x, y, z)) >= del_thres_ 
                || getCostIgnoreBound(Eigen::Vector3i(x, y, z)) > getCostIgnoreBound(parent.index)) 
            return false; // 检测到障碍物  
        }
    }
    return true; // 没有检测到障碍物
}

void ThetaStarExplored::updateG(const GridNode& parent, GridNode& child){
    if(!lineOfSight(parent, child)) return;
    if(parent.g + distance(parent, child) < child.g){
        child.g = parent.g + distance(parent, child);
        child.f_hash = parent.hash;
    }
}

ThetaStarExplored::ThetaStarExplored(altitude_map::AltitudeMap::Ptr& map, double w_travel, long long max_cal_num_limit)
        :PathSearcher(map, w_travel, max_cal_num_limit){}

bool ThetaStarExplored::plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end, 
        std::vector<Eigen::Vector3i>& path){
    std::vector<Eigen::Vector3i>().swap(path);
    std::priority_queue<GridNode, std::vector<GridNode>, CompareCost> open_list;
    std::unordered_set<GridNode, GridNodeHash, CompareIndex> closed_list;

    GridNode node_start(start), node_end(end);
    node_start.hash = indexToHash(start); node_start.f_hash = node_start.hash;
    node_end.hash = indexToHash(end);
    open_list.push(node_start);

    const auto neighbours = Neighbours();
    long long cal_num{0};

    auto sameIndexXY = [&](const Eigen::Vector3i& a, const Eigen::Vector3i& b, int error_z)->bool {
        if(!isIndexAvailable(a) && !isIndexAvailable(b)){
            // auto result = a.x() == b.x() && a.y() == b.y() && std::abs(a.z()-b.z()) < error_z;
            auto result = a.x() == b.x() && a.y() == b.y();
            if(result) debug_tools::Debug().print("aaaaa_2222: ", result);
            return result;
        } 
        return false; 
    };

    while(!open_list.empty()){
        if(cal_num++ >= max_cal_num_limit_){
            std::cout << " cal_n:" << cal_num << " ";
            break;
        } 

        auto top = open_list.top();
        GridNode cur_node = top;
        cur_node.hash = top.hash;
        cur_node.f_hash = top.f_hash;
        open_list.pop();

        if(closed_list.find(cur_node) != closed_list.end()) continue;

        closed_list.insert(cur_node);

        if(cur_node == node_end){
            std::cout << " cal_n:" << cal_num << " ";
            path = extractPath(closed_list, node_start, node_end);
            return true;
        }
        if(sameIndexXY(cur_node.index, node_end.index, 10)){
            std::cout << " cal_n s:" << cal_num << " ";
            path = extractPath(closed_list, node_start, cur_node);
            return true;
        }

        for(const auto& n : neighbours){
            GridNode node_new = cur_node + n;
            if(!isIndexAvailableWithBound(node_new.index)) continue;
            if(!isIndexAvailable(cur_node.index) && isIndexAvailable(node_new.index)) continue; //cur_node位于边界,new_node位于边界内
            if(!isIndexAvailable(cur_node.index) && !isIndexAvailable(node_new.index) 
                && cur_node.index.z() != node_new.index.z()) continue; //node_new cur_node 均位于边界,二者的z不同

            node_new.h = distance(node_new, node_end) + getCostIgnoreBound(node_new.index) * w_travel_;
            node_new.hash = indexToHash(node_new.index);
            node_new.f_hash = cur_node.hash;

            if(closed_list.find(node_new) != closed_list.end()) continue; //存在于closed_list，跳过

            if(getCostIgnoreBound(node_new.index) >= del_thres_ 
                    && getCostIgnoreBound(node_new.index) >= getCostIgnoreBound(cur_node.index)) continue;

            GridNode parent;
            parent.hash = cur_node.f_hash;

            auto find_parent = closed_list.find(parent);
            if(find_parent != closed_list.end()){
                parent = *find_parent;
                parent.hash = (*find_parent).hash;
                parent.f_hash = (*find_parent).f_hash;
                updateG(parent, node_new);
            }
            open_list.push(node_new);
        }
    }
    std::cout << " cal_n:" << cal_num << " ";
    return false;
}

} //namespace path_searcher
} //namespace path_motion_planning