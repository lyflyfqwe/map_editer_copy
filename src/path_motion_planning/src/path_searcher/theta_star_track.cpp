#include "path_motion_planning/path_searcher/theta_star_track.h"

namespace path_motion_planning{
namespace path_searcher{
ThetaStarTrack::ThetaStarTrack(altitude_map::AltitudeMap::Ptr& map, double w_travel, long long max_cal_num_limit)
    :ThetaStar(map, w_travel, max_cal_num_limit){}

bool ThetaStarTrack::plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end,
        const std::vector<Eigen::Vector3i>& path_in, std::vector<Eigen::Vector3i>& path_out){
    if(path_in.empty()){
        std::cout << "p_0\n";
        return false;
    } 
    Vec3iMap map_path_data;
    for(size_t i{0}; i < path_in.size(); i++){
        auto index = path_in.at(i);
        if(isIndexAvailable(index)) map_path_data.insert(index, static_cast<int>(i));
        else break;
    }
    std::cout << "map_path_data size:" << static_cast<int>(map_path_data.size()) << "\n";

    std::vector<GridNode> node_found_in_path;
    
    std::vector<Eigen::Vector3i>().swap(path_out);
    std::priority_queue<GridNode, std::vector<GridNode>, CompareCost> open_list;
    std::unordered_set<GridNode, GridNodeHash, CompareIndex> closed_list;

    GridNode node_start(start), node_end(end);
    node_start.hash = indexToHash(start); node_start.f_hash = node_start.hash;
    node_end.hash = indexToHash(end);
    open_list.push(node_start);

    const auto neighbours = Neighbours();
    long long cal_num{0};

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

        if(map_path_data.contains(cur_node.index)) node_found_in_path.push_back(cur_node);

        if(cur_node == node_end){
            std::cout << " cal_n:" << cal_num << " ";
            path_out = extractPath(closed_list, node_start, node_end);
            return true;
        }

        for(const auto& n : neighbours){
            GridNode node_new = cur_node + n;

            if(!isCanNeighBour(node_new.index)) continue;

            node_new.h = distance(node_new, node_end) + getCost(node_new.index) * w_travel_;
            node_new.hash = indexToHash(node_new.index);
            node_new.f_hash = cur_node.hash;

            if(closed_list.find(node_new) != closed_list.end()) continue; //存在于closed_list，跳过

            if(!isIndexAvailable(node_new.index) ||  //索引不合法
                    (getCost(node_new.index) >= del_thres_   //节点的sdf值大于del_thres
                        && getCost(node_new.index) >= getCost(cur_node.index))) 
                continue;
            
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
    std::cout << " cal_n not end:" << cal_num << " ";
    if(!node_found_in_path.empty()){
        std::cout << "node_found_in_path size: " << static_cast<int>(node_found_in_path.size()) << "\n"; 
        int max_v_i{-1}, max_v{-1};
        for(size_t i{0}; i < node_found_in_path.size(); i++){
            auto cur_node = node_found_in_path.at(i);
            int v, z;
            if(!map_path_data.get(cur_node.index, v, z)) continue;
            if(std::abs(cur_node.index.z() - z) <= error_z_thres_ && v > max_v){
                max_v = v;
                max_v_i = i;
            }
        }
        if(max_v_i < 0){
            std::cout << "p_1\n";
            return false;
        } 
        path_out = extractPath(closed_list, node_start, node_found_in_path.at(max_v_i));
        return true;
    }
    std::cout << "p_2 1111111111111111111111111111111111111111111\n";
    return false;
}
} //namespace path_searcher
} //namespace path_motion_planning