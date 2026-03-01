#ifndef PATH_MOTION_PLANNING_PATH_SEARCHER_PATH_SEARCHER_H
#define PATH_MOTION_PLANNING_PATH_SEARCHER_PATH_SEARCHER_H

#include <unordered_set>
#include "altitude_map/local_map.h"
#include "path_motion_planning/path_searcher/gird_node.h"

namespace path_motion_planning{
namespace path_searcher{
class PathSearcher{
private:
    bool local_mode_{true}; //标志位，是local,或global
    altitude_map::AltitudeMap::Ptr map_;
    altitude_map::LocalMap::Ptr map_l_;

    // const std::vector<Eigen::Vector3i> neighbours_{{-1,1,1},{0,1,1},{1,1,1},
    //                                                 {-1,0,1},{0,0,1},{1,0,1},
    //                                                 {-1,-1,1},{0,-1,1},{1,-1,1},
    //         {-1,1,0},{0,1,0},{1,1,0},
    //         {-1,0,0},        {1,0,0},
    //         {-1,-1,0},{0,-1,0},{1,-1,0}, 
    //                                     {-1,1,-1},{0,1,-1},{1,1,-1},
    //                                     {-1,0,-1},{0,0,-1},{1,0,-1},
    //                                     {-1,-1,-1},{0,-1,-1},{1,-1,-1}};

    const std::vector<Eigen::Vector3i> neighbours_{{-1,1,1},{0,1,1},{1,1,1},
                                                    {-1,0,1}       ,{1,0,1},
                                                    {-1,-1,1},{0,-1,1},{1,-1,1},
            {-1,1,0},{0,1,0},{1,1,0},
            {-1,0,0},        {1,0,0},
            {-1,-1,0},{0,-1,0},{1,-1,0}, 
                                        {-1,1,-1},{0,1,-1},{1,1,-1},
                                        {-1,0,-1},         {1,0,-1},
                                        {-1,-1,-1},{0,-1,-1},{1,-1,-1}};

protected:
    long long max_cal_num_limit_;
    double w_travel_, del_thres_;

    // 将 std::string 转换为 UUID（哈希值）
    GridHash stringToHash(const std::string& input) {
        static boost::uuids::name_generator generator(boost::uuids::ns::oid());
        return generator(input);
    }

    GridHash indexToHash(const Eigen::Vector3i& index){
        std::string input = std::to_string(index.x()) + "," + std::to_string(index.y()) + "," + std::to_string(index.z());
        return stringToHash(input);
    }

    std::vector<Eigen::Vector3i> extractPath(std::unordered_set<GridNode, GridNodeHash, CompareIndex>& list,
            const GridNode& start, const GridNode& end){
        auto cur_node = *(list.find(end));
        std::vector<Eigen::Vector3i> path;
        while(cur_node != start){
            path.push_back(cur_node.index);
            GridNode node;
            node.hash = cur_node.f_hash;
            auto it = list.find(node);
            
            if(it != list.end()) cur_node = *it;
            else return {};
        }
        path.push_back(start.index);
        std::reverse(path.begin(), path.end());
        return path;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<PathSearcher> Ptr;

    std::vector<GridNode> Neighbours() const{
        std::vector<GridNode> nodes;
        for(const auto& n : neighbours_){
            GridNode node;
            node.index = n;
            node.g = std::sqrt(node.index.x()*node.index.x() 
                + node.index.y()*node.index.y() + node.index.z()*node.index.z());
            nodes.push_back(node);
        }
        return nodes;
    }

    bool isCanNeighBour(Eigen::Vector3i& index){
        return map_->canBeNeighbour(index);
    }

    /**
     * @brief 加载map，仅在map进行路径搜索
     * @param map map
     * @param w_travel sdf权重
     * @param max_cal_num_limit 允许的最大搜索量 
     */
    PathSearcher(altitude_map::AltitudeMap::Ptr& map, double w_travel, long long max_cal_num_limit){
        map_ = map;
        w_travel_ = w_travel;
        max_cal_num_limit_ = max_cal_num_limit;
    }

    /**
     * @brief 加载local_map和global_map，综合考虑二者重叠区域的sdf,在global_map进行路径搜索
     * @param map_g global_map
     * @param map_l local_map
     * @param w_travel sdf权重
     * @param max_cal_num_limit 允许的最大搜索量 
     */
    PathSearcher(altitude_map::AltitudeMap::Ptr& map_g, altitude_map::LocalMap::Ptr& map_l, 
            double w_travel, long long max_cal_num_limit){
        map_ = map_g;
        map_l_ = map_l;
        local_mode_ = false;
        w_travel_ = w_travel;
        max_cal_num_limit_ = max_cal_num_limit;
    }
        
    virtual ~PathSearcher() = default;

    bool isIndexAvailable(const Eigen::Vector3i& index){
        return map_->isIndexVaild(index);
    }

    bool isIndexAvailableWithBound(const Eigen::Vector3i& index){
        if(map_->isIndexVaild(index)) return map_->canBeNeighbour(index);
        return map_->isIndexVaildWithBound(index);
    }

    double getCost(const Eigen::Vector3i& index){
        if(local_mode_) return map_->getSDFValue(index);
        Eigen::Vector3d pos_g = map_->getGridCubeCenter(index); 
        return std::max(map_->getSDFValue(index), map_l_->getSDF2GlobalFromPos(pos_g));
    }

    /**
     * @brief 允许存在index不合法的情况，如果index不合法，返回0
     */
    double getCostIgnoreBound(const Eigen::Vector3i& index){
        assert(!local_mode_ && "[PathSearcher] getCostIgnoreBound must be local_mode");
        if(!map_->isIndexVaild(index)) return 0;
        return map_->getSDFValue(index);
    }

    double distance(const GridNode& a, const GridNode& b){
        auto diff = a.index-b.index;
        return std::sqrt(diff.x()*diff.x() + diff.y()*diff.y() + diff.z()*diff.z());
    }

    void setDelThres(double thres){
        del_thres_ = thres;
    }

    void setCalNumLimit(int limit){
        max_cal_num_limit_ = limit;
    }

    virtual bool plan(const Eigen::Vector3i& start, const Eigen::Vector3i& end, 
        std::vector<Eigen::Vector3i>& path) = 0;
};
} //namespace path_searcher
} //namespace path_motion_planning

#endif