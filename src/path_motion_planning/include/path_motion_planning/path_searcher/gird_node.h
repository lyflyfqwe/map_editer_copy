#ifndef PATH_MOTION_PLANNING_PATH_SEARCHER_GRID_NODE_H
#define PATH_MOTION_PLANNING_PATH_SEARCHER_GRID_NODE_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <string>
#include <Eigen/Eigen>

namespace path_motion_planning{
namespace path_searcher{
typedef boost::uuids::uuid GridHash;

struct GridNode{
    Eigen::Vector3i index;
    double g;           // 从起点到当前节点的代价  
    double h;           // 当前节点到目标节点的估计代价  
    GridHash hash, f_hash;

    GridNode(Eigen::Vector3i v_index=Eigen::Vector3i::Zero(), double v_g=0, double v_h=0): index(v_index), g(v_g), h(v_h){}
    
    GridNode operator+(const GridNode& node) const{
        GridNode node_out;
        node_out.index = index + node.index;
        node_out.g = g + node.g;
        return node_out;
    }

    GridNode operator-(const GridNode& node) const{
        GridNode node_out;
        node_out.index = index - node.index;
        node_out.g = g - node.g;
        return node_out;
    }

    bool operator==(const GridNode& node) const{
        return index == node.index;
    }

    bool operator!=(const GridNode& node) const{
        return !operator==(node);
    }
};

class GridNodeHash{
public:
    std::size_t operator()(const GridNode& node) const{
        return std::hash<std::string>()(boost::uuids::to_string(node.hash));  
    }
};

struct CompareCost{
    // 首先计算 a 和 b 的总成本,g 代表从起点到当前节点的实际成本,h 代表从当前节点到目标节点的启发式估计成本
    // 如果 a 的总成本(a.g_ + a.h)大于 b 的总成本，则返回 true
    // 如果两者的总成本相等(a.g + a.h == b.g + b.h),再比较它们的启发式成本 h;如果 a.h 大于 b.h,则返回 true
    bool operator()(const GridNode& a, const GridNode& b) const{
        return (a.g+a.h > b.g+b.h) || ((a.g+a.h == b.g+b.h) && (a.h > b.h));
    }
};

struct CompareIndex{
    bool operator()(const GridNode& a, const GridNode& b) const{
        return a.hash == b.hash;
    }
};
} //namespace path_searcher
} //namespace path_motion_planning

#endif