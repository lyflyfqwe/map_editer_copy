#include "altitude_map/prob/sliding_map.h"

namespace altitude_map{

int SlidingMap::getLocalIndexHash(const Eigen::Vector3i &id_in) const{
    Eigen::Vector3i id = id_in + sli_cfg_.half_map_size_i;
    return id(0) * sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2) +
           id(1) * sli_cfg_.map_size_i(2) +
           id(2);
}

void SlidingMap::clearMemoryOutOfMap(const std::vector<int> &clear_id, const int &i){
    std::vector<int> ids{i, (i + 1) % 3, (i + 2) % 3};
    for(const auto &idd: clear_id){
        for(int x = -sli_cfg_.half_map_size_i(ids[1]); x <= sli_cfg_.half_map_size_i(ids[1]); x++) {
            for(int y = -sli_cfg_.half_map_size_i(ids[2]); y <= sli_cfg_.half_map_size_i(ids[2]); y++) {
                Eigen::Vector3i temp_clear_id;
                temp_clear_id(ids[0]) = idd;
                temp_clear_id(ids[1]) = x;
                temp_clear_id(ids[2]) = y;
                resetCell(getLocalIndexHash(temp_clear_id));
            }
        }
    }
}

void SlidingMap::posToGlobalIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) const{
    id = (sli_cfg_.resolution_inv * pos + pos.cwiseSign() * 0.5).cast<int>();
}

void SlidingMap::posToGlobalIndex(const double &pos, int &id) const{
    id = static_cast<int>((sli_cfg_.resolution_inv * pos + SIGN(pos) * 0.5));
}

void SlidingMap::globalIndexToPos(const Eigen::Vector3i &id_g, Eigen::Vector3d &pos) const{
    pos = id_g.cast<double>() * sli_cfg_.resolution;
}

void SlidingMap::globalIndexToLocalIndex(const Eigen::Vector3i &id_g, Eigen::Vector3i &id_l) const{
    for(int i{0}; i < 3; ++i){
        id_l(i) = id_g(i) % sli_cfg_.map_size_i(i);
        
        if(id_l(i) > sli_cfg_.half_map_size_i(i)) id_l(i) -= sli_cfg_.map_size_i(i);
        else if(id_l(i) < -sli_cfg_.half_map_size_i(i)) id_l(i) += sli_cfg_.map_size_i(i);
    }
}

void SlidingMap::localIndexToGlobalIndex(const Eigen::Vector3i &id_l, Eigen::Vector3i &id_g) const{
    for(int i{0}; i < 3; ++i) {
        int min_id_g = -sli_cfg_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sli_cfg_.map_size_i(i);
        min_id_l -= min_id_l > sli_cfg_.half_map_size_i(i) ? sli_cfg_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sli_cfg_.half_map_size_i(i) ? sli_cfg_.map_size_i(i) : 0;
        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sli_cfg_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        id_g(i) = cur_id;
    }
}

void SlidingMap::localIndexToPos(const Eigen::Vector3i &id_l, Eigen::Vector3d &pos) const{
    for(int i{0}; i < 3; ++i) {
        int min_id_g = -sli_cfg_.half_map_size_i(i) + local_map_origin_i_(i);

        int min_id_l = min_id_g % sli_cfg_.map_size_i(i);
        min_id_l -= min_id_l > sli_cfg_.half_map_size_i(i) ? sli_cfg_.map_size_i(i) : 0;
        min_id_l += min_id_l < -sli_cfg_.half_map_size_i(i) ? sli_cfg_.map_size_i(i) : 0;

        int cur_dis_to_min_id = id_l(i) - min_id_l;
        cur_dis_to_min_id =
                (cur_dis_to_min_id) < 0 ? (sli_cfg_.map_size_i(i) + cur_dis_to_min_id) : cur_dis_to_min_id;
        int cur_id = cur_dis_to_min_id + min_id_g;
        pos(i) = cur_id * sli_cfg_.resolution;
    }
}

void SlidingMap::hashIdToLocalIndex(const int &hash_id, Eigen::Vector3i &id) const{
    id(0) = hash_id / (sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2));
    id(1) = (hash_id - id(0) * sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2)) / sli_cfg_.map_size_i(2);
    id(2) = hash_id - id(0) * sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2) - id(1) * sli_cfg_.map_size_i(2);
    id -= sli_cfg_.half_map_size_i;
}

void SlidingMap::hashIdToPos(const int &hash_id, Eigen::Vector3d &pos) const{
    Eigen::Vector3i id;
    hashIdToLocalIndex(hash_id, id);
    localIndexToPos(id, pos);
}

void SlidingMap::hashIdToGlobalIndex(const int &hash_id, Eigen::Vector3i &id_g) const{
    Eigen::Vector3i id;
    id(0) = hash_id / (sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2));
    id(1) = (hash_id - id(0) * sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2)) / sli_cfg_.map_size_i(2);
    id(2) = hash_id - id(0) * sli_cfg_.map_size_i(1) * sli_cfg_.map_size_i(2) - id(1) * sli_cfg_.map_size_i(2);
    id -= sli_cfg_.half_map_size_i;
    localIndexToGlobalIndex(id, id_g);
}

int SlidingMap::getHashIndexFromPos(const Eigen::Vector3d &pos) const{
    Eigen::Vector3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}

int SlidingMap::getHashIndexFromGlobalIndex(const Eigen::Vector3i &id_g) const{
    Eigen::Vector3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    return getLocalIndexHash(id_l);
}

void SlidingMap::updateLocalMapOriginAndBound(const Eigen::Vector3d &new_origin_d, 
    const Eigen::Vector3i &new_origin_i){
    // update local map origin and local map bound
    local_map_origin_i_ = new_origin_i;
    local_map_origin_d_ = new_origin_d;

    local_map_bound_max_i_ = local_map_origin_i_ + sli_cfg_.half_map_size_i;
    local_map_bound_min_i_ = local_map_origin_i_ - sli_cfg_.half_map_size_i;

    // the double map bound only consider the closed cell center
    globalIndexToPos(local_map_bound_min_i_, local_map_bound_min_d_);
    globalIndexToPos(local_map_bound_max_i_, local_map_bound_max_d_);
}

void SlidingMap::initSlidingMap(const Eigen::Vector3i &half_map_size_i, const double &resolution,
        const bool &map_sliding_en, const double &sliding_thresh, const Eigen::Vector3d &fix_map_origin){
    if(has_been_initialized_) throw std::runtime_error(" -- [SlidingMap]: init can only be called once!");

    sli_cfg_.resolution = resolution;
    sli_cfg_.resolution_inv = 1.0 / resolution;
    sli_cfg_.map_sliding_en = map_sliding_en;
    sli_cfg_.sliding_thresh = sliding_thresh;
    sli_cfg_.fix_map_origin = fix_map_origin;
    sli_cfg_.half_map_size_i = half_map_size_i;
    sli_cfg_.map_size_i = 2 * sli_cfg_.half_map_size_i + Eigen::Vector3i::Constant(1);
    sli_cfg_.map_vox_num = sli_cfg_.map_size_i.prod();
    if (!map_sliding_en) {
        local_map_origin_d_ = fix_map_origin;
        posToGlobalIndex(local_map_origin_d_, local_map_origin_i_);
    }
    has_been_initialized_ = true;
}

SlidingMap::SlidingMap(const Eigen::Vector3i &half_map_size_i, const double &resolution,
        const bool &map_sliding_en, const double &sliding_thresh, const Eigen::Vector3d &fix_map_origin){
    initSlidingMap(half_map_size_i, resolution, map_sliding_en, sliding_thresh, fix_map_origin);
}

void SlidingMap::mapSliding(const Eigen::Vector3d &odom){
    Eigen::Vector3i new_origin_i;
    posToGlobalIndex(odom, new_origin_i);
    Eigen::Vector3d new_origin_d = new_origin_i.cast<double>() * sli_cfg_.resolution;
    Eigen::Vector3i shift_num = new_origin_i - local_map_origin_i_;

    for (long unsigned int i{0}; i < 3; i++) {
        if (fabs(shift_num[i]) > sli_cfg_.map_size_i[i]) {
            // Clear all map
            resetLocalMap();
            updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
            return;
        }
    }

    static auto normalize = [](int x, int a, int b) -> int {
        int range = b - a + 1;
        int y = (x - a) % range;
        return (y < 0 ? y + range : y) + a;
    };

    /// Clear the memory out of the map size
    for(int i{0}; i < 3; i++){
        if (shift_num[i] == 0) continue;

        int min_id_g = -sli_cfg_.half_map_size_i(i) + local_map_origin_i_(i);
        int min_id_l = min_id_g % sli_cfg_.map_size_i(i);
        std::vector<int> clear_id;
        if(shift_num(i) > 0){
            /// forward shift, the min id should be cut
            for(int k{0}; k < shift_num(i); k++) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sli_cfg_.half_map_size_i(i), sli_cfg_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        } 
        else{
            /// backward shift, the max should be shifted
            for(int k{-1}; k >= shift_num(i); k--) {
                int temp_id = min_id_l + k;
                temp_id = normalize(temp_id, -sli_cfg_.half_map_size_i(i), sli_cfg_.half_map_size_i(i));
                clear_id.push_back(temp_id);
            }
        }

        if(clear_id.empty()) continue;
        clearMemoryOutOfMap(clear_id, i);
    }

    updateLocalMapOriginAndBound(new_origin_d, new_origin_i);
}

bool SlidingMap::insideLocalMap(const Eigen::Vector3d &pos) const{
    Eigen::Vector3i id_g;
    posToGlobalIndex(pos, id_g);
    return insideLocalMap(id_g);
}

bool SlidingMap::insideLocalMap(const Eigen::Vector3i &id_g) const{
    if(((id_g - local_map_origin_i_).cwiseAbs() - sli_cfg_.half_map_size_i).maxCoeff() > 0){
        return false;
    }
    return true;
}

}//namespace altitude_map