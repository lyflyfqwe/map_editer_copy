#include "altitude_map/prob/prob_map.h"
#include "debug_tools/debug_tools.h"

namespace altitude_map{

void ProbMap::resetCell(const int& hash_id){
    float& ret = occupancy_buffer_[hash_id];
    ret = 0;
}

void ProbMap::resetLocalMap(){
    double unk_value = (prob_cfg_.l_free + prob_cfg_.l_occ)/2.0; //l_free: -0.00399995 l_occ: 1.7346
    // Clear local map
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), unk_value);
    while (!raycast_data_.update_cache_id_g.empty()) {
        raycast_data_.update_cache_id_g.pop();
    }
    raycast_data_.batch_update_counter = 0;
    std::fill(raycast_data_.operation_cnt.begin(), raycast_data_.operation_cnt.end(), 0);
    std::fill(raycast_data_.hit_cnt.begin(), raycast_data_.hit_cnt.end(), 0);
}

bool ProbMap::isOccupied(const Eigen::Vector3i &id_g) const{
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sli_cfg_.virtual_ceil_height_id_g ||
        id_g.z() < sli_cfg_.virtual_ground_height_id_g + sli_cfg_.safe_margin_i) {
        return true;
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isUnknown(const Eigen::Vector3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return true;
    }
    if (id_g.z() > sli_cfg_.virtual_ceil_height_id_g ||
        id_g.z() < sli_cfg_.virtual_ground_height_id_g + sli_cfg_.safe_margin_i) {
        return false;
    }
    return isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isKnownFree(const Eigen::Vector3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sli_cfg_.virtual_ceil_height_id_g ||
        id_g.z() < sli_cfg_.virtual_ground_height_id_g + sli_cfg_.safe_margin_i) {
        return true;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

void ProbMap::hitPointUpdate(const int &hash_id, const int &hit_num){
    float& ret = occupancy_buffer_[hash_id]; //ret is the probability of the cell
    ret += prob_cfg_.l_hit * hit_num;
    if(ret > prob_cfg_.l_max) ret = prob_cfg_.l_max;
}

void ProbMap::missPointUpdate(const int &hash_id, const int &hit_num){
    float& ret = occupancy_buffer_[hash_id];//ret is the probability of the cell
    ret += prob_cfg_.l_miss * hit_num;
    if(ret < prob_cfg_.l_min) ret = prob_cfg_.l_min;
}

void ProbMap::probabilisticMapFromCache(){ 
    while (!raycast_data_.update_cache_id_g.empty()){
        Eigen::Vector3i id_g = raycast_data_.update_cache_id_g.front();
        raycast_data_.update_cache_id_g.pop();
        Eigen::Vector3i id_l;
        globalIndexToLocalIndex(id_g, id_l);
        int hash_id = getLocalIndexHash(id_l);
        if (raycast_data_.hit_cnt[hash_id] > 0)
            hitPointUpdate(hash_id, raycast_data_.hit_cnt[hash_id]);
        else
            missPointUpdate(hash_id, raycast_data_.operation_cnt[hash_id] - raycast_data_.hit_cnt[hash_id]);
            // missPointUpdate(hash_id, raycast_data_.operation_cnt[hash_id]);
        raycast_data_.hit_cnt[hash_id] = 0;
        raycast_data_.operation_cnt[hash_id] = 0;
    }
}

void ProbMap::insertUpdateCandidate(const Eigen::Vector3i &id_g, bool is_hit){
    const auto& hash_id = getHashIndexFromGlobalIndex(id_g);
    raycast_data_.operation_cnt[hash_id]++;
    if (raycast_data_.operation_cnt[hash_id] == 1) {
        raycast_data_.update_cache_id_g.push(id_g);
    }
    if (is_hit) {
        raycast_data_.hit_cnt[hash_id]++;
    }
}

void ProbMap::raycastProcess(const pcl::PointCloud<pcl::PointXYZI> &input_cloud, const Eigen::Vector3d &cur_odom){
    // bounding box of updated region
    raycast_data_.cache_box_min = cur_odom;
    raycast_data_.cache_box_max = cur_odom;
    Eigen::Vector3d raycast_box_min, raycast_box_max;

    {
        std::lock_guard<std::mutex> lck{raycast_data_.raycast_range_mtx};
        raycast_box_max = raycast_data_.local_update_box_max;
        raycast_box_min = raycast_data_.local_update_box_min;
    }

    /// Step 1; Raycast and add to update cache.
    const int& cloud_in_size = input_cloud.size();
    // new version of raycasting process
    auto raycasting_cloud = std::vector<Eigen::Vector3d>{};
    raycasting_cloud.reserve(cloud_in_size);

    // 1) process all non-inf points, update occupied probability
    int temperol_cnt{0};
    for (const auto& pcl_p : input_cloud) {
        // 1.1) intensity filter
        if (prob_cfg_.intensity_thresh > 0 &&
            pcl_p.intensity < prob_cfg_.intensity_thresh) {
            continue;
        }

        // 1.2) temporal filter
        if (temperol_cnt++ % prob_cfg_.point_filt_num) {
            continue;
        }

        Eigen::Vector3d p(pcl_p.x, pcl_p.y, pcl_p.z);
        Eigen::Vector3i pt_id_g;

        // no raycasting, purely add occ pints
        if (!prob_cfg_.raycasting_en) {
            if (insideLocalMap(p)) {
                double sqrdis = (p - cur_odom).squaredNorm();
                if(sqrdis<prob_cfg_.sqr_raycast_range_min){
                    continue;
                }
                posToGlobalIndex(p, pt_id_g);
                insertUpdateCandidate(pt_id_g, true);
                // record cache box size;
                raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
                raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
            }
            continue;
        }

        bool update_hit{true};
        // 1.3) filter for virtual ceil and ground
        if (p.z() > prob_cfg_.virtual_ceil_height) {
            update_hit = false;
            // find the intersect point with the ceil
            const double dz = p.z() - cur_odom.z();
            const double pc = prob_cfg_.virtual_ceil_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }
        else if (p.z() < prob_cfg_.virtual_ground_height) {
            update_hit = false;
            // find the intersect point with the ground
            const double dz = p.z() - cur_odom.z();
            const double pc = prob_cfg_.virtual_ground_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }

        // 1.4) bounding box filter
        // raycasting max
        const double sqr_dis = (p - cur_odom).squaredNorm();
        if (sqr_dis > prob_cfg_.sqr_raycast_range_max) {
            double k = prob_cfg_.raycast_range_max / sqrt(sqr_dis);
            p = k * (p - cur_odom) + cur_odom;
            update_hit = false;
        }

        if(sqr_dis < prob_cfg_.sqr_raycast_range_min) {
            continue;
        }

        // local map bound
        if (((p - raycast_box_min).minCoeff() < 0) ||
            ((p - raycast_box_max).maxCoeff() > 0)) {
            p = lineBoxIntersectPoint(p, cur_odom, raycast_box_min, raycast_box_max);
            update_hit = false;
        }

        // record cache box size;
        raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
        raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);

        // 1.4) for all validate hit points, update probability
        raycasting_cloud.push_back(p);

        if (update_hit) {
            posToGlobalIndex(p, pt_id_g);
            insertUpdateCandidate(pt_id_g, true);
        }
    }

    if (prob_cfg_.raycasting_en) {
        // 4) process all inf points, updae free probability
        for (const auto& p : raycasting_cloud) {
            Eigen::Vector3d raycast_start = (p - cur_odom).normalized() * prob_cfg_.raycast_range_min + cur_odom;
            raycast_data_.raycaster.setInput(raycast_start, p);
            Eigen::Vector3d ray_pt;
            while (raycast_data_.raycaster.step(ray_pt)) {
                Eigen::Vector3i cur_ray_id_g;
                posToGlobalIndex(ray_pt, cur_ray_id_g);
                if (!insideLocalMap(cur_ray_id_g)) break;
                insertUpdateCandidate(cur_ray_id_g, false);
            }
        }
    }
}

void ProbMap::updateLocalBox(const Eigen::Vector3d &cur_odom){
    std::lock_guard<std::mutex> lck(raycast_data_.raycast_range_mtx);

    Eigen::Vector3i cur_odom_i;
    posToGlobalIndex(cur_odom, cur_odom_i);
    Eigen::Vector3i local_updatebox_min_i, local_updatebox_max_i;

    if (prob_cfg_.raycasting_en) {
        local_updatebox_max_i = cur_odom_i + prob_cfg_.half_local_update_box_i;
        local_updatebox_min_i = cur_odom_i - prob_cfg_.half_local_update_box_i;
    }

    globalIndexToPos(local_updatebox_min_i, raycast_data_.local_update_box_min);
    globalIndexToPos(local_updatebox_max_i, raycast_data_.local_update_box_max);

    // the local update box must inside the local map
    raycast_data_.local_update_box_max = raycast_data_.local_update_box_max.cwiseMin(
        local_map_bound_max_d_);
    raycast_data_.local_update_box_min = raycast_data_.local_update_box_min.cwiseMax(
        local_map_bound_min_d_);
}

Eigen::Vector3d ProbMap::lineBoxIntersectPoint(const Eigen::Vector3d& pt, const Eigen::Vector3d& pos,
        const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max){
    Eigen::Vector3d diff = pt - pos;
    Eigen::Vector3d max_tc = box_max - pos;
    Eigen::Vector3d min_tc = box_min - pos;
    double min_t = 1000000;
    for(int i{0}; i < 3; ++i){
        if (fabs(diff[i]) > 0) {
            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;
            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }
    return pos + (min_t - 1e-3) * diff;
}

void ProbMap::initProbMap(){
    if(init_once) throw std::runtime_error("ProbMap can only init once.");
    init_once = true;

    prob_cfg_.initForTest();

    initSlidingMap(prob_cfg_.half_map_size_i, prob_cfg_.resolution,
                   prob_cfg_.map_sliding_en, prob_cfg_.map_sliding_thresh,
                   prob_cfg_.fix_map_origin);
    
    posToGlobalIndex(prob_cfg_.visualization_range, sli_cfg_.visualization_range_i);
    posToGlobalIndex(prob_cfg_.virtual_ceil_height, sli_cfg_.virtual_ceil_height_id_g);
    posToGlobalIndex(prob_cfg_.virtual_ground_height, sli_cfg_.virtual_ground_height_id_g);

    prob_cfg_.virtual_ceil_height = sli_cfg_.virtual_ceil_height_id_g * prob_cfg_.resolution;
    prob_cfg_.virtual_ground_height = sli_cfg_.virtual_ground_height_id_g * prob_cfg_.resolution;

    if(!prob_cfg_.map_sliding_en) mapSliding(prob_cfg_.fix_map_origin);

    int map_size = sli_cfg_.map_size_i.prod();

    occupancy_buffer_.resize(map_size, 0);
    raycast_data_.raycaster.setResolution(prob_cfg_.resolution);
    raycast_data_.operation_cnt.resize(map_size, 0);
    raycast_data_.hit_cnt.resize(map_size, 0);

    resetLocalMap();
}

bool ProbMap::isOccupied(const Eigen::Vector3d &pos) const{
    if(!insideLocalMap(pos)) {
        return false;
    }
    if(pos.z() > prob_cfg_.virtual_ceil_height ||
        pos.z() < prob_cfg_.virtual_ground_height) {
        return true;
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromPos(pos)]);  
}

bool ProbMap::isUnknown(const Eigen::Vector3d &pos) const{
    if(!insideLocalMap(pos)) {
        return true;
    }
    if(pos.z() > prob_cfg_.virtual_ceil_height ||
        pos.z() < prob_cfg_.virtual_ground_height) {
        return false;
    }

    return isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isKnownFree(const Eigen::Vector3d &pos) const{
    if(!insideLocalMap(pos)){
        return false;
    }
    if(pos.z() > prob_cfg_.virtual_ceil_height ||
        pos.z() < prob_cfg_.virtual_ground_height) {
        return false;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

GridType ProbMap::getGridType(Eigen::Vector3i &id_g) const{
    if (id_g.z() <= sli_cfg_.virtual_ground_height_id_g ||
        id_g.z() >= sli_cfg_.virtual_ceil_height_id_g - sli_cfg_.safe_margin_i){
        return GridType::OCCUPIED;
    }
    if (!insideLocalMap(id_g)) {
        return GridType::OUT_OF_MAP;
    }
    Eigen::Vector3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    int hash_id = getLocalIndexHash(id_l);
    double ret = occupancy_buffer_[hash_id];
    if (isKnownFree(ret)) {
        return GridType::KNOWN_FREE;
    }
    else if (isOccupied(ret)) {
        return GridType::OCCUPIED;
    }
    else {
        return GridType::UNKNOWN;
    }
}

GridType ProbMap::getGridType(const Eigen::Vector3d &pos) const{
    if(pos.z() <= prob_cfg_.virtual_ground_height ||
        pos.z() >= prob_cfg_.virtual_ceil_height) {
        return OCCUPIED;
    }
    if(!insideLocalMap(pos)) {
        return OUT_OF_MAP;
    }
    Eigen::Vector3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    return getGridType(id_g);
}

double ProbMap::getMapValue(const Eigen::Vector3d &pos) const{
    if (!insideLocalMap(pos)) return 0;
    return occupancy_buffer_[getHashIndexFromPos(pos)];
}

void ProbMap::boundBoxByLocalMap(Eigen::Vector3d &box_min, Eigen::Vector3d &box_max) const{
    if((box_max - box_min).minCoeff() <= 0) {
        box_min = box_max;
        return;
    }

    box_min = box_min.cwiseMax(local_map_bound_min_d_);
    box_max = box_max.cwiseMin(local_map_bound_max_d_);
    box_max.z() = std::min(box_max.z(), prob_cfg_.virtual_ceil_height);
    box_min.z() = std::max(box_min.z(), prob_cfg_.virtual_ground_height);
}

void ProbMap::boxSearch(const Eigen::Vector3d &_box_min, const Eigen::Vector3d &_box_max,
        const GridType &gt, std::vector<Eigen::Vector3d> &out_points) const{
    out_points.clear();
    if(map_empty_) return;
    if((_box_max - _box_min).minCoeff() <= 0) return;

    Eigen::Vector3d box_min_d = _box_min, box_max_d = _box_max;
    boundBoxByLocalMap(box_min_d, box_max_d);
    if((box_max_d - box_min_d).minCoeff() <= 0) return;

    Eigen::Vector3i box_min_id_g, box_max_id_g;
    posToGlobalIndex(box_min_d, box_min_id_g);
    posToGlobalIndex(box_max_d, box_max_id_g);
    Eigen::Vector3i box_size = box_max_id_g - box_min_id_g;

    if(gt == UNKNOWN) {
        out_points.reserve(box_size.prod());
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Eigen::Vector3i id_g(i, j, k);
                    if (isUnknown(id_g)) {
                        Eigen::Vector3d pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else if(gt == OCCUPIED) {
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Eigen::Vector3i id_g(i, j, k);
                    if (isOccupied(id_g)) {
                        Eigen::Vector3d pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
}

Eigen::Vector3i ProbMap::boxSearch(const Eigen::Vector3d& odom_pos, int*** voxel_map, 
        int size_x, int size_y, int size_z, Eigen::Vector3i bound_zr_min, int* iter_z){
    Eigen::Vector3i bound_zr_max = bound_zr_min + Eigen::Vector3i(size_x, size_y, size_z);
    auto obs_num = [&](int num){return num>0? num : -num;};
    if(std::max(obs_num(bound_zr_min.x()), obs_num(bound_zr_max.x())) > prob_cfg_.half_local_update_box_i.x() 
            || std::max(obs_num(bound_zr_min.y()), obs_num(bound_zr_max.y())) > prob_cfg_.half_local_update_box_i.y()
            || std::max(obs_num(bound_zr_min.z()), obs_num(bound_zr_max.z())) > prob_cfg_.half_local_update_box_i.z()){
        std::cout << "x: " << obs_num(bound_zr_min.x()) << " " << obs_num(bound_zr_max.x()) << " " << prob_cfg_.half_local_update_box_i.x() << "\n";
        std::cout << "y: " << obs_num(bound_zr_min.y()) << " " << obs_num(bound_zr_max.y()) << " " << prob_cfg_.half_local_update_box_i.y() << "\n";
        std::cout << "z: " << obs_num(bound_zr_min.z()) << " " << obs_num(bound_zr_max.z()) << " " << prob_cfg_.half_local_update_box_i.z() << "\n";
        std::cout << "[ProbMap] boxSearch, beyond search range\n";
        return Eigen::Vector3i::Zero();
    }
    // debug_tools::Debug().print("map size: ", size_x, size_y, size_z);
    Eigen::Vector3i odom_index;
    posToGlobalIndex(odom_pos, odom_index);
    // debug_tools::Debug().print("[", odom_pos.x(), odom_pos.y(), odom_pos.z(), "][", odom_index.x(), odom_index.y(), odom_index.z(), "]");
    auto bound_prob_min = odom_index + bound_zr_min;
    std::for_each(std::execution::par, iter_z, iter_z+size_z, [&](int z){
        for(int y{0}; y < size_y; y++){
            for(int x{0}; x < size_x; x++){
                Eigen::Vector3i index = bound_prob_min + Eigen::Vector3i(x, y, z);
                voxel_map[z][y][x] = isOccupied(index)? 1 : 0;
            }
        }
    });
    return odom_index;
}

void ProbMap::updateProbMap(const pcl::PointCloud<pcl::PointXYZI> &cloud, const Eigen::Vector3d& pos){
    if(prob_cfg_.map_sliding_en && !insideLocalMap(pos) && raycast_data_.batch_update_counter == 0){
        mapSliding(pos);
        std::cout << "map slide a\n";
        return;
    }

    if(pos.z() > prob_cfg_.virtual_ceil_height) return;
    else if(pos.z() < prob_cfg_.virtual_ground_height) return;

    if(raycast_data_.batch_update_counter == 0 &&
        prob_cfg_.map_sliding_en  &&
        (map_empty_ || (pos - local_map_origin_d_).norm() > prob_cfg_.map_sliding_thresh)){
            std::cout << "map slide b\n";
            mapSliding(pos);
    } 

    debug_tools::Timer t_0;
    updateLocalBox(pos);
    debug_tools::Timer t_1(t_0); t_1.log("+++ updateLocalBox", "ms");
    raycastProcess(cloud, pos);
    debug_tools::Timer t_2(t_1); t_2.log("+++ raycastProcess", "ms");
    raycast_data_.batch_update_counter++;
    if(raycast_data_.batch_update_counter >= prob_cfg_.batch_update_size) {
        raycast_data_.batch_update_counter = 0;
        probabilisticMapFromCache();
        debug_tools::Timer t_3(t_2); t_3.log("+++ probabilisticMapFromCache", "ms");
        map_empty_ = false;
        std::cout << "map slide c\n";
    }

    /* For the first frame, clear all unknown around the robot */
    static bool first = true;
    if(first) {
        first = false;
        for(double dx = -prob_cfg_.raycast_range_min; dx <= prob_cfg_.raycast_range_min; dx += prob_cfg_.resolution) {
            for(double dy = -prob_cfg_.raycast_range_min; dy <= prob_cfg_.raycast_range_min; dy += prob_cfg_.resolution) {
                for(double dz = -prob_cfg_.raycast_range_min; dz <= prob_cfg_.raycast_range_min; dz += prob_cfg_.resolution) {
                    Eigen::Vector3d p(dx, dy, dz);
                    if(p.norm() <= prob_cfg_.raycast_range_min){
                        Eigen::Vector3d pp = pos + p;
                        int hash_id = getHashIndexFromPos(pp);
                        missPointUpdate(hash_id, 999);
                    }
                }
            }
        }
    }
}

}//namespace altitude_map