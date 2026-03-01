#ifndef ALTITUDE_MAP_PROB_CONFIG_HPP
#define ALTITUDE_MAP_PROB_CONFIG_HPP

#include <Eigen/Eigen>

namespace altitude_map{
struct ProbConfig{
    bool map_sliding_en{true};
    double map_sliding_thresh{-1.0};
    Eigen::Vector3d fix_map_origin{0, 0, 0};
    double resolution{0.05};
    Eigen::Vector3d visualization_range{};
    Eigen::Vector3d map_size_d{7, 7, 2}, half_map_size_d{};
    Eigen::Vector3i half_map_size_i{};

    bool raycasting_en{true};
    int intensity_thresh{}; //probability update
    Eigen::Vector3i local_update_box_i, half_local_update_box_i{}; //raycaster update range
    double raycast_range_min{}, raycast_range_max{};
    double sqr_raycast_range_min{}, sqr_raycast_range_max{};

    double virtual_ceil_height{}, virtual_ground_height{};

    int point_filt_num{1}, batch_update_size{};

    float p_hit{}, p_miss{}, p_min{}, p_max{}, p_occ{}, p_free{};
    float l_hit{}, l_miss{}, l_min{}, l_max{}, l_occ{}, l_free{};

    void initForTest(){
        map_sliding_en = true;
        map_sliding_thresh = 0.2;
        fix_map_origin = Eigen::Vector3d(0, 0, 0);
        visualization_range = Eigen::Vector3d(5, 5, 2);

        half_map_size_d = map_size_d / 2;
        half_map_size_i = (half_map_size_d / resolution).cast<int>();

        raycasting_en = true;
        intensity_thresh = -10;
        auto raycaster_range_d = map_size_d;
        local_update_box_i = (raycaster_range_d / resolution).cast<int>();
        half_local_update_box_i = local_update_box_i / 2;
        raycast_range_min = 0.1; raycast_range_max = 10;
        sqr_raycast_range_min = raycast_range_min * raycast_range_min;
        sqr_raycast_range_max = raycast_range_max * raycast_range_max;

        virtual_ceil_height = 4;
        virtual_ground_height = -4;

        point_filt_num = 1;
        batch_update_size = 1;

        p_min = 0.12; 
        p_max = 0.98;
        p_miss = 0.35;   //0.49
        p_hit = 0.9;
        p_free = 0.499; 
        p_occ = 0.85;

#define logit(x) (log((x) / (1 - (x))))

        l_hit = logit(p_hit);
        l_miss = logit(p_miss);
        l_min = logit(p_min);
        l_max = logit(p_max);
        l_occ = logit(p_occ);
        l_free = logit(p_free);

        std::cout << "l_free: " << l_free << " l_occ: " << l_occ << "\n";
    }
};
} //namespace altitude_map

#endif