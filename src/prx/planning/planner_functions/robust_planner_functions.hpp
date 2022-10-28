#pragma once

#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/convex_hull.hpp"

namespace prx
{
    // @aravind: TODO
    typedef convex_hull_t reachable_set_t;

    typedef std::function<void (std::vector<space_point_t>&, std::vector<plan_t*>&, std::vector<trajectory_t*>&)> propagate_particles_t;
    typedef std::function<bool (const std::vector<space_point_t>&)> valid_particles_t;
    typedef std::function<bool (const std::vector<trajectory_t*>&)> valid_trajectories_t;
    typedef std::function<void (const std::vector<space_point_t>&, reachable_set_t&)> compute_reachable_set_t;
    typedef std::function<bool (std::vector<space_point_t>&)> goal_check_particles_t;
    
    void default_propagate_particles(std::vector<space_point_t>&, std::vector<plan_t*>&, std::vector<trajectory_t*>&, std::shared_ptr<system_group_t>);
    bool default_valid_particles(const std::vector<space_point_t>&, space_t*, std::shared_ptr<collision_group_t>);
    bool default_valid_trajectories(const std::vector<trajectory_t*>&, valid_trajectory_t);
    void default_compute_reachable_set(const std::vector<space_point_t>&, reachable_set_t&, space_t*);
    bool default_particles_goal_check(std::vector<space_point_t>&, goal_check_t);
}