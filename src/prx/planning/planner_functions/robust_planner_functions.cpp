#include "prx/planning/planner_functions/robust_planner_functions.hpp"

namespace prx
{
    void default_propagate_particles(std::vector<space_point_t>& pts, plan_t& plan, std::vector<trajectory_t*>& trajs, std::shared_ptr<system_group_t> sg)
    {
        for (unsigned i = 0; i < pts.size(); ++i)
        {
            trajs.push_back(new trajectory_t(sg->get_state_space()));
            sg->propagate(pts[i], plan, *trajs[i]);
        }
    }

    bool default_valid_particles(const std::vector<space_point_t>& pts, space_t* state_space, std::shared_ptr<collision_group_t> cg)
    {
        for (unsigned i = 0; i < pts.size(); ++i)
        {
            state_space -> copy_from_point(pts[i]);
            if (cg->in_collision() || !state_space -> satisfies_bounds(pts[i]))
                return false;
        }
        return true;
    }

    void default_compute_reachable_set(const std::vector<space_point_t>& pts, reachable_set_t& rs, space_t* state_space)
    {
        prx_throw("Not implemented");
    }
}