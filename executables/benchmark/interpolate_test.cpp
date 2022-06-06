#include "prx/utilities/defs.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    simulation_step = 0.01;
    init_random(11101993);

    std::string plant_name = "treaded_vehicle";
    std::string plant_type = "treaded_vehicle";
    auto plant = system_factory_t::create_system(plant_type,plant_name);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t world_model({plant},{});
    world_model.create_context("planning_context",{plant_name},{});
    auto context = world_model.get_context("planning_context");
    auto ss = context.first -> get_state_space();

    space_point_t pt1 = ss -> make_point();
    space_point_t pt2 = ss -> make_point();
    pt2 -> at(0) = 1; pt2 -> at(2) = PRX_PI;

    trajectory_t traj(ss);

    default_interpolate_trajectory(pt1,pt2,traj,10);
    std::cout << "Interpolated trajectory: " << std::endl;
    std::cout << traj.print(4) << std::endl;
}