#include "prx/utilities/defs.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planners/prm.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    simulation_step = 0.1;
    init_random(11101993);

    std::string plant_name = "treaded_vehicle";
    std::string plant_type = "treaded_vehicle";
    auto plant = system_factory_t::create_system(plant_type,plant_name);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t world_model({plant},{});
    world_model.create_context("planning_context",{plant_name},{});
    auto context = world_model.get_context("planning_context");
    auto ss = context.first -> get_state_space();

    prm_t prm("prm");
    prm_specification_t prm_spec(context.first,context.second);
    prm_spec.k = 10;
    prm_spec.M = 100;

    prm_query_t prm_query(ss,context.first->get_control_space());
    prm.link_and_setup_spec(&prm_spec);
    prm.preprocess();
}