#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/simulation/plants/plants.hpp"

#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include "prx/utilities/data_structures/convex_hull.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/robust/test.yaml");

    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;
        
    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    std::vector<double> lb = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    std::vector<double> ub = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    plant -> set_state_space_bounds(lb, ub);

    world_model_t world_model({plant},{obstacle_list});
    world_model.create_context("rrt_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("rrt_context");

    auto sg = context.first;
    auto ss = sg -> get_state_space();
    auto cs = sg -> get_control_space();

    rrt_t rrt("rrt");
    rrt_specification_t rrt_spec(context.first,context.second);

    rrt_spec.propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj)
    {
        // Add some noise to the plan's control.
        for (auto p = plan.begin(); p != plan.end(); ++p)
        {
            auto ctrl = p -> control;
            for (unsigned i = 0; i < ctrl->get_dim(); ++i)
            {
                ctrl -> at(i) += uniform_random(-0.1,0.1);
            }
        }
        default_propagate(start_state,plan,out_traj,sg);
    };

    rrt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
    rrt_spec.max_control_steps = params["/plant/max_steps"].as<int>();

    rrt_query_t rrt_query(ss, cs);
    rrt_query.start_state = ss -> make_point();
    rrt_query.goal_state  = ss -> make_point();

    ss -> copy_point_from_vector(rrt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss -> copy_point_from_vector(rrt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

    rrt_query.goal_region_radius = params["goal_region_radius"].as<double>();

    rrt.link_and_setup_spec(&rrt_spec);
    rrt.preprocess();
    rrt.link_and_setup_query(&rrt_query);

    condition_check_t checker("time", 5.0);
    rrt.resolve_query(&checker);
    rrt.fulfill_query(); 

    std::vector<trajectory_t*> trajectories;
    const int num_rollouts = 100;
    std::vector<space_point_t> end_states;
    space_point_t end_state = ss -> make_point();

    for (int i = 0; i < num_rollouts; ++i)
    {
        trajectory_t* traj = new trajectory_t(ss);
        plan_t current_plan(rrt_query.solution_plan);

        rrt_spec.propagate(rrt_query.start_state, current_plan, *traj);
        ss -> copy_point(end_state, traj -> back());
        if (rrt_spec.valid_check(*traj) && rrt_query.goal_check(end_state))
        {
            trajectories.push_back(traj);
            end_states.push_back(end_state);
        }
        else
        {
            delete traj;
        }
    }

    std::string output_dir = params["output_dir"].as<>();
    for (int i = 0; i < trajectories.size(); ++i)
    {
        std::string filename = out_path + output_dir + "trajectory_" + std::to_string(i) + ".txt";
        std::ofstream out(filename);
        out << trajectories[i] -> print();
        out.close();
    }

    std::cout << "Success rate: " << trajectories.size() / (double)num_rollouts << std::endl;
}
