#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
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
    world_model.create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("dirt_context");

    auto sg = context.first;
    auto ss = sg -> get_state_space();
    auto cs = sg -> get_control_space();

    space_point_t start = ss -> make_point();
    std::vector<double> s = params["/plant/start_state"].as<std::vector<double>>();
    ss -> copy_point_from_vector(start, s);

    dirt_t dirt("dirt");
    dirt_specification_t dirt_spec(context.first,context.second);

    dirt_spec.propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj)
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

    const int num_steps = 10;
    plan_t true_plan = plan_t(cs);
    for (int i = 0; i < num_steps; ++i)
    {
        true_plan.append_onto_back(1.0);
        cs -> sample(true_plan.back().control);
    }
    std::cout << true_plan.print() << std::endl;

    std::vector<trajectory_t*> trajectories;
    const int num_rollouts = 100;
    std::vector<space_point_t> end_states;

    for (int i = 0; i < num_rollouts; ++i)
    {
        trajectory_t* traj = new trajectory_t(ss);
        plan_t current_plan(true_plan);
        dirt_spec.propagate(start,current_plan,*traj);
        if (dirt_spec.valid_check(*traj))
        {
            trajectories.push_back(traj);
            end_states.push_back(ss -> clone_point(traj -> back()));
        }
        else
        {
            delete traj;
        }
    }

    std::cout << true_plan.print() << std::endl;
    std::cout << "Number of valid trajectories: " << trajectories.size() << std::endl;

    std::string output_dir = params["output_dir"].as<>();
    for (int i = 0; i < trajectories.size(); ++i)
    {
        std::string filename = out_path + output_dir + "trajectory_" + std::to_string(i) + ".txt";
        std::ofstream out(filename);
        out << trajectories[i] -> print();
        out.close();
    }

    convex_hull_t quickhull;

    std::vector<space_point_t> hull = quickhull.compute_hull(end_states);
    std::cout << "Number of points in hull: " << hull.size() << std::endl;

    std::ofstream out(out_path + output_dir + "hull.txt");
    for (int i = 0; i < hull.size(); ++i)
    {
        out << ss -> print_point(hull[i]) << std::endl;
    }

}