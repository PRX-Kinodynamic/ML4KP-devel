#include <iostream>
#include <fstream>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/controllers/ackermann_FO_ctrl.hpp"

using namespace prx;

bool state_increment(space_point_t pt, double step_inc, std::vector<double> lower_bounds, std::vector<double> upper_bounds)
{
    for (int i = 0; i < pt -> get_dim(); ++i)
    {
        pt -> at(i) = pt -> at(i) + step_inc;
        if (pt -> at(i) <= upper_bounds[i])
        {
            return true;
        }
        pt -> at(i) = lower_bounds[i];
    }
    return false;
}


int main(int argc, char* argv[])
{
    auto params = param_loader("examples/tripods/ackermann_ha_roa.yaml", argc, argv);

	simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t world_model({plant},{obstacle_list});
    world_model.create_context("context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("context");

    const auto ss = context.first -> get_state_space();
    const auto cs = context.first -> get_control_space();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    ss -> set_bounds(lower_bounds, upper_bounds);

    auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
    auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
    cs -> set_bounds(cs_lb, cs_up);

    auto start_state = ss -> make_point();
	auto goal_state = ss -> make_point();

    ss -> copy_point_from_vector(start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss -> copy_point_from_vector(goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    ss -> copy_from_point(start_state);
    
    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

    trajectory_t solution_traj(ss);

    int ss_dim = ss -> get_dimension();
    int cs_dim = cs -> get_dimension();

    ss -> print_bounds();
    cs -> print_bounds();

    ackermann_FO_ctrl_t ctrl_1(plant);
    ackermann_FO_ctrl_t ctrl_2(plant);

    double k_rho_1   = +1.0 ; //params["k_rho"].as<double>();
    double k_alpha_1 = +9.5 ; //params["k_alpha"].as<double>();
    double k_beta_1  = -9.0 ; //params["k_beta"].as<double>();

    
    double k_rho_2   = +1.0 ;
    double k_alpha_2 = +9.5 ;
    double k_beta_2  = -9.5 ;
    
    ctrl_1.set_gains(k_rho_1, k_alpha_1, k_beta_1);
    ctrl_2.set_gains(k_rho_2, k_alpha_2, k_beta_2);

    ctrl_1.set_goal(goal_state);
    ctrl_2.set_goal(goal_state);
    solution_traj.copy_onto_back(ss);

    ackermann_FO_ctrl_t* ctrl;
    // v = [-1.9, -1.8, -1.57]
    if (-1.9 <= solution_traj.back() -> at(0) &&
            -1.8 <= solution_traj.back() -> at(1) && solution_traj.back() -> at(1) <= 1.2)
    {
        ctrl = &ctrl_2;
    }
    else 
    {
        ctrl = &ctrl_1;
        // ctrl_1.compute_controls();
    }

    do
    {
        ctrl -> compute_controls();
        cs -> enforce_bounds();
        std::cout << "[Ackermann] " << plant << std::endl;

        plant -> propagate(simulation_step);
        // std::cout << "[plant] " << plant << std::endl;
        solution_traj.copy_onto_back(ss);

    }
    while(!checker.check()); //&& space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) > 0.01);

    params.print();

    std::cout << "Last state: " << solution_traj.back() << " distance: " << space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) << std::endl;

    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, 
        body_name, ss);

    vis_group -> add_animation(solution_traj, ss, start_state);

    vis_group -> output_html("ackermann_ctrl.html");

    delete vis_group;

    
}