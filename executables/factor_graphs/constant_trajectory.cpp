#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("executables/factor_graphs/constant_trajectory.yaml", argc, argv);

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

    auto sg = context.first;
    const auto ss = sg -> get_state_space();
    const auto cs = sg -> get_control_space();
    const auto ps = sg -> get_parameter_space();

    ps -> copy_from_vector(params["/plant/parameters"].as<std::vector<double>>());

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    ss -> set_bounds(lower_bounds, upper_bounds);

    auto start_state = ss -> make_point();

    ss -> copy_point_from_vector(start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss -> copy_from_point(start_state);

    trajectory_t traj_out(ss);
    // solution_traj.copy_onto_back(ss);

    plan_t plan(cs);
    auto ctrl_pt = cs -> make_point();
    cs -> copy_point_from_vector(ctrl_pt, params["/plant/constant_ctrl"].as<std::vector<double>>());
    plan.copy_onto_back(ctrl_pt, params["/plant/ctrl_duration"].as<double>());

    sg -> propagate(start_state, plan, traj_out);

    std::string file_prefix = out_path + "omnibot_trajs/";

    std::string plan_file_name = file_prefix + "constant_plan_" + params["/plant/name"].as<>() + "_" + params["out_file"].as<>();
    std::string traj_file_name = file_prefix + "constant_traj_" + params["/plant/name"].as<>() + "_" + params["out_file"].as<>();
    
    plan.to_file(plan_file_name);
    traj_out.to_file(traj_file_name);
    
    // std::cout << "Control: " << params["/plant/constant_ctrl"].as<>() << std::endl;
    PRX_DEBUG_ITERABLE("Control: ", params["/plant/constant_ctrl"].as<std::vector<double>>());
    PRX_DEBUG_ITERABLE("Parameters: ", params["/plant/parameters"].as<std::vector<double>>());
    // std::cout << "Parameters: " << params["/plant/parameters"].as<>() << std::endl;
    std::cout << "plan file name: " << plan_file_name << std::endl;
    std::cout << "trajectory file name: " << traj_file_name << std::endl;

    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_out, 
        body_name, ss);

    vis_group -> add_animation(traj_out, ss, start_state);

    vis_group -> output_html("constant_trajectory_" + params["/plant/name"].as<>() + ".html");

    delete vis_group;

    // params.print();
    
}