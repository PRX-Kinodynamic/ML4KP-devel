#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/intermediate/lqr.yaml", argc, argv);

	simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;

    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t<> world_model({plant},{obstacle_list});
    world_model.create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("dirt_context");

    const auto ss = context.first -> get_state_space();
    const auto cs = context.first -> get_control_space();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    ss -> set_bounds(lower_bounds, upper_bounds);

    auto start_state = ss -> make_point();
	auto goal_state = ss -> make_point();

    ss -> copy_point_from_vector(start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss -> copy_point_from_vector(goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    ss -> copy_from_point(start_state);
    
    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

    trajectory_t solution_traj(ss);

    auto pendulum = std::dynamic_pointer_cast<prx::pendulum_t>(plant);
    pendulum -> linearize();


    auto Q = Eigen::MatrixXd::Identity(2,2);
    auto R = Eigen::MatrixXd::Identity(1,1);
    lqr_t lqr(pendulum, Q, R, "LQR");
    lqr.compute_K();
    Eigen::MatrixXd K = lqr.get_K();
    std::cout << "K: " << K << std::endl;
    
    do
    {
        cs -> enforce_bounds();
        lqr.compute_controls();
        plant -> propagate(simulation_step);
        std::cout << "[pendulum] " << plant << std::endl;
        solution_traj.copy_onto_back(ss);

    }
    while(!checker.check() && space_t::euclidean_2d(solution_traj.back(), goal_state, 0, 2) > 0.01);

    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, 
        body_name, ss);

    vis_group -> add_animation(solution_traj, ss, start_state);

    vis_group -> output_html("lqr_pendulum.html");

    delete vis_group;

    params.print();
}