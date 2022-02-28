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
    auto params = param_loader("examples/intermediate/lqr_num.yaml", argc, argv);

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
    
    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

    trajectory_t solution_traj(ss);

    int ss_dim = ss -> get_dimension();
    int cs_dim = cs -> get_dimension();
        
    auto u_goal = cs -> make_point();
    for (int i = 0; i < cs_dim; ++i) u_goal -> at(i) = 0;

    u_goal -> at(1) = 1;
    auto ltv = std::dynamic_pointer_cast<prx::ltv_t>(plant);
    ltv -> linearize(goal_state, u_goal);
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(ss_dim, ss_dim);
    auto q_vec = params["/plant/lqr_Q"].as<std::vector<double>>();
    for (int i = 0; i < ss_dim; ++i) Q(i,i) = q_vec[i];

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(cs_dim, cs_dim);

    // Eigen::VectorXd v_goal(ss_dim);
    // ss -> copy_vector_from_point(v_goal, goal_state);
    lqr_t lqr(ltv, Q, R, "LQR");
    lqr.set_goal(goal_state);
    lqr.compute_K();
    Eigen::MatrixXd K = lqr.get_K();
    std::cout << "A: " << ltv -> get_A() << std::endl;
    std::cout << "B: " << ltv -> get_B() << std::endl;
    std::cout << "K: " << K << std::endl;
    
    ss -> copy_from_point(start_state);
    solution_traj.copy_onto_back(ss);
    do
    {
        lqr.compute_controls();
        cs -> enforce_bounds();
        plant -> propagate(simulation_step);
        std::cout << "[plant] " << plant << std::endl;
        solution_traj.copy_onto_back(ss);

    }
    while(!checker.check()); //&& space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) > 0.01);

    std::cout << "Last state: " << solution_traj.back() << " distance: " << space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) << std::endl;

    params.print();

    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, 
        body_name, ss);

    vis_group -> add_animation(solution_traj, ss, start_state);

    vis_group -> output_html("lqr_num_ctrl.html");

    delete vis_group;

}