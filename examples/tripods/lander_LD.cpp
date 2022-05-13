#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/condition_check.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

using namespace prx;

space_point_t start_state = nullptr;
space_point_t goal_state = nullptr;

controller_ptr_t create_controller(const system_ptr_t& _sys_ptr, param_loader& params)
{
    auto ss = _sys_ptr -> get_state_space();
    auto cs = _sys_ptr -> get_control_space();
    auto ss_dim = ss -> get_dimension();
    auto cs_dim = cs -> get_dimension();


    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(ss_dim, ss_dim);
    auto q_vec = params["/plant/lqr_Q"].as<std::vector<double>>();
    for (int i = 0; i < ss_dim; ++i) Q(i,i) = q_vec[i];

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(cs_dim, cs_dim);
    auto r_vec = params["/plant/lqr_R"].as<std::vector<double>>();
    // PRX_DEBUG_ITERABLE("lqr_R: ", r_vec)
    for (int i = 0; i < cs_dim; ++i) R(i,i) = r_vec[i];
    std::cout << "R: " << R.transpose() << std::endl;

    Eigen::VectorXd v_goal(ss_dim);
    ss -> copy_vector_from_point(v_goal, goal_state);
    std::shared_ptr<lqr_t> lqr = std::make_shared<lqr_t>(_sys_ptr, Q, R, "LQR");
    lqr -> set_goal(v_goal);
    lqr -> compute_K();
    Eigen::MatrixXd K = lqr -> get_K();
    std::cout << "K: " << K << std::endl;
    // K(0,0) = K(0,0) * 0.25;
    lqr -> set_K(K);
    std::cout << "K: " << K << std::endl;
    
    return lqr;
}

int main(int argc, char* argv[])
{
    auto params = param_loader("plants/lander_LD.yaml", argc, argv);

	simulation_step = 0.01;
    init_random(231192);

    auto obstacles = load_obstacles("environments/empty.yaml");
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;
    PRX_DEBUG_PRINT

    std::string plant_name = params["name"].as<>();
    std::string plant_path = params["path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");
    PRX_DEBUG_PRINT

    world_model_t world_model({plant},{obstacle_list});
    world_model.create_context("context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("context");

    PRX_DEBUG_PRINT
    const auto ss = context.first -> get_state_space();
    const auto cs = context.first -> get_control_space();
    const auto ps = plant -> get_parameter_space();

    PRX_DEBUG_PRINT
    auto lower_bounds = params["state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["state_space_upper_bound"].as<std::vector<double>>();
    ss -> set_bounds(lower_bounds, upper_bounds);

    auto cs_lb = params["control_space_lower_bound"].as<std::vector<double>>();
    auto cs_up = params["control_space_upper_bound"].as<std::vector<double>>();
    cs -> set_bounds(cs_lb, cs_up);

    PRX_DEBUG_PRINT
    if (ps -> get_dimension() > 0)
    {
        ps -> copy_from_vector(params["parameters"].as<std::vector<double>>());
        std::cout << "params: " << ps -> print_memory(2) << std::endl;
    }
    PRX_DEBUG_PRINT

    start_state = ss -> make_point();
	goal_state = ss -> make_point();
    auto u_goal = cs -> make_point();

    ss -> copy_point_from_vector(start_state, params["start_state"].as<std::vector<double>>());
    ss -> copy_point_from_vector(goal_state, params["goal_state"].as<std::vector<double>>());
    

    condition_check_t checker("sim_time", 10);

    trajectory_t solution_traj(ss);

    // auto ltv = std::dynamic_pointer_cast<prx::ltv_t>(plant);

    int ss_dim = ss -> get_dimension();
    int cs_dim = cs -> get_dimension();
    
    // auto lqr = create_controller( plant,  params);
    controller_ptr_t ctrl = std::make_shared<lander_meditch_ctrl_t>(plant, "ctrl");
    ss -> copy_from_point(start_state);
    solution_traj.copy_onto_back(ss);
    
    do
    {
        std::cout << "[plant] " << plant << std::endl;
        ctrl -> compute_controls();
        cs -> enforce_bounds();
        plant -> propagate(simulation_step);
        solution_traj.copy_onto_back(ss);

    }
    while(!checker.check()); //&& space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) > 0.01);

    std::cout << "Last state: " << solution_traj.back() << " distance: " << space_t::euclidean_2d(solution_traj.back(), goal_state, 0, ss_dim) << std::endl;

    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["name"].as<>() + "/" + params["vis_body"].as<>();

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, solution_traj, 
        body_name, ss);

    vis_group -> add_animation(solution_traj, ss, start_state);

    vis_group -> output_html("lander_ctrl.html");

    delete vis_group;

    params.print();
}