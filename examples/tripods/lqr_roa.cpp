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
    auto params = param_loader("examples/tripods/lqr_roa.yaml", argc, argv);

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
    auto u_goal = cs -> make_point();

    ss -> copy_point_from_vector(start_state, params["/plant/start_state"].as<std::vector<double>>());
    ss -> copy_point_from_vector(goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    ss -> copy_from_point(start_state);
    
    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());

    trajectory_t solution_traj(ss);
    
    int ss_dim = ss -> get_dimension();
    int cs_dim = cs -> get_dimension();

    cs -> copy_point_from_vector(u_goal, std::vector<double>(cs_dim, 0));

    auto ltv = std::dynamic_pointer_cast<prx::ltv_t>(plant);
    ltv -> linearize(goal_state, u_goal);


    ss -> print_bounds();
    cs -> print_bounds();

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(ss_dim, ss_dim);
    auto q_vec = params["/plant/lqr_Q"].as<std::vector<double>>();
    for (int i = 0; i < ss_dim; ++i) Q(i,i) = q_vec[i];

    // std::cout << "Q:\n" << Q << std::endl;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(cs_dim, cs_dim);

    Eigen::VectorXd v_goal(ss_dim);
    ss -> copy_vector_from_point(v_goal, goal_state);
    lqr_t lqr(ltv, Q, R, "LQR");
    lqr.set_goal(v_goal);
    lqr.compute_K();
    Eigen::MatrixXd K = lqr.get_K();
    // std::cout << "K: " << K << std::endl;
    
    std::ofstream fout_trajs;
    std::ofstream fout_roa;

    int file_id = params["file_id"].as<int>();
    std::string horizon = params["checker_value"].as<>();
    std::string tau_max = std::to_string((int)cs_up[0]);
    std::ostringstream ss_file_id;
    ss_file_id << std::setw(5) << std::setfill('0') << file_id;
    std::string roa_file_name = lib_path + "out/" + plant_name + "/" + horizon + "/lqr_" + std::to_string( (*goal_state)[0] ) + "_" + std::to_string( (*goal_state)[1] ) +  "_" + ss_file_id.str() + "_" + tau_max + "_roa.txt";
    
    std::cout << "saving roa to: " << roa_file_name << std::endl;
    fout_roa.open(roa_file_name.c_str());

    std::cout << "goal: " << goal_state << std::endl;

    params.print();

    bool save_trajs_to_file = params["trajs_to_file"].as<bool>();
    if (save_trajs_to_file)
    {
        std::string trajs_file_name = lib_path + "out/lqr_" + plant_name + "_roa_trajs.txt";
        fout_trajs.open(trajs_file_name.c_str());
    }

    int traj_id = 0;
    double rad = params["goal_region_radius"].as<double>();
    auto df = [&](space_point_t a, space_point_t b)
    {
        return space_t::euclidean_2d(a, b, 0, ss_dim);
    };
    
    auto end_state = ss -> make_point();
    auto trajs_pt = ss -> make_point();
    // auto state = ss -> make_point();

    double trajs_inc = 0.5;
    auto starting_lower_bound = params["/plant/starting_lower_bound"].as<std::vector<double>>();
    auto ending_upper_bound = params["/plant/ending_upper_bound"].as<std::vector<double>>();

    ss -> copy_point_from_vector(trajs_pt, starting_lower_bound);

    double traj_duration = 0;
    auto compute_traj = [&](space_point_t& state)
    {
        traj_duration = 0;
        checker.reset();
        // condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>());
        ss -> copy_from_point(state);
        // trajectory_t solution_traj(ss);
        do
        {
            lqr.compute_controls();
            plant -> propagate(simulation_step);
            ss -> copy_to_point(end_state);
            // std::cout << "[pendulum] " << plant << std::endl;
            // solution_traj.copy_onto_back(ss);
            // if (space_t::euclidean_2d(end_state, goal_state, 0, ss_dim) > rad)
            // {
            traj_duration += simulation_step;
                // break;
            // }
        }
        while(!checker.check() && df(end_state, goal_state) > 0.1 );

        // ss -> copy_to_point(end_state);
        // auto end_state = solution_traj.back();
        // std::cout << traj_id << " ";
        // std::cout << end_state << " ";
        // std::cout << df(end_state, goal_state) << std::endl;

        // fout_roa << traj_id << " ";
        fout_roa << std::setprecision(3) << std::fixed << state << " ";
        fout_roa << (df(end_state, goal_state) <= rad?1:0) << " ";
        fout_roa << traj_duration;
        fout_roa << "\n";
        
    };

        // state_space->set_bounds({-PRX_PI,-2*PRX_PI},{PRX_PI,2*PRX_PI});
    double step_inc = params["state_increment"].as<double>();

    // auto bounds = ss -> get_bounds();
    double total_states = 1;

    space_point_t state = ss -> make_point();
    ss -> copy_point_from_vector(state, starting_lower_bound);
    std::cout << "first pt: " << state << std::endl;
    
    // for (auto b : bounds)
    double l, u;
    for (auto b : prx::zip_iters(starting_lower_bound, ending_upper_bound))
    {
        std::tie(l,u) = prx::unzip(b);

        total_states *= 1. + std::floor((u - l) / step_inc);
    }
    std::cout << "Total states: " << total_states << std::endl;

    progress_bar_t bar(total_states, "");


    double prev_last_dim = state -> at(ss_dim-1);
    do
    {
        // std::cout << "[" << traj_id << "]: " << pt << std::endl;
        if (ss_dim > 2 && state -> at(ss_dim-1) != prev_last_dim)
        {
            file_id++;
            fout_roa.close();
            std::ostringstream ss_file_id;
            ss_file_id << std::setw(5) << std::setfill('0') << file_id;
            // roa_file_name = lib_path + "out/" + plant_name +  "/lqr_" + ss_file_id.str() + "_" + horizon + "_" + tau_max + "_roa.txt";
            roa_file_name = lib_path + "out/" + plant_name + "/" + horizon + "/lqr_" + ss_file_id.str() + "_" + tau_max + "_roa.txt";
            fout_roa.open(roa_file_name.c_str());
            prev_last_dim = state -> at(ss_dim-1);
        }

        compute_traj(state);


        bar.update(traj_id);
        traj_id++;
    }
    while (state_increment(state, step_inc, starting_lower_bound, ending_upper_bound));


    if (save_trajs_to_file)
    {
        std::string trajs_file_name = params["/plant/path"].as<>();
        fout_trajs.close();
    }
    fout_roa.close();

    
}