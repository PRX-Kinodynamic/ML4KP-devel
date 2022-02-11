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

#include <sstream>
#include <string>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/tripods/pendulum_lqr_roa.yaml", argc, argv);

    simulation_step = params["simulation_step"].as<double>();


    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t<> world_model({plant},{});
    world_model.create_context("context",{plant_name},{});
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

    auto compute_traj = [&](space_point_t& state)
    {
        checker.reset();
        ss -> copy_from_point(state);
        do
        {
            lqr.compute_controls();
            plant -> propagate(simulation_step);
            ss -> copy_to_point(end_state);

        }
        while(!checker.check() && df(end_state, goal_state) > 0.1 );

    };


    double step_inc = params["state_increment"].as<double>();

    // auto bounds = ss -> get_bounds();
    double total_states = 1;

    space_point_t state = ss -> make_point();
    ss -> copy_point_from_vector(state, starting_lower_bound);
    std::cout << "first pt: " << state << std::endl;
    

    progress_bar_t bar(total_states, "");

    std::string line;
    int line_num = 0;
    // std::string file_name = lib_path + "out/pendulum/sos_roa_c1.250000e+00.txt";
    std::string file_name = lib_path + params["est_roa_file"].as<std::string>();
    
    std::cout << "Using file: " << file_name << std::endl;
    std::ifstream infile(file_name);

    int true_pos = 0;
    int true_neg = 0;
    int false_pos = 0;
    int false_neg = 0;
    bool reached, safe;
    total_states = 0;

    while (std::getline(infile, line))
    {
        // std::cout << "line: " << line_num << std::endl;
        std::istringstream iss(line);
        double th, thdot, safe_val;
        if (iss >> th >> thdot >> safe_val)
        {
            line_num++;
            (*state)[0] = th;
            (*state)[1] = thdot;
            compute_traj(state);

            reached = df(end_state, goal_state) <= 0.1;
            safe = safe_val > 0;

            true_pos  = true_pos  + (int)(reached && safe);
            true_neg  = true_neg  + (int)((!reached) && (!safe));
            false_pos = false_pos + (int)((!reached) && safe);
            false_neg = false_neg + (int)(reached && (!safe));

            total_states++;
            // printf("%.2f %.2f %.2f\n", th, thdot, safe);
            // if ( line_num == 10 ) break;
        } // error

    }

    std::cout << "Total states: " << total_states << std::endl;
    std::cout << "true_pos: " << true_pos << "\t---\t" << 100 * true_pos / total_states << std::endl;
    std::cout << "true_neg: " << true_neg << "\t---\t" << 100 * true_neg / total_states << std::endl;
    std::cout << "false_pos: " << false_pos << "\t---\t" << 100 * false_pos / total_states << std::endl;
    std::cout << "false_neg: " << false_neg << "\t---\t" << 100 * false_neg / total_states << std::endl;

    return 0;
}