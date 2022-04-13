#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>


#include "prx/utilities/defs.hpp"

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/loaders/planner_loader.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/gtdynamics/utilities/fg_logger.hpp"
#include "prx/gtdynamics/planning/trajectory_fg.hpp"
#include "prx/gtdynamics/planning/initialization_trajs_fg.hpp"

#include "prx/gtdynamics/utilities/utilities_functions.hpp"

using namespace prx;

int main(int argc, char* argv[])
{

    auto params = param_loader("executables/factor_graphs/acrobot_mp.yaml", argc, argv);

    simulation_step = params["simulation_step"].as<double>();
	double T = 5, dt = simulation_step;  // Time horizon (s) and timestep duration (s).

  	std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    // std::cout << "plant: " << plant_name << std::endl;
    
    auto plant = system_factory_t::create_system(plant_name, plant_path);


    space_t* ss = plant -> get_state_space();
    space_t* cs = plant -> get_control_space();

    auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
    auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
    cs -> set_bounds(cs_lb, cs_up);
    std::shared_ptr<trajectory_t> sln_traj = std::make_shared<trajectory_t>(ss);
    std::shared_ptr<trajectory_t> initial_traj;
    std::shared_ptr<plan_t>       initial_plan;
    
    auto dynamics_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.
  	auto objectives_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-2);  // Objectives.
  	auto control_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-1); 

    auto start_v  = params["/plant/start_state"].as<std::vector<double>>();
    auto start_state = ss -> make_point();
    
    auto goal_v  = params["/plant/goal_state"].as<std::vector<double>>();
    auto goal_state = ss -> make_point();

    ss -> copy_point_from_vector(start_state, start_v);
    ss -> copy_point_from_vector(goal_state, goal_v);

    trajectory_fg_params_t fg_params;
    gtsam::Values init_vals;
    int t_steps = -1;
    if (params["init_traj_from_file"].as<bool>())
    {
        std::string traj_name = params["traj_file_in"].as<>();
        std::string plan_name = params["plan_file_in"].as<>();
        initial_traj = std::make_shared<trajectory_t>(ss);
        initial_plan = std::make_shared<plan_t>(cs);
        initial_traj -> from_file(traj_name);
        initial_plan -> from_file(plan_name);

        t_steps = initial_traj -> get_num_states() - 1;
        init_vals = initialization_trajs_fg_t::init_from_traj(plant, *initial_traj, *initial_plan);
    }
    else if (params["init_traj_constant"].as<bool>())
    {
        t_steps = static_cast<int>(std::ceil(params["traj_duration"].as<int>() / dt));
        auto sigma = params["sigma"].as<double>();

        auto x = ss -> make_point();
        auto u = cs -> make_point();

// PRX_DEBUG_PRINT
        ss -> copy_point_from_vector(x, params["constant_state"].as<std::vector<double>>());
        cs -> copy_point_from_vector(u, params["constant_ctrl"].as<std::vector<double>>());

        init_vals = initialization_trajs_fg_t::constant_trajectory(
            plant, x, u, t_steps, sigma);

    }
    else if (params["init_traj_linear"].as<bool>())
    {
        t_steps = static_cast<int>(std::ceil(params["traj_duration"].as<int>() / dt));
        // t_steps = params["num_steps"].as<int>();

        init_vals = initialization_trajs_fg_t::linear_trajectory(plant, start_state, goal_state, t_steps);
    }
    else
    {
        prx_throw("Values initialization type not supported");
    }
    fg_params.initial_state_as_prior = true;
    fg_params.num_steps = t_steps;
    fg_params.goal_state_as_prior = params["goal_state_as_prior"].as<bool>();
    fg_params.propagation_factors_type = params["propagation_factors_type"].as<int>();
    fg_params.use_goal_factors = params["use_goal_factors"].as<bool>();
    
    if (fg_params.propagation_factors_type == 4)
    {
        init_vals.insert(initialization_trajs_fg_t::init_time_factors(t_steps, params["sigma"].as<double>()));
    }

	auto tfg = trajectory_fg_t(plant);
	tfg.set_initial_state(start_v);
	tfg.set_goal_state(goal_v);

    fg_params.print();
	auto graph = tfg.get_fg(fg_params);

  	gtsam::LevenbergMarquardtParams lm_params;
  	lm_params.setVerbosityLM("SUMMARY");
  	lm_params.setlambdaUpperBound(1e32);
  	lm_params.setUseFixedLambdaFactor(true);
    lm_params.setDiagonalDamping(false);
    lm_params.setlambdaFactor(4);
    lm_params.setlambdaInitial(1e-6);
    lm_params.setMaxIterations(params["max_iterations"].as<int>());
    // lm_params.setMaxIterations(1);
    // lm_params.setlambdaLowerBound(double value) { lambdaLowerBound = value; }
    // lm_params.setLogFile(out_path + "fg_" + params["/plant/name"].as<>() + "_log.txt");

  	// std::cout << "Printing graph: " << std::endl; 
  	// graph.print("Printing graph: ", prx_key_formatter);
  	// graph.printErrors(init_vals, "NonlinearFactorGraph: ", prx_key_formatter);
    fg_logger_t lg(out_path + "fg_" + params["/plant/name"].as<>() + "_log.txt", ' ', "-");

  	gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);

  	// auto results = optimizer.optimize();
    auto results = fg_utilities::optimize_and_log(optimizer, lm_params, lg);
    // graph.printErrors(results, "NonlinearFactorGraph: ", prx_key_formatter);

    std::cout << "Done!" << std::endl;
  	std::ofstream traj_file;
  	traj_file.open(out_path + "fg_traj.txt");
  	double t_elapsed = 0;
  	// auto ss = plant -> get_state_space();
  	auto aux_pt = ss -> make_point();
  	for (int t = 0; t <= t_steps; t++, t_elapsed += dt) 
  	{
  	  	int j = 0;

  	    auto k = prx_symbol_t::state_symbol(t);
  	  	auto v = results.at<Eigen::VectorXd>(k);

  	  	std::ostringstream out;
    	out.precision(4);

  	  	std::vector<std::string> vals = {std::to_string(t_elapsed)};
  	  	ss -> copy_from_vector(v);
  	  	for (auto e : v)
  	  	{
  	  		out << std::fixed << e << "\t";
  	  		// vals.push_back(std::to_string(e));
  	  	}

  	  	sln_traj -> copy_onto_back(ss);
  	  	
  	  	if (t < t_steps)
  	  	{
  	  		auto u = results.at<Eigen::VectorXd>(prx_symbol_t::control_symbol(t));
  	  		for (auto ui : u)
  	  		{
    			out << std::fixed << ui << "\t";
  	  			// vals.push_back(std::to_string(ui));
  	  		}
  	  	}

  	  	// traj_file << boost::algorithm::join(vals, "\t") << "\n";
  	  	traj_file << out.str() << "\n";
  	}
  	traj_file.close();

    three_js_group_t* vis_group = new three_js_group_t({plant},{});
    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    if (params["init_traj_from_file"].as<bool>())
    {
        vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, *initial_traj, body_name, ss);
        vis_group -> add_animation(*initial_traj, ss, initial_traj -> front());
        vis_group -> output_html(params["/plant/name"].as<>() + "_fg_initial_traj.html");
    }
  	three_js_group_t* vis_group_sln = new three_js_group_t({plant},{});


    vis_group_sln -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, *sln_traj, body_name, ss);

    vis_group_sln -> add_animation(*sln_traj, ss, start_state);

    vis_group_sln -> output_html(params["/plant/name"].as<>() + "_fg_sln_traj.html");

    delete vis_group;

  	return 0;

}