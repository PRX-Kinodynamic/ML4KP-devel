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
#include "prx/simulation/system_group.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/gtdynamics/defs.hpp"
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
	world_model_t world_model({plant},{});
	world_model.create_context("context",{plant_name},{});
	auto context = world_model.get_context("context");
	auto sg = context.first;
	const auto ss = sg -> get_state_space();
	const auto cs = sg -> get_control_space();
	auto ss_dim = ss -> get_dimension();

	auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
	auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
	cs -> set_bounds(cs_lb, cs_up);
	std::shared_ptr<plan_t> sln_plan = std::make_shared<plan_t>(cs);
	std::shared_ptr<plan_t> aux_plan = std::make_shared<plan_t>(cs);
	std::shared_ptr<trajectory_t> sln_traj = std::make_shared<trajectory_t>(ss);
	std::shared_ptr<trajectory_t> aux_traj = std::make_shared<trajectory_t>(ss);
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
	auto sigma = params["sigma"].as<double>();

	if (params["init_traj_from_file"].as<bool>())
	{
		std::string traj_name = params["traj_file_in"].as<>();
		std::string plan_name = params["plan_file_in"].as<>();
		initial_traj = std::make_shared<trajectory_t>(ss);
		initial_plan = std::make_shared<plan_t>(cs);
		initial_traj -> from_file(traj_name);
		initial_plan -> from_file(plan_name);

		// init_vals = initialization_trajs_fg_t::init_from_traj(plant, *initial_traj, *initial_plan);
		init_vals = initialization_trajs_fg_t::init_from_plan(sg, start_state, *initial_plan, t_steps, sigma);
		// t_steps = initial_plan -> size() ;
	}
	else if (params["init_traj_constant"].as<bool>())
	{
		t_steps = static_cast<int>(std::ceil(params["traj_duration"].as<int>() / dt));

		auto x = ss -> make_point();
		auto u = cs -> make_point();

// PRX_DEBUG_PRINT
		ss -> copy_point_from_vector(x, params["/plant/constant_state"].as<std::vector<double>>());
		cs -> copy_point_from_vector(u, params["/plant/constant_ctrl"].as<std::vector<double>>());

		init_vals = initialization_trajs_fg_t::constant_trajectory(
			plant, start_state, x, u, t_steps, sigma);

	}
	else if (params["init_traj_linear"].as<bool>())
	{
		t_steps = static_cast<int>(std::ceil(params["traj_duration"].as<int>() / dt));
		// t_steps = params["num_steps"].as<int>();

		init_vals = initialization_trajs_fg_t::linear_trajectory(plant, start_state, goal_state, t_steps);
		fg_utilities::values_to_plan_and_traj(init_vals, sln_traj.get(), sln_plan.get(), fg_params.num_steps);
		fg_utilities::updates_values_from_plan_and_traj(init_vals, plant, *sln_traj, *sln_plan, fg_params.num_steps);
	}
	else
	{
		prx_throw("Values initialization type not supported");
	}
	fg_params.limits_factors = params["limits_factors"].as<bool>();
	fg_params.num_steps = t_steps;
	fg_params.goal_state_as_prior = params["goal_state_as_prior"].as<bool>();
	fg_params.propagation_factors_type = params["propagation_factors_type"].as<int>();
	fg_params.use_goal_factors = params["use_goal_factors"].as<bool>();
	fg_params.initial_state_as_prior = params["initial_state_as_prior"].as<bool>();

	fg_params.goal_factor_params.T = t_steps;
	fg_params.goal_factor_params.theta = params["goal_discount"].as<double>();
	fg_params.use_energy_factors = params["use_energy_factors"].as<bool>();

	// Eigen::Map<Eigen::VectorXd> start_ev(start_v.data(), start_v.size());
	fg_params.goal_factor_params.goal = Eigen::Map<Eigen::VectorXd>(goal_v.data(), goal_v.size());
	auto error_scale = params["/plant/error_scale"].as<std::vector<double>>();
	fg_params.goal_factor_params.error_scale = Eigen::Map<Eigen::VectorXd>(error_scale.data(), error_scale.size());
	
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
	lm_params.setUseFixedLambdaFactor(false);
	lm_params.setDiagonalDamping(false);
	lm_params.setlambdaFactor(4);
	lm_params.setlambdaInitial(1e-6);
	lm_params.setMaxIterations(params["max_iterations"].as<int>());
	// lm_params.setMaxIterations(1);
	// lm_params.setlambdaLowerBound(double value) { lambdaLowerBound = value; }
	// lm_params.setLogFile(out_path + "fg_" + params["/plant/name"].as<>() + "_log.txt");

	// std::cout << "Printing graph: " << std::endl; 
	// graph.print("Printing graph: ", prx::key_formatter);
	// graph.printErrors(init_vals, "NonlinearFactorGraph: ", prx_key_formatter);
	// 
	std::string file_prefix = out_path + "fg_" + params["/plant/name"].as<>();
	fg_logger_t lg(file_prefix + "_log.txt", ' ', "-");
	int outer_iters = params["outer_iters"].as<int>();
	// const gtsam::Values results;
	int total_iters = 0;
	double last_error = 0;
	for (int i = 0; i < outer_iters; ++i)
	{
		gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
		auto results = fg_utilities::optimize_and_log(optimizer, lm_params, lg, total_iters);
		total_iters += optimizer.iterations();
		init_vals = results;
		fg_utilities::values_to_plan_and_traj(init_vals, sln_traj.get(), sln_plan.get(), fg_params.num_steps);
		fg_utilities::values_to_plan_and_traj(init_vals, aux_traj.get(), aux_plan.get(), fg_params.num_steps);

		if (i < outer_iters - 1)
		{
			sg -> propagate(start_state, *sln_plan, *sln_traj);
			fg_utilities::updates_values_from_plan_and_traj(init_vals, plant, *sln_traj, *sln_plan, fg_params.num_steps);

		}
		last_error = optimizer.error();
	}

	// auto results = optimizer.optimize();
	// graph.printErrors(results, "NonlinearFactorGraph: ", prx_key_formatter);

	std::cout << "Done!" << std::endl;
	std::cout << "Final error: " << last_error << std::endl;
	// std::ofstream traj_file;
	// traj_file.open(out_path + "fg_traj.txt");
	double t_elapsed = 0;

	// fg_utilities::values_to_plan_and_traj(results, sln_traj.get(), sln_plan.get(), fg_params.num_steps);
	// std::cout << "initial_plan:\n" << initial_plan.get() << std::endl;
	std::cout << "sln_plan:\n" << sln_plan.get() << std::endl;
	std::cout << "aux_plan:\n" << aux_plan.get() << std::endl;
	sg -> propagate(start_state, *aux_plan, *aux_traj);

	std::cout << "REAL TRAJECTORY\n" << aux_traj << "-~-~-~-~-~-~" << std::endl;
	std::cout << "PROP TRAJECTORY\n" << sln_traj << "-~-~-~-~-~-~" << std::endl;
	aux_traj -> to_file(file_prefix + "_traj_real.txt");
	sln_traj -> to_file(file_prefix + "_traj_prop.txt");
	aux_plan -> to_file(file_prefix + "_prop_plan.txt");
	if (initial_plan != nullptr) initial_plan -> to_file(file_prefix + "_real_plan.txt");
	else 
	{
		// Just erase the "(...)_real_plan.txt" file...
		aux_plan -> clear();
		aux_plan -> to_file(file_prefix + "_real_plan.txt");
	}

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

	vis_group_sln -> reset();
	vis_group_sln -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, *aux_traj, body_name, ss);
	vis_group_sln -> add_animation(*aux_traj, ss, start_state);
	vis_group_sln -> output_html(params["/plant/name"].as<>() + "_fg_real_traj.html");

	delete vis_group;

	return 0;

}