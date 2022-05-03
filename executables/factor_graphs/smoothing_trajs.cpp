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
	auto params = param_loader("executables/factor_graphs/smoothing_trajs.yaml", argc, argv);

	simulation_step = params["simulation_step"].as<double>();

	std::string plant_name = params["/plant/name"].as<>();
	std::string plant_path = params["/plant/path"].as<>();

    auto obstacles = load_obstacles(params["environment"].as<>());

    auto obstacle_list = obstacles.second;
    auto obstacle_names = obstacles.first;

	auto plant = system_factory_t::create_system(plant_name, plant_path);
	world_model_t world_model({plant},{obstacle_list});
	world_model.create_context("context",{plant_name},{obstacle_names});
	auto context = world_model.get_context("context");
	auto sg = context.first;
	const auto ss = sg -> get_state_space();
	const auto cs = sg -> get_control_space();
	auto ss_dim = ss -> get_dimension();
	auto cs_dim = cs -> get_dimension();

	auto cs_lb = params["/plant/control_space_lower_bound"].as<std::vector<double>>();
	auto cs_up = params["/plant/control_space_upper_bound"].as<std::vector<double>>();
	cs -> set_bounds(cs_lb, cs_up);

	// std::string file_prefix = out_path + "fg_" + params["/plant/name"].as<>();
	std::string traj_in_name = params["traj_file_in"].as<>();

	trajectory_t traj_in(ss);
	trajectory_t traj_in0(ss);
	trajectory_t traj_out(ss);
	trajectory_t traj_aux(ss);
	traj_in0.from_file(traj_in_name);
	// traj_in.from_file(traj_in_name);

	traj_in.copy_onto_back(traj_in0.front());

	for (unsigned i = 1; i < traj_in0.size()-1; ++i)
	// for (unsigned i = 0; i < traj_in0.size(); ++i)
	{
		if (uniform_random(0,1) > 0.05) continue;
		traj_in.copy_onto_back(traj_in0[i]);
	}
	traj_in.copy_onto_back(traj_in0.back());

	std::cout << "[IN] traj size original: " << traj_in0.size() << std::endl;
 	std::cout << "[IN] traj size purged: " << traj_in.size() << std::endl;

	plan_t plan_in(cs);
	plan_t plan_out(cs);
	plan_t plan_aux(cs);

	int num_steps = traj_in.size()-1;
	for (int i = 0; i < num_steps; ++i)
	{
		plan_in.copy_onto_back(Eigen::VectorXd::Zero(cs_dim), simulation_step);
	}

	auto start_state = ss -> make_point();
	auto goal_state = ss -> make_point();
	ss -> copy_point(start_state, traj_in.front());
	ss -> copy_point(goal_state, traj_in.back());

 	std::cout << "start_state: " << start_state << std::endl;
 	std::cout << "goal_state: " << goal_state << std::endl;

	std::vector<double> start_v;
 	std::vector<double> goal_v ;

 	// PRX_DEBUG_ITERABLE("start_v", start_v);

 	ss -> copy_vector_from_point(start_v, start_state);
 	ss -> copy_vector_from_point(goal_v, goal_state);
 	// PRX_DEBUG_ITERABLE("start_v", start_v);

	trajectory_fg_params_t fg_params;

	auto init_vals = initialization_trajs_fg_t::init_from_traj(plant, traj_in, plan_in);

	auto dynamics_model = gtsam::noiseModel::Isotropic::Sigma(ss_dim, params["sigma"].as<double>());
	fg_utilities::add_noise(init_vals, "Xi", dynamics_model);


	fg_params.limits_factors = params["limits_factors"].as<bool>();
	fg_params.num_steps = num_steps;
	fg_params.goal_state_as_prior = params["goal_state_as_prior"].as<bool>();
	fg_params.propagation_factors_type = params["propagation_factors_type"].as<int>();
	fg_params.use_goal_factors = params["use_goal_factors"].as<bool>();
	fg_params.initial_state_as_prior = params["initial_state_as_prior"].as<bool>();

	fg_params.goal_factor_params.T = num_steps;
	fg_params.goal_factor_params.theta = params["goal_discount"].as<double>();
	fg_params.use_energy_factors = params["use_energy_factors"].as<bool>();

	// fg_params.goal_factor_params.goal = Eigen::Map<Eigen::VectorXd>(goal_v.data(), goal_v.size());
	fg_params.goal_factor_params.error_scale = Eigen::VectorXd::Zero(ss_dim);
	for (int i = 0; i < ss_dim; ++i) fg_params.goal_factor_params.error_scale[i] = 1;

	fg_params.print();
	auto tfg = trajectory_fg_t(plant);
	
	tfg.set_initial_state(start_v);
	tfg.set_goal_state(goal_v);

	auto graph = tfg.get_smoothing_fg(fg_params);

	gtsam::LevenbergMarquardtParams lm_params;
	lm_params.setVerbosityLM("SUMMARY");
	lm_params.setlambdaUpperBound(1e32);
	lm_params.setUseFixedLambdaFactor(false);
	lm_params.setDiagonalDamping(false);
	lm_params.setlambdaFactor(4);
	lm_params.setlambdaInitial(1e-6);
	lm_params.setMaxIterations(params["max_iterations"].as<int>());

 	PRX_DEBUG_PRINT

	std::string file_prefix = out_path + "smoothing_" + params["/plant/name"].as<>();
	fg_logger_t lg(file_prefix + "_log.txt", ' ', "-");
	int outer_iters = params["outer_iters"].as<int>();

	int total_iters = 0;
	double last_error = 0;

	for (int i = 0; i < outer_iters; ++i)
	{
		gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
		auto results = fg_utilities::optimize_and_log(optimizer, lm_params, lg, total_iters);
		total_iters += optimizer.iterations();
		init_vals = results;
		fg_utilities::values_to_plan_and_traj(init_vals, &traj_out, &plan_out, fg_params.num_steps);
		fg_utilities::values_to_plan_and_traj(init_vals, &traj_aux, &plan_aux, fg_params.num_steps);

		if (i < outer_iters - 1)
		{
			sg -> propagate(start_state, plan_out, traj_out);
			fg_utilities::updates_values_from_plan_and_traj(init_vals, plant, traj_out, plan_out, fg_params.num_steps);

		}
		last_error = optimizer.error();
	}

 	PRX_DEBUG_PRINT
	sg -> propagate(start_state, plan_aux, traj_aux);

	traj_out.to_file(file_prefix + "_traj_out.txt");

	std::cout << "[OUT] start_state: " << traj_out.front() << std::endl;
 	std::cout << "[OUT] goal_state: " << traj_out.back() << std::endl;

	three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
	std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

	vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_in, body_name, ss);
	vis_group -> add_animation(traj_in, ss, start_state);
	vis_group -> output_html(params["/plant/name"].as<>() + "_smoothing_in_traj.html");

	vis_group -> reset();
	vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj_out, body_name, ss);
	vis_group -> add_animation(traj_out, ss, start_state);
	vis_group -> output_html(params["/plant/name"].as<>() + "_smoothing_out_traj.html");


	delete vis_group;

	return 0;
}