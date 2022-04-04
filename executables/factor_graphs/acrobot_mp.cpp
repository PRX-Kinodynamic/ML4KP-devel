#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/gtdynamics/planning/trajectory_fg.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/gtdynamics/planning/initialization_trajs_fg.hpp"

using namespace prx;

int main(int argc, char* argv[])
{

    auto params = param_loader("executables/factor_graphs/acrobot_mp.yaml", argc, argv);

    simulation_step = 0.01;
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
    std::shared_ptr<trajectory_t> initial_traj = std::make_shared<trajectory_t>(ss);
    std::shared_ptr<trajectory_t> sln_traj = std::make_shared<trajectory_t>(ss);
    std::shared_ptr<plan_t> initial_plan = std::make_shared<plan_t>(cs);
    	
    // std::string traj_name = "lqr";	
    // std::string traj_name = "dirt";	
    std::string traj_name = "rrt";	
    initial_traj -> from_file(out_path + traj_name + "_traj.txt");
    initial_plan -> from_file(out_path + traj_name + "_plan.txt");

    auto dynamics_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.
  	auto objectives_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-2);  // Objectives.
  	auto control_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-1); 

  	// int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.
  	int t_steps = initial_traj -> get_num_states()-1;  // Timesteps.

  	auto start_v  = params["/plant/start_state"].as<std::vector<double>>();
  	auto start_state = ss -> make_point();
  	
  	auto goal_v  = params["/plant/goal_state"].as<std::vector<double>>();
  	auto goal_state = ss -> make_point();


  	ss -> copy_point_from_vector(start_state, start_v);
    ss -> copy_point_from_vector(goal_state, goal_v);

	auto tfg = trajectory_fg_t(plant);
	tfg.set_initial_state(start_v);
	tfg.set_goal_state(goal_v);
	auto graph = tfg.get_fg(t_steps);

  	// auto init_vals = initialization_trajs_fg_t::zeros_trajectory(plant, t_steps, 1);
  	auto init_vals = initialization_trajs_fg_t::init_from_traj(plant, *initial_traj, *initial_plan);
  	// auto init_vals = initialization_trajs_fg_t::linear_trajectory(plant, start_state, goal_state, t_steps);


  	gtsam::LevenbergMarquardtParams lm_params;
  	lm_params.setVerbosityLM("SUMMARY");
  	lm_params.setlambdaUpperBound(1e32);
  	lm_params.setUseFixedLambdaFactor(false);

  	std::cout << "Printing graph: " << std::endl; 
  	graph.print("Printing graph: ", prx_key_formatter);
  	// graph.printErrors(init_vals, "NonlinearFactorGraph: ", prx_key_formatter);

  	gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
  	auto results = optimizer.optimize();

  	std::ofstream traj_file;
  	traj_file.open(out_path + "fg_traj.txt");
  	// traj_file << "t,theta,dtheta,ddtheta,tau"
  	          // << "\n";
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
  	three_js_group_t* vis_group_sln = new three_js_group_t({plant},{});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

    vis_group     -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, *initial_traj, body_name, ss);
    vis_group_sln -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, *sln_traj, body_name, ss);

    vis_group     -> add_animation(*initial_traj, ss, initial_traj -> front());
    vis_group_sln -> add_animation(*sln_traj, ss, initial_traj -> front());

    vis_group     -> output_html("fg_initial_traj.html");
    vis_group_sln -> output_html("fg_sln_traj.html");

    delete vis_group;

  	return 0;

}