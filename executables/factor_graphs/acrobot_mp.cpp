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

using namespace prx;

int main(int argc, char* argv[])
{

    auto params = param_loader("executables/factor_graphs/acrobot_mp.yaml", argc, argv);

    simulation_step = 0.1;
	double T = 5, dt = simulation_step;  // Time horizon (s) and timestep duration (s).
  	int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  	std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    // std::cout << "plant: " << plant_name << std::endl;
    
    auto plant = system_factory_t::create_system(plant_name, plant_path);

    auto dynamics_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);    // Dynamics constraints.
  	auto objectives_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-2);  // Objectives.
  	auto control_model = gtsam::noiseModel::Isotropic::Sigma(1, 1e-1); 

	auto tfg = trajectory_fg_t(plant);
PRX_DEBUG_PRINT
	tfg.set_initial_state({0,0,0,0});
	tfg.set_goal_state({PRX_PI,0,0,0});
PRX_DEBUG_PRINT
	auto graph = tfg.get_fg(t_steps);

PRX_DEBUG_PRINT
  	auto init_vals = tfg.ZeroValuesTrajectory(t_steps);


  	gtsam::LevenbergMarquardtParams lm_params;
  	lm_params.setVerbosityLM("SUMMARY");
PRX_DEBUG_PRINT
  	gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, lm_params);
PRX_DEBUG_PRINT
  	auto results = optimizer.optimize();
PRX_DEBUG_PRINT

  	std::ofstream traj_file;
  	traj_file.open("traj.csv");
  	// traj_file << "t,theta,dtheta,ddtheta,tau"
  	          // << "\n";
  	double t_elapsed = 0;
  	auto ss = plant -> get_state_space();
  	for (int t = 0; t <= t_steps; t++, t_elapsed += dt) 
  	{
  	  	std::vector<gtsam::Key> keys = {};
  	  	int i = 0;
  		// std::string str_SN("Si");
  		// for (int i = 0; i < ss -> get_dimension(); ++i)
  		// {	
  	    keys.push_back(
  	    	prx_symbol_t::state_symbol("Si", i, t)
  	    	);
  	 //    	i++;
  		// }
  	  	
  	  	std::vector<std::string> vals = {std::to_string(t_elapsed)};
  	  	for (auto&& k : keys)
  	  	{
  	  		auto v = results.at<Eigen::VectorXd>(k);
  	  		for (auto e : v)
  	  		{
  	  			vals.push_back(std::to_string(e));
  	  		}
  	  	} 

  	  	traj_file << boost::algorithm::join(vals, " ") << "\n";
  	}
  	traj_file.close();

  	return 0;

}