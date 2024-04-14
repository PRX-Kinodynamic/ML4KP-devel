#include <fstream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

// Hypothetical conversion function
prx::trajectory_t convertToTrajectory(const std::vector<std::vector<double>>& vec) {
    // Assuming prx::trajectory_t can be directly constructed from std::vector<std::vector<double>>
    return prx::trajectory_t(vec.begin(), vec.end());
}

// Function to read trajectory from a file
std::vector<std::vector<double>> read_trajectory(const std::string& filename) 
{
    std::vector<std::vector<double>> trajectory;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) 
    {
        std::istringstream iss(line);
        std::vector<double> point;
        double value;

        while (iss >> value) 
        {
            // Print value
            std::cout << value << std::endl;
            point.push_back(value);
        }

        if (!point.empty()) 
        {
            // Print "Point" and the point
            std::cout << "Point: " << std::endl;
            for (const auto& val : point) 
            {
                std::cout << val << " "; // Print each value in the inner vector followed by a space
            }
            trajectory.push_back(point);
        }
    }
    return trajectory;
}



int main(int argc, char* argv[])
{
  
  prx::param_loader params("executables/peg_in_hole.yaml", argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  prx::PairNameObstacles obstacles{ prx::load_obstacles(params["environment"].as<>()) };
  const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  const std::vector<std::string> obstacle_names{ obstacles.first };

  const std::string plant_name{ params["/plant/name"].as<>() };
  const std::string plant_path{ params["/plant/path"].as<>() };
  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_path) };
  prx_assert(plant != nullptr, "Plant is nullptr!");

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sys_group{ context.first };

  prx::space_t* ss{ sys_group->get_state_space() };
  prx::space_t* cs{ sys_group->get_control_space() };
  prx::space_t* ps{ sys_group->get_parameter_space() };

  prx::rrt_star_t rrt_star(params["/planner/name"].as<>());
  prx::rrt_star_specification_t rrt_star_spec(context.first, context.second);

  rrt_star_spec.distance_function = [&](const prx::space_point_t& x, const prx::space_point_t& y) {
    double dist{ 0.0 };
    dist = (Vec(x).head(3) - Vec(y).head(3)).norm();
    const double q0_w{ x->at(3 + 0) };
    const double q0_x{ x->at(3 + 1) };
    const double q0_y{ x->at(3 + 2) };
    const double q0_z{ x->at(3 + 3) };
    const double q1_w{ y->at(3 + 0) };
    const double q1_x{ y->at(3 + 1) };
    const double q1_y{ y->at(3 + 2) };
    const double q1_z{ y->at(3 + 3) };
    const Eigen::Quaterniond q0(q0_w, q0_x, q0_y, q0_z);
    const Eigen::Quaterniond q1(q1_w, q1_x, q1_y, q1_z);
    dist += q0.angularDistance(q1);
    // const Eigen::Matrix3d R0{ q0.toRotationMatrix() };
    // const Eigen::Matrix3d R1{ q1.toRotationMatrix() };
    // dist += std::acos(((R1.transpose() * R0).trace() - 1) / 2.0);
    return dist;
  };

  rrt_star_spec.min_control_steps = params["plant/min_steps"].as<int>();
  rrt_star_spec.max_control_steps = params["/plant/max_steps"].as<int>();

  prx::rrt_star_query_t rrt_star_query(ss, cs);

  rrt_star_spec.eta_min = params["/planner/eta_min"].as<double>();
  rrt_star_spec.eta_max = params["/planner/eta_max"].as<double>();

  const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

  rrt_star_query.goal_region_radius = params["/planner/goal_region_radius"].as<double>();

  // Alternatively, change the goal_check function
  rrt_star_query.goal_check = [&](prx::space_point_t pt)  // no-lint
  {
    const double dist_to_goal{ rrt_star_spec.distance_function(pt, rrt_star_query.goal_state) };
    return dist_to_goal < rrt_star_query.goal_region_radius;
  };

  rrt_star_query.get_visualization = params["visualize"].as<bool>();

  rrt_star.link_and_setup_spec(&rrt_star_spec);
  rrt_star.preprocess();
  rrt_star.link_and_setup_query(&rrt_star_query);

  const std::string out_dir{ params["/out/dir"].as<>() };
  const std::string file_prefix{ params["/out/file_prefix"].as<>() };
  rrt_star.from_files(file_prefix, out_dir);

  std::vector<double> goal{ params["/plant/goal_state"].as<std::vector<double>>() };
  rrt_star.connect_goal();
  rrt_star.fulfill_query();

  // params.print();

  // rrt_star.to_files(file_prefix, out_dir);
  // rrt_star_query.solution_traj.to_file(out_dir + "/" + file_prefix + "_sln_traj.txt");

  // aorrt_query.solution_traj.to_file();

  // Read the learned trajectory and visualize it
  std::vector<std::vector<double>> learned_trajectory = read_trajectory("/common/im316/home/RL4Insertion_code/CORL/algortihms/learned_RRT_Star_rectangular_16mm_0001_updated.txt");
  
  // Print "I read traj"
  std::cout << "I read traj" << std::endl;

  // Printing each point in learned_trajectory
  for (const auto& pt : learned_trajectory) 
  {
    for (const auto& val : pt) 
    {
        std::cout << val << " "; // Print each value in the inner vector followed by a space
    }
    std::cout << std::endl; // New line after each inner vector
  }

  // Print the solution trajectory
  std::cout << "Solution trajectory: " << std::endl;
  for (const auto& pt : rrt_star_query.solution_traj)
  {
    std::cout << pt << std::endl;
  }
  //sleep(10);

  /*
  // Change the following lines in the main function
  for (int i = 0; i < learned_trajectory.size(); i++)
  {
    rrt_star_query.solution_traj[i][0] = learned_trajectory[i][0];
    rrt_star_query.solution_traj[i][1] = learned_trajectory[i][1];
    rrt_star_query.solution_traj[i][2] = learned_trajectory[i][2];
    rrt_star_query.solution_traj[i][3] = learned_trajectory[i][3];
    rrt_star_query.solution_traj[i][4] = learned_trajectory[i][4];
    rrt_star_query.solution_traj[i][5] = learned_trajectory[i][5];
    rrt_star_query.solution_traj[i][6] = learned_trajectory[i][6];
  }
  */


  /*
  for (unsigned int i = 0; i < learned_trajectory.size(); i++)
  {
      auto& traj_point = rrt_star_query.solution_traj[i]; // traj_point is a reference to space_point_t
      // Ensure traj_point can be indexed or has methods to set values
      traj_point[0] = learned_trajectory[i][0];
      traj_point[1] = learned_trajectory[i][1];
      traj_point[2] = learned_trajectory[i][2];
      traj_point[3] = learned_trajectory[i][3];
      traj_point[4] = learned_trajectory[i][4];
      traj_point[5] = learned_trajectory[i][5];
      traj_point[6] = learned_trajectory[i][6];
  }
  */


  // Example conversion function (Adjust based on actual prx::trajectory_t definition)
  prx::trajectory_t convertToTrajectory(const std::vector<std::vector<double>>& learned_trajectory) {
      prx::trajectory_t traj;
      // Conversion logic here
      return traj;
  }

  // In your main function
  prx::trajectory_t converted_trajectory = convertToTrajectory(learned_trajectory);


  /*

  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();


  // Replace the problematic lines with
  //vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, converted_trajectory, body_name, ss);
  //vis_group->add_animation(converted_trajectory, ss, converted_trajectory[0]); // Assuming converted_trajectory[0] is valid

  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, converted_trajectory, body_name, ss);
  //vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  //vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, learned_trajectory, body_name, ss);
  vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00",rrt_star_query.goal_region_radius);
  //vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
  //vis_group->add_animation(learned_trajectory, ss, learned_trajectory[0]);
  vis_group->add_animation(converted_trajectory, ss, converted_trajectory[0]); // Assuming converted_trajectory[0] is valid
  vis_group->output_html("learned_peg_in_hole_rrt_star.html");

  delete vis_group;
  std::cout << "End of program" << std::endl;

  */


  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });
  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, converted_trajectory, body_name, ss);
  vis_group->add_animation(converted_trajectory, ss, converted_trajectory[0]);  
  // Assuming converted_trajectory[0] is valid
  vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00",rrt_star_query.goal_region_radius);
  
  vis_group->output_html("learned_peg_in_hole_rrt_star.html");

  delete vis_group;
  std::cout << "End of program" << std::endl;


}
