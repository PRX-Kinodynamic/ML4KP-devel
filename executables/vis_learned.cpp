#include <fstream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
//#include "prx/simulation/trajectory.hpp"


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


  /*
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

  

  

  // Read the trajectory from file
  prx::trajectory_t learned_trajectory(ss);  // Assuming ss is the state space corresponding to the trajectory
  //learned_trajectory.from_file("/common/home/im316/RL4Insertion_code/CORL/algorithms/learned_RRT_Star_rectangular_16mm_0001_5.txt");
 
  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  //vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  //vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
  // Visualize the loaded trajectory
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, learned_trajectory, body_name, ss);
  vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00",
                           rrt_star_query.goal_region_radius);
  vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
  vis_group->output_html("peg_in_hole_rrt_star.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;

  */


  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
  //vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  //vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);

  // Read and visualize the trajectory from "learned.txt"
  prx::trajectory_t learned_trajectory(ss); // Ensure 'ss' is the correct state space
  //learned_trajectory.from_file("/common/home/im316/RL4Insertion_code/CORL/algorithms/interpolated_poses_quat_ml4kp.txt");
  learned_trajectory.from_file("/common/home/im316/RL4Insertion_code/CORL/algorithms/learned_RRT_Star_rectangular_16mm_0001_Deep_Quat_Old_0_ml4kp.txt");

  prx::space_point_t current = ss->make_point();
  for (unsigned i = 0; i < learned_trajectory.size(); i++)
  {
    current = learned_trajectory.at(i);
    auto pqp_dists = default_obstacle_distance_function(current, ss, context.second);
    double min_dist = PRX_INFINITY;
    for (auto&& dist : pqp_dists.distances)
    {
      if (dist < min_dist)
      {
        min_dist = dist;
      }
    }
    std::cout << "In collision? " << rrt_star_spec.valid_state(current) << " ; Dist = " << min_dist << std::endl;
  }
  
  PRX_DEBUG_VAR_1(learned_trajectory);
  
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, learned_trajectory, body_name, ss);
  //vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00", rrt_star_query.goal_region_radius);
  vis_group->add_animation(learned_trajectory,ss, learned_trajectory.front()); 
  vis_group->output_html("learned_peg_in_hole_rrt_star_Old.html");

  delete vis_group;
  std::cout << "End of program" << std::endl;
  return 0;

}
