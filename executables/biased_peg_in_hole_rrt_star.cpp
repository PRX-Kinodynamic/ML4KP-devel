#include <fstream>

#include <random>
#include <cmath>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For Quaternion and AngleAxis and For Quaternion and Transform
 
#include <vector>
#include <cmath>
#include <iostream>

#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

// Define a nominal point type for clarity
typedef Eigen::Vector3d NominalPoint;


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

  rrt_star_spec.distance_function = [&](const prx::space_point_t& x, const prx::space_point_t& y) 
  {
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

  /*
  rrt_star_spec.distance_function = [&](const prx::space_point_t& x, const prx::space_point_t& y) 
  {

    double dist{ 0.0 };

    //
    //
    //
    // x,y are two SE(3) poses (for the Peg and Hole correspondingly) in the format: x, y, z, qw, qx, qy, qz
    // We take 3(?) sampled nominal 3-dim points X_1_P, X_2_P, X_3_P (different per geometry) for the Peg
    // and  X_1_H, X_2_H, X_3_H for the Hole correspondingly.
    //
    // They are non-colinear and sampled along the Principal Axes of the Peg and Hole
    //
    // Based on x,y, we calculate the Homogenous Pose Transformations H_1, H_2
    // and we apply them to X_1_P, X_2_P, X_3_P and X_1_H, X_2_H, X_3_H correspondingly.
    // Now, we got X_1_P_Transformed, X_2_P_Transformed, X_3_P_Transformed and X_1_H_Transformed, X_2_H_Transformed, X_3_H_Transformed correspondingly.
    // We stack them into x_transformed and y_transformed, correspondingly.
    // Now, we calculate the Euclidean Distance (C-Dist (?) ) as follows:

    //

    dist = (Vec(x_transformed).head(9) - Vec(y_transformed).head(9)).norm();
    
    return dist;
  };
  */


  /*
  
  // ChatGPT says

  // Assuming Vec is some kind of placeholder for Eigen vector types
// Let's use Eigen::VectorXd or Eigen::Vector3d as needed

// Assume prx::space_point_t is somehow compatible with Eigen::VectorXd
// Here's how you might define the distance function:
rrt_star_spec.distance_function = [&](const Eigen::VectorXd& x, const Eigen::VectorXd& y) 
{
    double dist{ 0.0 };

    // Define the nominal points for the Peg and Hole
    std::vector<NominalPoint> nominalPointsPeg = {NominalPoint(1, 0, 0), NominalPoint(0, 1, 0), NominalPoint(0, 0, 1)};
    std::vector<NominalPoint> nominalPointsHole = {NominalPoint(-1, 0, 0), NominalPoint(0, -1, 0), NominalPoint(0, 0, -1)};
    
    // Extract pose information (position + quaternion) for both x and y
    Eigen::Vector3d pos_x = x.head<3>();
    Eigen::Quaterniond quat_x(x[3], x[4], x[5], x[6]);
    Eigen::Vector3d pos_y = y.head<3>();
    Eigen::Quaterniond quat_y(y[3], y[4], y[5], y[6]);

    // Construct the homogeneous transformation matrices
    Eigen::Transform<double, 3, Eigen::Affine> H_x(quat_x);
    H_x.pretranslate(pos_x);
    Eigen::Transform<double, 3, Eigen::Affine> H_y(quat_y);
    H_y.pretranslate(pos_y);

    // Apply transformations to nominal points
    std::vector<NominalPoint> transformedPointsPeg, transformedPointsHole;
    for(const auto& p : nominalPointsPeg)
        transformedPointsPeg.push_back(H_x * p);
    for(const auto& p : nominalPointsHole)
        transformedPointsHole.push_back(H_y * p);

    // Assuming Vec was a placeholder, we replace Vec with direct Eigen usage
    // Calculate distance between all transformed points (assuming this is the desired logic)
    for(size_t i = 0; i < transformedPointsPeg.size(); ++i)
    {
        dist += (transformedPointsPeg[i] - transformedPointsHole[i]).norm();
    }

    // Return the computed distance
    return dist;
};

    
*/


  rrt_star_spec.min_control_steps = params["plant/min_steps"].as<int>();
  rrt_star_spec.max_control_steps = params["/plant/max_steps"].as<int>();

  prx::rrt_star_query_t rrt_star_query(ss, cs);
  rrt_star_query.start_state = context.first->get_state_space()->make_point();
  rrt_star_query.goal_state = context.first->get_state_space()->make_point();

  rrt_star_spec.eta_min = params["/planner/eta_min"].as<double>();
  rrt_star_spec.eta_max = params["/planner/eta_max"].as<double>();

  const std::vector<double> ss_lower_bounds{ params["/plant/state_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> ss_upper_bounds{ params["/plant/state_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> cs_lower_bounds{ params["/plant/control_space/lower_bound"].as<std::vector<double>>() };
  const std::vector<double> cs_upper_bounds{ params["/plant/control_space/upper_bound"].as<std::vector<double>>() };

  const std::vector<double> ps_values{ params["/plant/parameter_space/values"].as<std::vector<double>>() };

  ss->set_bounds(ss_lower_bounds, ss_upper_bounds);
  cs->set_bounds(cs_lower_bounds, cs_upper_bounds);

  ps->copy_from(ps_values);

  ss->copy(rrt_star_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
  ss->copy(rrt_star_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

  rrt_star_query.goal_region_radius = params["/planner/goal_region_radius"].as<double>();

  // Alternatively, change the goal_check function
  rrt_star_query.goal_check = [&](prx::space_point_t pt)  // no-lint
  {
    const double dist_to_goal{ rrt_star_spec.distance_function(pt, rrt_star_query.goal_state) };
    return dist_to_goal < rrt_star_query.goal_region_radius;
  };
  

  // Assuming prx::global_generator is already initialized somewhere in your code
  std::normal_distribution<double> r_dist{5.0, 1.5};
  std::normal_distribution<double> z_dist{75.0, 10.0};
  std::uniform_real_distribution<double> coin_flip{0.0, 1.0};
  std::uniform_real_distribution<double> angle_dist(-45 * M_PI / 180, 45 * M_PI / 180); // +/- 45 degrees in radians

  // Simulate coin flip
  //if (coin_flip(prx::global_generator) < 0.5)
  if(1)
  {
      // Heads: Apply both biased sampling for position and restricted sampling for orientation
      rrt_star_spec.sample_state = [&](prx::space_point_t& s)
      {
          ss->sample(s); // Sample a state (ensure this method populates position and orientation in s)

          // Apply biased sampling for position
          const double th = prx::uniform_random(-prx::constants::pi, prx::constants::pi);
          const double r = r_dist(prx::global_generator);
          s->at(0) = r * std::cos(th);
          s->at(1) = r * std::sin(th);
          s->at(2) = z_dist(prx::global_generator);

          // Apply restricted sampling for orientation
          double theta = angle_dist(prx::global_generator); // Sample the rotation angle within +/- 10 degrees
          Eigen::Vector3d axis(0.0, 0.0, 1.0); // Restrict rotation around the Z-axis
          Eigen::AngleAxisd angleAxis(theta, axis);
          Eigen::Quaterniond q(angleAxis);
          s->at(3) = q.w();
          s->at(4) = q.x();
          s->at(5) = q.y();
          s->at(6) = q.z();
      };
  }

  else
  
  {
      // Tails: Apply only restricted sampling for orientation
      rrt_star_spec.sample_state = [&](prx::space_point_t& s)
      {
          ss->sample(s); // Sample a state
          
          // Apply restricted sampling for orientation
          double theta = angle_dist(prx::global_generator); // Sample the rotation angle within +/- 10 degrees
          Eigen::Vector3d axis(0.0, 0.0, 1.0); // Restrict rotation around the Z-axis
          Eigen::AngleAxisd angleAxis(theta, axis);
          Eigen::Quaterniond q(angleAxis);
          s->at(3) = q.w();
          s->at(4) = q.x();
          s->at(5) = q.y();
          s->at(6) = q.z();
      };
  }
  
  const std::string out_dir{ params["/out/dir"].as<>() };
  const std::string file_prefix{ params["/out/file_prefix"].as<>() };

  rrt_star_query.get_visualization = params["visualize"].as<bool>();

  rrt_star.link_and_setup_spec(&rrt_star_spec);
  rrt_star.preprocess();
  rrt_star.link_and_setup_query(&rrt_star_query);
  
  if (params["grow_tree"].as<bool>())
  {
    prx::condition_check_t checker(params["/planner/checker_type"].as<>(), params["/planner/checker_value"].as<int>());

    rrt_star.resolve_query(&checker);
  }

  if (params["query_tree"].as<bool>())
  {
	  PRX_DEBUG_VAR_2(file_prefix, out_dir);
    rrt_star.from_files(file_prefix, out_dir);
    rrt_star.connect_goal();
  }
  rrt_star.fulfill_query();

  // params.print();

  if (params["tree_to_files"].as<bool>())
  {
    rrt_star.to_files(file_prefix, out_dir);
    rrt_star.costs_to_files(file_prefix, out_dir);
    rrt_star_query.solution_traj.to_file(out_dir + "/" + file_prefix + "_sln_traj.txt");
  }

  // aorrt_query.solution_traj.to_file();

  // Visualization
  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();

  vis_group->set_floor_plane(std::vector<double>({ 0, 0, -3 }), std::vector<double>({ 0.707, 0, 0, 0.707 }), std::vector<double>({ 500, 500 }), "0xbbbbbb");
  vis_group->add_vis_infos(prx::info_geometry_t::LINE, rrt_star_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, rrt_star_query.solution_traj, body_name, ss);
  vis_group->add_vis_infos(prx::info_geometry_t::SPHERE, { Vec(rrt_star_query.goal_state).head(3) }, "0xffff00", rrt_star_query.goal_region_radius);
  vis_group->add_animation(rrt_star_query.solution_traj, ss, rrt_star_query.start_state);
  vis_group->output_html("biased_peg_in_hole_rrt_star.html");

  delete vis_group;

  std::cout << "End of program" << std::endl;
}
