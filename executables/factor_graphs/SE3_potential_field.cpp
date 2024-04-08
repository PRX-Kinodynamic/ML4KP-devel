#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/simulation/plants/first_order_free_body.hpp"

using prx::split;
using prx::utilities::convert_to;
int main(int argc, char* argv[])
{
  prx::param_loader params(argc, argv);
  std::string input;
  Eigen::Quaterniond quat{ Eigen::Quaterniond::Identity() };
  Eigen::Vector3d pos{ Eigen::Vector3d(0, 0, 0) };
  prx::fg::se3_t xi{};
  prx::fg::se3_t xres{};
  prx::fg::se3_t xgoal{ quat, pos };
  prx::fg::se3_t xobstacle{ quat, pos };

  prx::PairNameObstacles obstacles{ prx::load_obstacles("environments/peg_in_hole_0.yaml") };
  const std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list{ obstacles.second };
  const std::vector<std::string> obstacle_names{ obstacles.first };

  prx::system_ptr_t plant = prx::system_factory_t::create_system("first_order_free_body", "first_order_free_body");
  prx::space_t* ss{ plant->get_state_space() };
  prx::space_point_t pt{ ss->make_point() };
  prx::collision_group_t cg({ plant }, { obstacle_list });
  // Eigen::Matrix<double, 6, 6> H{ Eigen::Matrix<double, 6, 6>::Zero() };

  const bool normalize{ params.exists("normalize") ? params["normalize"].as<bool>() : false };
  const bool repulsive{ params.exists("repulsive") ? params["repulsive"].as<bool>() : false };

  const double cuttoff{ params.exists("cuttoff") ? params["cuttoff"].as<double>() : 1.0 };
  const double x_obstacle{ params.exists("x_obstacle") ? params["x_obstacle"].as<double>() : 5.0 };
  const double z_obstacle{ params.exists("z_obstacle") ? params["z_obstacle"].as<double>() : 28.0 };

  const double gain{ params.exists("gain") ? params["gain"].as<double>() : 1.0 };
  const double gain_goal{ params.exists("gain_goal") ? params["gain_goal"].as<double>() : 1.0 };

  while (getline(std::cin, input))
  {
    const std::vector<double> line{ split<double>(input) };

    quat.w() = convert_to<double>(line[0]);
    quat.x() = convert_to<double>(line[1]);
    quat.y() = convert_to<double>(line[2]);
    quat.z() = convert_to<double>(line[3]);

    pos[0] = convert_to<double>(line[4]);
    pos[1] = convert_to<double>(line[5]);
    pos[2] = convert_to<double>(line[6]);

    xi.quaternion() = quat;
    xi.position() = pos;
    xres = xi.between(xgoal);

    if (repulsive)
    {
      // xobstacle.position() = xi.position();
      Vec(pt).head(3) = xi.position();
      Vec(pt)[3] = xi.quaternion().w();
      Vec(pt).tail(3) = xi.quaternion().vec();
      ss->copy_from(pt);

      const prx::collision_group_t::pqp_distance_t pqp_distances{ cg.get_distances() };
      // PRX_DEBUG_VAR_3(pt, pqp_distances.distances.size(), pqp_distances.closest_points.size());
      // PRX_DEBUG_CONTAINER(pqp_distances.distances);
      // PRX_DEBUG_CONTAINER(pqp_distances.closest_points[0]);
      // PRX_DEBUG_CONTAINER(pqp_distances.closest_points[1]);
      // PRX_DEBUG_CONTAINER(pqp_distances.closest_points[2]);
      // PRX_DEBUG_CONTAINER(pqp_distances.closest_points[3]);
      // PRX_DEBUG_CONTAINER(pqp_distances.closest_points[4]);
      // PRX_DEBUG_CONTAINER(pqp_distances.closest_points[1]);
      // const double dist_xp{ -xi.position()[0] + x_obstacle };
      // const double dist_xm{ xi.position()[0] + x_obstacle };
      // const double dist_z{ xi.position()[2] - z_obstacle };
      // // PRX_DEBUG_VAR_3(dist_xp, dist_xm, dist_z);
      bool do_repulsive{ false };
      double distance{ cuttoff * 2 };  // just some high-enough value
      for (int i = 0; i < pqp_distances.distances.size() - 1; ++i)
      {
        const double dist{ pqp_distances.distances[i] };
        if (dist <= cuttoff and dist < distance)
        {
          distance = dist;
          xobstacle.position()[0] = pqp_distances.closest_points[i][0];
          xobstacle.position()[1] = pqp_distances.closest_points[i][1];
          xobstacle.position()[2] = pqp_distances.closest_points[i][2];
          do_repulsive = true;
        }
      }
      // if (std::abs(dist_z) <= cuttoff and (dist_xp < 0.0 or dist_xm < 0.0))
      // {
      //   distance += dist_z * dist_z;
      //   xobstacle.position()[2] = -z_obstacle;
      //   do_repulsive = true;
      // }

      // if (cg.in_collision() or (dist_z < 0 and (dist_xp < 0.0 or dist_xm < 0.0)))
      // {
      //   distance = 0;
      //   do_repulsive = true;
      // } Threshold
      if (do_repulsive)
      {
        xobstacle = xobstacle.between(xi);
        xobstacle.position() = gain * xobstacle.position() / (distance);
        // xres.position() = gain_goal * xres.position() ;
        // xres.position().normalize();
        xres = xobstacle * xres;
      }
    }
    if (normalize)
    {
      xobstacle.position().normalize();
      xres.position().normalize();
    }
    std::cout << xres << " " << xobstacle << "\n";
  }

  return 0;
}
