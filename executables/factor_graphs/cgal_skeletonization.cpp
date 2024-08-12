#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/aorrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planner_functions/tree_fix_time_discretization.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/Straight_skeleton_2/IO/print.h>
#include <boost/shared_ptr.hpp>
#include <cassert>

#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/draw_straight_skeleton_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K> Ss;
typedef boost::shared_ptr<Ss> SsPtr;

Polygon_2 create_hole_from_obstacle(std::shared_ptr<prx::fg::collision_info_t> info)
{
  Polygon_2 hole;

  const prx::geometry_type_t& geom_type{ info->geom_type };
  const std::vector<double>& params{ info->params };

  const Eigen::Vector2d position{ info->pose.position().head(2) };
  switch (geom_type)
  {
    case prx::geometry_type_t::BOX: {
      const double x_half{ params[0] / 2.0 };
      const double y_half{ params[1] / 2.0 };
      const Eigen::Quaterniond quat{ info->pose.quaternion() };
      const Eigen::Rotation2D R(prx::quaternion_to_euler(quat)[2]);
      const Eigen::Vector2d pt0{ position + R * Eigen::Vector2d(+x_half, +y_half) };
      const Eigen::Vector2d pt1{ position + R * Eigen::Vector2d(+x_half, -y_half) };
      const Eigen::Vector2d pt2{ position + R * Eigen::Vector2d(-x_half, -y_half) };
      const Eigen::Vector2d pt3{ position + R * Eigen::Vector2d(-x_half, +y_half) };
      // PRX_DBG_VARS(pt0.transpose());
      // PRX_DBG_VARS(pt1.transpose());
      // PRX_DBG_VARS(pt2.transpose());
      // PRX_DBG_VARS(pt3.transpose());
      hole.push_back(Point(pt0[0], pt0[1]));
      hole.push_back(Point(pt1[0], pt1[1]));
      hole.push_back(Point(pt2[0], pt2[1]));
      hole.push_back(Point(pt3[0], pt3[1]));
    }
    break;
    case prx::geometry_type_t::SPHERE:
      break;
    case prx::geometry_type_t::ELLIPSOID:
      break;
    case prx::geometry_type_t::CAPSULE:
      break;
    case prx::geometry_type_t::CONE:
      break;
    case prx::geometry_type_t::CYLINDER:
      const double radius{ params[0] };
      const Eigen::Vector2d v0(radius, 0);
      for (double angle = 2.0 * prx::constants::pi; angle > 0.0; angle -= 0.5)
      {
        // double radius, double height
        const Eigen::Vector2d pt{ position + Eigen::Rotation2D(angle) * v0 };
        hole.push_back(Point(pt[0], pt[1]));
        // PRX_DBG_VARS(pt.transpose());
      }
      break;
      // default:
      //   prx_throw("Error on geometry_type_t");
  };
  return hole;
}

int main()
{
  // const std::string environment{ "environments/warehouse.yaml" };
  // const std::string environment{ "environments/forest.yaml" };
  const std::string environment{ "environments/simple_obstacle.yaml" };
  auto obstacles = prx::load_obstacles(environment);
  std::string ignore_substr{ "wall" };

  int i{ 0 };
  while (i < obstacles.first.size())
  {
    // for (int i = 0; i < obstacles.first.size(); ++i)

    if (obstacles.first[i].find(ignore_substr) != std::string::npos)
    {
      PRX_DBG_VARS(obstacles.first[i]);
      obstacles.first.erase(obstacles.first.begin() + i);
      obstacles.second.erase(obstacles.second.begin() + i);
    }
    else
    {
      i++;
    }
  }
  auto obstacle_collision_infos = prx::fg::collision_info_t::generate_infos(obstacles.second);

  Polygon_2 outer;
  // outer.push_back(Point(-4, -4));
  // outer.push_back(Point(45, -4));
  // outer.push_back(Point(45, 30));
  // outer.push_back(Point(-4, 30));
  outer.push_back(Point(-4, -4));
  outer.push_back(Point(24, -4));
  outer.push_back(Point(24, 24));
  outer.push_back(Point(-4, 24));
  // Polygon_2 hole;
  assert(outer.is_counterclockwise_oriented());
  Polygon_with_holes poly(outer);
  for (auto info : obstacle_collision_infos)
  {
    // PRX_DBG_VARS(info->pose.position().transpose());
    Polygon_2 hole{ create_hole_from_obstacle(info) };
    assert(hole.is_clockwise_oriented());
    poly.add_hole(hole);
    // break;
  }
  // hole.push_back(Point(8.5, 8.5));
  // hole.push_back(Point(8.5, 11.5));
  // hole.push_back(Point(11.5, 11.5));
  // hole.push_back(Point(11.5, 8.5));
  SsPtr iss = CGAL::create_interior_straight_skeleton_2(poly);
  // CGAL::Straight_skeletons_2::IO::print_straight_skeleton(*iss);
  // vertices_begin ()
  std::ofstream ofs(prx::out_path + "/cgal_skw.txt");

  std::vector<std::pair<int, int>> edges;
  std::vector<std::pair<int, Point>> points;
  std::set<int> valid_ids;
  // Vertex_iterator   vertices_end ()
  for (auto iter = iss->halfedges_begin(); iter != iss->halfedges_end(); iter++)
  {
    if (iter->is_inner_bisector())
    {
      valid_ids.insert(iter->id());
      // points.push_back(std::make_pair(iter->prev()->id(), iter->prev()->vertex()->point()));
      points.push_back(std::make_pair(iter->id(), iter->vertex()->point()));
      // ofs << iter->id() << " " << iter->vertex()->point() << "\n";
      edges.push_back(std::make_pair(iter->prev()->id(), iter->id()));
      // print_vertex(h->prev()->vertex());
    }
  }

  for (auto id_pt : points)
  {
    if (valid_ids.count(id_pt.first) > 0)
      ofs << id_pt.first << " " << id_pt.second << "\n";
  }
  ofs << "\n";
  for (auto pair : edges)
  {
    if (valid_ids.count(pair.first) > 0 and valid_ids.count(pair.second) > 0)
      ofs << pair.first << " " << pair.second << "\n";
  }
  ofs.close();

  return EXIT_SUCCESS;
}