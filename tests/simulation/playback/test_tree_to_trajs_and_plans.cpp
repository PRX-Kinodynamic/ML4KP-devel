#define BOOST_AUTO_TEST_MAIN constants_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/simulation/playback/tree_to_trajs_and_plans.hpp"
namespace mock
{
struct space_test_t
{
  space_test_t() : u0(0), u1(0), address({ &u0, &u1 }), space("EE", address, "space_test")
  {
    space.set_bounds({ -100, -200 }, { 100, 200 });
  }
  double u0, u1;
  std::vector<double*> address;
  prx::space_t space;
};

struct edge_t : public prx::tree_edge_t
{
  edge_t() : prx::tree_edge_t() {};

  std::shared_ptr<prx::plan_t> plan;
  std::shared_ptr<prx::trajectory_t> traj;
};
struct node_t : public prx::tree_node_t
{
  node_t() : prx::tree_node_t() {};
};

}  // namespace mock

BOOST_AUTO_TEST_CASE(plan_construction_test)
{
  using VectorTrajsPlans = std::vector<std::pair<prx::trajectory_t, prx::plan_t>>;
  mock::space_test_t test;
  prx::space_t& space(test.space);
  prx::plan_t plan(&space);
  prx::trajectory_t traj(&space);

  prx::tree_t tree{};
  tree.allocate_memory<mock::node_t, mock::edge_t>(1000);

  const Eigen::Vector2d pt(1, 2);
  traj.push_back(pt);
  plan.copy_onto_back(pt, 1.0);

  //     n0
  //   n1  n2
  //      n3 n4
  //          n5
  //           n6
  const prx::node_index_t n0_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };
  const prx::node_index_t n1_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };
  const prx::node_index_t n2_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };
  const prx::node_index_t n3_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };
  const prx::node_index_t n4_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };
  const prx::node_index_t n5_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };
  const prx::node_index_t n6_index{ tree.add_vertex<mock::node_t, mock::edge_t>() };

  std::vector<prx::edge_index_t> edges{};
  edges.push_back(tree.add_edge(n0_index, n1_index));
  edges.push_back(tree.add_edge(n0_index, n2_index));
  edges.push_back(tree.add_edge(n2_index, n3_index));
  edges.push_back(tree.add_edge(n2_index, n4_index));
  edges.push_back(tree.add_edge(n4_index, n5_index));
  edges.push_back(tree.add_edge(n5_index, n6_index));

  for (auto idx : edges)
  {
    tree.get_edge_as<mock::edge_t>(idx)->plan = std::make_shared<prx::plan_t>(plan);
    tree.get_edge_as<mock::edge_t>(idx)->traj = std::make_shared<prx::trajectory_t>(traj);
  }

  // tree.get_edge_as<mock::edge_t>(e02_index)->plan = std::make_shared<prx::plan_t>(plan);
  // tree.get_edge_as<mock::edge_t>(e02_index)->traj = std::make_shared<prx::trajectory_t>(traj);

  VectorTrajsPlans res{ prx::simulation::tree_to_trajs_and_plans<mock::node_t, mock::edge_t>(&tree, n0_index) };

  std::set<double> durations;
  for (auto pair : res)
  {
    durations.insert(pair.second.duration());
  }
  // PRX_DBG_VARS(durations);

  BOOST_REQUIRE(durations.count(1.0) == 1);
  BOOST_REQUIRE(durations.count(2.0) == 1);
  BOOST_REQUIRE(durations.count(4.0) == 1);
  // BOOST_REQUIRE(durations.count(2.0) == 1);
  // BOOST_REQUIRE(durations.count(2.0) == 1);

  // PRX_DBG_VARS(res);
}
