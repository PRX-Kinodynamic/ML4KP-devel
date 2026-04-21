#include <memory>
#include "general/debug_utils.hpp"
#define BOOST_AUTO_TEST_MAIN gnn_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/data_structures/graph_nearest_neighbors.hpp"
namespace mock
{
using State = Eigen::Vector2d;
using DistanceFunction = std::function<double(const State&, const State&)>;
DistanceFunction df = [](const State& a, const State& b) { return (a - b).norm(); };
using GNN = prx::data_structures::graph_nearest_neighbors_t<State, DistanceFunction>;
}  // namespace mock

const double EPSILON{ 0.01 };
BOOST_AUTO_TEST_CASE(gnn_builds_ok)
{
  mock::GNN gnn(mock::df);

  const long unsigned expected_nr_nodes{ 0 };
  const long unsigned resulting_nr_nodes{ gnn.size() };
  BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes, EXPECTED_GOT(expected_nr_nodes, resulting_nr_nodes));
}
BOOST_AUTO_TEST_CASE(gnn_add_node_test)
{
  mock::GNN gnn(mock::df);
  Eigen::Vector2d v0(0, 0);

  gnn.insert(v0);

  BOOST_REQUIRE(gnn.size() == 1);
}

BOOST_AUTO_TEST_CASE(gnn_small_single_query_test)
{
  mock::GNN gnn(mock::df);

  /**
   * p0 ---- p1 --p2
   */

  Eigen::Vector2d p0(0.0, 0.0);
  Eigen::Vector2d p1(4.0, 0.0);
  Eigen::Vector2d p2(6.0, 0.0);

  gnn.insert(p0);
  gnn.insert(p1);
  gnn.insert(p2);

  const std::size_t expected_nr_nodes{ 3 };
  const std::size_t resulting_nr_nodes{ gnn.size() };
  BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes, EXPECTED_GOT(expected_nr_nodes, resulting_nr_nodes));

  Eigen::Vector2d query0(1.0, 0.0);
  Eigen::Vector2d query1(4.5, 0.0);
  Eigen::Vector2d query2(6.5, 0.0);

  auto [result_closest_to_p0, result_distance_to_p0] = gnn.closest_neighbor(query0);
  auto [result_closest_to_p1, result_distance_to_p1] = gnn.closest_neighbor(query1);
  auto [result_closest_to_p2, result_distance_to_p2] = gnn.closest_neighbor(query2);

  const double dist_p0qres0{ (p0 - result_closest_to_p0).norm() };
  const double dist_p1qres1{ (p1 - result_closest_to_p1).norm() };
  const double dist_p2qres2{ (p2 - result_closest_to_p2).norm() };

  const double dist_p0q0{ (p0 - query0).norm() };
  const double dist_p1q1{ (p1 - query1).norm() };
  const double dist_p2q2{ (p2 - query2).norm() };

  BOOST_CHECK(dist_p0qres0 < EPSILON);
  BOOST_CHECK(dist_p1qres1 < EPSILON);
  BOOST_CHECK(dist_p2qres2 < EPSILON);

  // PRX_DBG_VARS(result_distance_to_p0, dist_p0q0)
  // PRX_DBG_VARS(result_distance_to_p1, dist_p1q1)
  // PRX_DBG_VARS(result_distance_to_p2, dist_p2q2)
  BOOST_CHECK(std::fabs(result_distance_to_p0 - dist_p0q0) < EPSILON);
  BOOST_CHECK(std::fabs(result_distance_to_p1 - dist_p1q1) < EPSILON);
  BOOST_CHECK(std::fabs(result_distance_to_p2 - dist_p2q2) < EPSILON);
}

BOOST_AUTO_TEST_CASE(gnn_big_single_query_test)
{
  mock::GNN gnn(mock::df);

  Eigen::Vector2d p0(0.0, 0.0);

  gnn.insert(p0);

  // Insert many nodes far away from p0
  for (int i = 0; i < 10000; ++i)
  {
    Eigen::Vector2d pfar(10 + i, 10 + i);
    gnn.insert(pfar);
  }

  Eigen::Vector2d query0(3.0, 0.0);

  auto [result_closest_to_p0, result_distance_to_p0] = gnn.closest_neighbor(query0);

  // The closest is p2
  // const double dist_p2q0{ (result_closest_to_p0 - p0).norm() };

  const double dist_p0qres0{ (p0 - result_closest_to_p0).norm() };

  const double dist_p0q0{ (p0 - query0).norm() };

  BOOST_CHECK(dist_p0qres0 < EPSILON);

  BOOST_CHECK(std::fabs(result_distance_to_p0 - dist_p0q0) < EPSILON);
}

BOOST_AUTO_TEST_CASE(gnn_k_neighbors_query_test)
{
  mock::GNN gnn(mock::df);

  Eigen::Vector2d p0(0.0, 0.0);
  Eigen::Vector2d p1(1.0, 0.0);
  Eigen::Vector2d p2(2.0, 0.0);

  gnn.insert(p0);
  gnn.insert(p1);
  gnn.insert(p2);

  // Insert many nodes far away from p0
  for (int i = 0; i < 10; ++i)
  {
    Eigen::Vector2d pfar(10 + i, 10 + i);
    gnn.insert(pfar);
  }

  Eigen::Vector2d query0(3.0, 0.0);

  std::vector<Eigen::Vector2d> result_3{ gnn.k_neighbors(query0, 3) };
  std::vector<Eigen::Vector2d> result_4{ gnn.k_neighbors(query0, 4) };

  // PRX_DBG_VARS(result_3)
  // The closest is p2
  const double dist_p2q0{ (result_3[0] - p2).norm() };
  const double dist_p1q0{ (result_3[1] - p1).norm() };
  const double dist_p0q0{ (result_3[2] - p0).norm() };

  BOOST_CHECK(dist_p2q0 < EPSILON);
  BOOST_CHECK(dist_p1q0 < EPSILON);
  BOOST_CHECK(dist_p0q0 < EPSILON);

  BOOST_CHECK((result_3[0] - result_4[0]).norm() < EPSILON);
  BOOST_CHECK((result_3[1] - result_4[1]).norm() < EPSILON);
  BOOST_CHECK((result_3[2] - result_4[2]).norm() < EPSILON);
  BOOST_CHECK((Eigen::Vector2d(10, 10) - result_4[3]).norm() < EPSILON);

  // BOOST_CHECK(std::fabs(result_distance_to_p0 - dist_p0q0) < EPSILON);
  // BOOST_CHECK(std::fabs(result_distance_to_p1 - dist_p1q1) < EPSILON);
  // BOOST_CHECK(std::fabs(result_distance_to_p2 - dist_p2q2) < EPSILON);
}

BOOST_AUTO_TEST_CASE(gnn_neighbors_in_radius_query_test)
{
  mock::GNN gnn(mock::df);

  Eigen::Vector2d p0(0.0, 0.0);
  Eigen::Vector2d p1(1.0, 0.0);
  Eigen::Vector2d p2(2.0, 0.0);

  gnn.insert(p0);
  gnn.insert(p1);
  gnn.insert(p2);

  // Insert many nodes far away from p0
  for (int i = 0; i < 10; ++i)
  {
    Eigen::Vector2d pfar(10 + i, 10 + i);
    gnn.insert(pfar);
  }

  Eigen::Vector2d query0(3.0, 0.0);

  std::vector<Eigen::Vector2d> result_3{ gnn.neighbors_in_radius(query0, 3.5) };
  std::vector<Eigen::Vector2d> result_4{ gnn.neighbors_in_radius(query0, 13.0) };
  // query dist to (10,10) = 12.2066
  // query dist to (11,11) = 13.6015

  // PRX_DBG_VARS(result_3)
  BOOST_CHECK(result_3.size() == 3);
  BOOST_CHECK(result_4.size() == 4);
  // The closest is p2
  const double dist_p2q0{ (result_3[0] - p2).norm() };
  const double dist_p1q0{ (result_3[1] - p1).norm() };
  const double dist_p0q0{ (result_3[2] - p0).norm() };

  BOOST_CHECK(dist_p2q0 < EPSILON);
  BOOST_CHECK(dist_p1q0 < EPSILON);
  BOOST_CHECK(dist_p0q0 < EPSILON);

  BOOST_CHECK((result_3[0] - result_4[0]).norm() < EPSILON);
  BOOST_CHECK((result_3[1] - result_4[1]).norm() < EPSILON);
  BOOST_CHECK((result_3[2] - result_4[2]).norm() < EPSILON);
  BOOST_CHECK((Eigen::Vector2d(10, 10) - result_4[3]).norm() < EPSILON);

  // BOOST_CHECK(std::fabs(result_distance_to_p0 - dist_p0q0) < EPSILON);
  // BOOST_CHECK(std::fabs(result_distance_to_p1 - dist_p1q1) < EPSILON);
  // BOOST_CHECK(std::fabs(result_distance_to_p2 - dist_p2q2) < EPSILON);
}

BOOST_AUTO_TEST_CASE(gnn_neighbors_delete_node)
{
  mock::GNN gnn(mock::df);

  Eigen::Vector2d p0(0.0, 0.0);
  Eigen::Vector2d p1(1.0, 0.0);
  Eigen::Vector2d p2(2.0, 0.0);

  const std::size_t idx_0{ gnn.insert(p0) };
  const std::size_t idx_1{ gnn.insert(p1) };
  const std::size_t idx_2{ gnn.insert(p2) };

  gnn.remove_node(idx_0);
  BOOST_REQUIRE(gnn.size() == 2);

  Eigen::Vector2d query0(0.0, 0.0);

  auto [result, result_distance] = gnn.closest_neighbor(query0);
  // std::vector<Eigen::Vector2d> result_4{ gnn.neighbors_in_radius(query0, 13.0) };
  // // query dist to (10,10) = 12.2066
  // // query dist to (11,11) = 13.6015

  // PRX_DBG_VARS(result_3)
  BOOST_CHECK((result - p1).norm() < 0.001);
  BOOST_REQUIRE_CLOSE(result_distance, 1.0, 0.001);
  // BOOST_CHECK(result_4.size() == 4);
  // // The closest is p2
  // const double dist_p2q0{ (result_3[0] - p2).norm() };
  // const double dist_p1q0{ (result_3[1] - p1).norm() };
  // const double dist_p0q0{ (result_3[2] - p0).norm() };

  // BOOST_CHECK(dist_p2q0 < EPSILON);
  // BOOST_CHECK(dist_p1q0 < EPSILON);
  // BOOST_CHECK(dist_p0q0 < EPSILON);

  // BOOST_CHECK((result_3[0] - result_4[0]).norm() < EPSILON);
  // BOOST_CHECK((result_3[1] - result_4[1]).norm() < EPSILON);
  // BOOST_CHECK((result_3[2] - result_4[2]).norm() < EPSILON);
  // BOOST_CHECK((Eigen::Vector2d(10, 10) - result_4[3]).norm() < EPSILON);

  // BOOST_CHECK(std::fabs(result_distance_to_p0 - dist_p0q0) < EPSILON);
  // BOOST_CHECK(std::fabs(result_distance_to_p1 - dist_p1q1) < EPSILON);
  // BOOST_CHECK(std::fabs(result_distance_to_p2 - dist_p2q2) < EPSILON);
}
