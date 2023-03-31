#define BOOST_AUTO_TEST_MAIN time_map_test
#include <string>
#include <boost/test/unit_test.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "prx/utilities/general/dijkstra.hpp"

using prx::utilities::dijkstra_t;

template <typename Point>
struct test_node_t
{
  test_node_t() = default;
  test_node_t(const test_node_t& other) = default;
  test_node_t(const std::size_t _id, Point _pt) : id(_id), pt(_pt)
  {
  }
  std::size_t id;
  Point pt;
  std::vector<std::size_t> neighbors;
};

BOOST_AUTO_TEST_CASE(dijkstra_shortest_path_when_nodes_are_neighbors)
{
  using Point = Eigen::Vector2d;
  using TestNode = test_node_t<Point>;
  const std::size_t n0_id{ 0 };
  const std::size_t n1_id{ 1 };
  TestNode n0(n0_id, Point(0, 0));
  TestNode n1(n1_id, Point(0, 1));
  n0.neighbors.push_back(n1_id);
  n1.neighbors.push_back(n0_id);

  std::unordered_map<std::size_t, TestNode> graph;
  graph[n0_id] = n0;
  graph[n1_id] = n1;

  std::function<std::vector<std::size_t>(const std::size_t&)> get_neighbors = [&](const std::size_t& id) {
    return graph[id].neighbors;
  };
  std::function<double(const std::size_t&, const std::size_t&)> node_distance =
      [&](const std::size_t& a, const std::size_t& b) { return (graph[a].pt - graph[b].pt).norm(); };
  std::vector<std::size_t> sp = dijkstra_t::shortest_path(n0_id, n1_id, get_neighbors, node_distance);

  const std::vector<std::size_t> expected_path = { n0_id, n1_id };
  BOOST_CHECK(sp == expected_path);
}

BOOST_AUTO_TEST_CASE(dijkstra_shortest_path_of_a_line_is_a_line)
{
  using Point = Eigen::Vector2d;
  using TestNode = test_node_t<Point>;
  const std::size_t n0_id{ 0 };
  const std::size_t n1_id{ 1 };
  const std::size_t n2_id{ 2 };
  const std::size_t n3_id{ 3 };
  TestNode n0(n0_id, Point(0, 0));
  TestNode n1(n1_id, Point(0, 1));
  TestNode n2(n2_id, Point(0, 2));
  TestNode n3(n3_id, Point(0, 3));

  n0.neighbors.push_back(n1_id);

  n1.neighbors.push_back(n0_id);
  n1.neighbors.push_back(n2_id);

  n2.neighbors.push_back(n1_id);
  n2.neighbors.push_back(n3_id);

  n3.neighbors.push_back(n2_id);

  std::unordered_map<std::size_t, TestNode> graph;
  graph[n0_id] = n0;
  graph[n1_id] = n1;
  graph[n2_id] = n2;
  graph[n3_id] = n3;

  std::function<std::vector<std::size_t>(const std::size_t&)> get_neighbors = [&](const std::size_t& id) {
    return graph[id].neighbors;
  };
  std::function<double(const std::size_t&, const std::size_t&)> node_distance =
      [&](const std::size_t& a, const std::size_t& b) { return (graph[a].pt - graph[b].pt).norm(); };
  std::vector<std::size_t> sp = dijkstra_t::shortest_path(n0_id, n3_id, get_neighbors, node_distance);

  const std::vector<std::size_t> expected_path = { n0_id, n1_id, n2_id, n3_id };
  BOOST_CHECK(sp == expected_path);
}

BOOST_AUTO_TEST_CASE(dijkstra_shortest_path_in_a_graph_with_multiple_paths)
{
  /*      2
  **	  /   \
  **	 /     \
  **  0 			3
  **	  \   /
  **	    1
  */
  using Point = Eigen::Vector2d;
  using TestNode = test_node_t<Point>;
  const std::size_t n0_id{ 0 };
  const std::size_t n1_id{ 1 };
  const std::size_t n2_id{ 2 };
  const std::size_t n3_id{ 3 };
  TestNode n0(n0_id, Point(0, 0));
  TestNode n1(n1_id, Point(1, -1));
  TestNode n2(n2_id, Point(1, 2));
  TestNode n3(n3_id, Point(2, 0));

  n0.neighbors.push_back(n1_id);
  n0.neighbors.push_back(n2_id);

  n1.neighbors.push_back(n0_id);
  n1.neighbors.push_back(n3_id);

  n2.neighbors.push_back(n0_id);
  n2.neighbors.push_back(n3_id);

  n3.neighbors.push_back(n1_id);
  n3.neighbors.push_back(n2_id);

  std::unordered_map<std::size_t, TestNode> graph;
  graph[n0_id] = n0;
  graph[n1_id] = n1;
  graph[n2_id] = n2;
  graph[n3_id] = n3;

  std::function<std::vector<std::size_t>(const std::size_t&)> get_neighbors = [&](const std::size_t& id) {
    return graph[id].neighbors;
  };
  std::function<double(const std::size_t&, const std::size_t&)> node_distance =
      [&](const std::size_t& a, const std::size_t& b) { return (graph[a].pt - graph[b].pt).norm(); };
  std::vector<std::size_t> sp = dijkstra_t::shortest_path(n0_id, n3_id, get_neighbors, node_distance);

  const std::vector<std::size_t> expected_path = { n0_id, n1_id, n3_id };
  // PRX_DEBUG_ITERABLE("sp: ", sp);
  BOOST_CHECK(sp == expected_path);
}