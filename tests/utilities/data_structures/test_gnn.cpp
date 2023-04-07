#define BOOST_TEST_MODULE gnn_test
#include <boost/test/included/unit_test.hpp>
#include <string>
#include "prx/utilities/data_structures/gnn.hpp"

BOOST_AUTO_TEST_CASE(gnn_builds_ok)
{
  using Point = Eigen::Vector2d;
  using Metric = std::function<double(const Point&, const Point&)>;
  Metric metric = [](const Point& a, const Point& b) { return (a - b).norm(); };
  prx::graph_nearest_neighbors_t<Point> gnn(metric);

  const long unsigned expected_nr_nodes{ 0 };
  const long unsigned resulting_nr_nodes{ gnn.get_nr_nodes() };
  BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes,
                      "Expected: " << expected_nr_nodes << ". Got: " << resulting_nr_nodes << ".");
}

BOOST_AUTO_TEST_CASE(gnn_2d_single_query_test)
{
  using Point = Eigen::Vector2d;
  using Metric = std::function<double(const Point&, const Point&)>;
  Metric metric = [](const Point& a, const Point& b) { return (a - b).norm(); };
  prx::graph_nearest_neighbors_t<Point> gnn(metric);

  /**
   * p0 ---- p1 --p2
   */
  prx::proximity_node_t<Point>* p0;
  prx::proximity_node_t<Point>* p1;
  prx::proximity_node_t<Point>* p2;

  p0 = new prx::proximity_node_t<Point>();
  p1 = new prx::proximity_node_t<Point>();
  p2 = new prx::proximity_node_t<Point>();

  p0->point = Point(0, 0);
  p1->point = Point(4, 0);
  p2->point = Point(6, 0);

  gnn.add_node(p0);
  gnn.add_node(p1);
  gnn.add_node(p2);

  const long unsigned expected_nr_nodes{ 3 };
  const long unsigned resulting_nr_nodes{ gnn.get_nr_nodes() };
  BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes,
                      "Expected: " << expected_nr_nodes << ". Got: " << resulting_nr_nodes << ".");

  const Point p0_query(1.0, 0);
  const Point p1_query(4.5, 0);
  const Point p2_query(6.5, 0);

  const Point result_closest_to_p0{ gnn.single_query(p0_query)->point };
  const Point result_closest_to_p1{ gnn.single_query(p1_query)->point };
  const Point result_closest_to_p2{ gnn.single_query(p2_query)->point };

  const Point expected_closest_to_p0{ p0->point };
  const Point expected_closest_to_p1{ p1->point };
  const Point expected_closest_to_p2{ p2->point };

  BOOST_CHECK_MESSAGE(result_closest_to_p0.isApprox(expected_closest_to_p0),
                      "Got: " << result_closest_to_p0 << ". Expected: " << expected_closest_to_p0 << ".");

  BOOST_CHECK_MESSAGE(result_closest_to_p1.isApprox(expected_closest_to_p1),
                      "Got: " << result_closest_to_p1 << ". Expected: " << expected_closest_to_p1 << ".");

  BOOST_CHECK_MESSAGE(result_closest_to_p2.isApprox(expected_closest_to_p2),
                      "Got: " << result_closest_to_p2 << ". Expected: " << expected_closest_to_p2 << ".");
}

BOOST_AUTO_TEST_CASE(gnn_2d_multi_query_test)
{
  using Point = Eigen::Vector2d;
  using Metric = std::function<double(const Point&, const Point&)>;
  Metric metric = [](const Point& a, const Point& b) { return (a - b).norm(); };
  prx::graph_nearest_neighbors_t<Point> gnn(metric);

  std::vector<prx::proximity_node_t<Point>*> nodes;
  for (int i = 0; i < 10; ++i)
  {
    nodes.push_back(new prx::proximity_node_t<Point>());
    nodes.back()->point = Point::Ones() * i;
    gnn.add_node(nodes.back());
  }

  const Point query(0.25, 0.25);
  const std::size_t expected_vector_size{ 3 };
  const std::vector<prx::proximity_node_t<Point>*> result_closest{ gnn.multi_query(query, expected_vector_size) };

  BOOST_CHECK_MESSAGE(expected_vector_size == result_closest.size(),
                      "Expected: " << expected_vector_size << ". Got: " << result_closest.size() << ".");

  const Point result_close_0{ result_closest[0]->point };
  const Point result_close_1{ result_closest[1]->point };
  const Point result_close_2{ result_closest[2]->point };

  const Point expected_closest_to_p0{ Point(0.0, 0.0) };
  const Point expected_closest_to_p1{ Point(1.0, 1.0) };
  const Point expected_closest_to_p2{ Point(2.0, 2.0) };

  BOOST_CHECK_MESSAGE(result_close_0.isApprox(expected_closest_to_p0),
                      "Got: " << result_close_0 << ". Expected: " << expected_closest_to_p0 << ".");

  BOOST_CHECK_MESSAGE(result_close_1.isApprox(expected_closest_to_p1),
                      "Got: " << result_close_1 << ". Expected: " << expected_closest_to_p1 << ".");

  BOOST_CHECK_MESSAGE(result_close_2.isApprox(expected_closest_to_p2),
                      "Got: " << result_close_2 << ". Expected: " << expected_closest_to_p2 << ".");
}

BOOST_AUTO_TEST_CASE(gnn_2d_radius_and_closest_query_test)
{
  using Point = Eigen::Vector2d;
  using Metric = std::function<double(const Point&, const Point&)>;
  Metric metric = [](const Point& a, const Point& b) { return (a - b).norm(); };
  prx::graph_nearest_neighbors_t<Point> gnn(metric);

  std::vector<prx::proximity_node_t<Point>*> nodes;
  for (int i = 0; i < 10; ++i)
  {
    nodes.push_back(new prx::proximity_node_t<Point>());
    nodes.back()->point = Point::Ones() * i;
    gnn.add_node(nodes.back());
  }

  const Point query(5.1, 5.1);
  const double query_radius{ 2.0 };  // more than sqrt(2), less than 2sqrt(2)
  const std::vector<prx::proximity_node_t<Point>*> result_closest{ gnn.radius_and_closest_query(query, query_radius) };

  const std::size_t expected_vector_size{ 3 };
  BOOST_CHECK_MESSAGE(expected_vector_size == result_closest.size(),
                      "Expected: " << expected_vector_size << ". Got: " << result_closest.size() << ".");

  const Point result_close_0{ result_closest[0]->point };
  const Point result_close_1{ result_closest[1]->point };
  const Point result_close_2{ result_closest[2]->point };

  const Point expected_closest_to_p0{ Point(5.0, 5.0) };
  const Point expected_closest_to_p1{ Point(6.0, 6.0) };
  const Point expected_closest_to_p2{ Point(4.0, 4.0) };

  BOOST_CHECK_MESSAGE(result_close_0.isApprox(expected_closest_to_p0),
                      "Got: " << result_close_0 << ". Expected: " << expected_closest_to_p0 << ".");

  BOOST_CHECK_MESSAGE(result_close_1.isApprox(expected_closest_to_p1),
                      "Got: " << result_close_1 << ". Expected: " << expected_closest_to_p1 << ".");

  BOOST_CHECK_MESSAGE(result_close_2.isApprox(expected_closest_to_p2),
                      "Got: " << result_close_2 << ". Expected: " << expected_closest_to_p2 << ".");
}