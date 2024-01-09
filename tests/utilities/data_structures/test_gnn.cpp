#define BOOST_AUTO_TEST_MAIN gnn_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
namespace mock
{
struct space2d_t
{
  space2d_t() : x(0), y(0), address({ &x, &y }), space("EE", address, "space_test")
  {
  }
  double x, y;
  std::vector<double*> address;
  prx::space_t space;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(gnn_builds_ok)
{
  prx::distance_function_t metric = [](const prx::space_point_t& a, const prx::space_point_t& b) {
    return (Vec(a) - Vec(b)).norm();
  };
  prx::graph_nearest_neighbors_t gnn(metric);

  const long unsigned expected_nr_nodes{ 0 };
  const long unsigned resulting_nr_nodes{ gnn.get_nr_nodes() };
  BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes, EXPECTED_GOT(expected_nr_nodes, resulting_nr_nodes));
}

BOOST_AUTO_TEST_CASE(gnn_2d_single_query_test)
{
  prx::distance_function_t metric = [](const prx::space_point_t& a, const prx::space_point_t& b) {
    return (Vec(a) - Vec(b)).norm();
  };
  prx::graph_nearest_neighbors_t gnn(metric);
  mock::space2d_t test{};
  prx::space_t& space{ test.space };

  /**
   * p0 ---- p1 --p2
   */
  prx::abstract_node_t* p0 = new prx::abstract_node_t();
  prx::abstract_node_t* p1 = new prx::abstract_node_t();
  prx::abstract_node_t* p2 = new prx::abstract_node_t();

  p0->point = space.make_point({ 0.0, 0.0 });
  p1->point = space.make_point({ 4.0, 0.0 });
  p2->point = space.make_point({ 6.0, 0.0 });

  gnn.add_node(p0);
  gnn.add_node(p1);
  gnn.add_node(p2);

  const long unsigned expected_nr_nodes{ 3 };
  const long unsigned resulting_nr_nodes{ gnn.get_nr_nodes() };
  BOOST_CHECK_MESSAGE(expected_nr_nodes == resulting_nr_nodes, EXPECTED_GOT(expected_nr_nodes, resulting_nr_nodes));

  const prx::space_point_t p0_query{ space.make_point() };
  const prx::space_point_t p1_query{ space.make_point() };
  const prx::space_point_t p2_query{ space.make_point() };

  space.copy(p0_query, { 1.0, 0.0 });
  space.copy(p1_query, { 4.5, 0.0 });
  space.copy(p2_query, { 6.5, 0.0 });

  const prx::space_point_t result_closest_to_p0{
    static_cast<prx::abstract_node_t*>(gnn.single_query(p0_query))->point
  };
  const prx::space_point_t result_closest_to_p1{
    static_cast<prx::abstract_node_t*>(gnn.single_query(p1_query))->point
  };
  const prx::space_point_t result_closest_to_p2{
    static_cast<prx::abstract_node_t*>(gnn.single_query(p2_query))->point
  };

  const prx::space_point_t expected_closest_to_p0{ p0->point };
  const prx::space_point_t expected_closest_to_p1{ p1->point };
  const prx::space_point_t expected_closest_to_p2{ p2->point };

  BOOST_CHECK_MESSAGE(space.equal_points(result_closest_to_p0, expected_closest_to_p0),
                      EXPECTED_GOT(result_closest_to_p0, expected_closest_to_p0));

  BOOST_CHECK_MESSAGE(space.equal_points(result_closest_to_p1, expected_closest_to_p1),
                      EXPECTED_GOT(result_closest_to_p1, expected_closest_to_p1));

  BOOST_CHECK_MESSAGE(space.equal_points(result_closest_to_p2, expected_closest_to_p2),
                      EXPECTED_GOT(result_closest_to_p2, expected_closest_to_p2));
}

BOOST_AUTO_TEST_CASE(gnn_2d_multi_query_test)
{
  using Point = prx::space_point_t;
  prx::distance_function_t metric = [](const prx::space_point_t& a, const prx::space_point_t& b) {
    return (Vec(a) - Vec(b)).norm();
  };
  prx::graph_nearest_neighbors_t gnn(metric);
  mock::space2d_t test{};
  prx::space_t& space{ test.space };

  std::vector<prx::abstract_node_t*> nodes;
  for (int i = 0; i < 10; ++i)
  {
    nodes.push_back(new prx::abstract_node_t());
    const Eigen::Vector2d initial_value{ Eigen::Vector2d::Ones() * i };
    nodes.back()->point = space.make_point(initial_value);
    gnn.add_node(nodes.back());
  }

  const Point query{ space.make_point({ 0.25, 0.25 }) };
  const std::size_t expected_vector_size{ 3 };
  const std::vector<prx::proximity_node_t*> result_closest{ gnn.multi_query(query, expected_vector_size) };

  BOOST_CHECK_MESSAGE(expected_vector_size == result_closest.size(),
                      EXPECTED_GOT(expected_vector_size, result_closest.size()));

  const Point result_close_0{ static_cast<prx::abstract_node_t*>(result_closest[0])->point };
  const Point result_close_1{ static_cast<prx::abstract_node_t*>(result_closest[1])->point };
  const Point result_close_2{ static_cast<prx::abstract_node_t*>(result_closest[2])->point };

  const Point expected_closest_to_p0{ space.make_point({ 0.0, 0.0 }) };
  const Point expected_closest_to_p1{ space.make_point({ 1.0, 1.0 }) };
  const Point expected_closest_to_p2{ space.make_point({ 2.0, 2.0 }) };

  BOOST_CHECK_MESSAGE(space.equal_points(result_close_0, expected_closest_to_p0),
                      EXPECTED_GOT(expected_closest_to_p0, result_close_0));

  BOOST_CHECK_MESSAGE(space.equal_points(result_close_1, expected_closest_to_p1),
                      EXPECTED_GOT(expected_closest_to_p1, result_close_1));

  BOOST_CHECK_MESSAGE(space.equal_points(result_close_2, expected_closest_to_p2),
                      EXPECTED_GOT(expected_closest_to_p2, result_close_2));
}

BOOST_AUTO_TEST_CASE(gnn_2d_radius_and_closest_query_test)
{
  using Point = prx::space_point_t;
  prx::distance_function_t metric = [](const prx::space_point_t& a, const prx::space_point_t& b) {
    return (Vec(a) - Vec(b)).norm();
  };
  prx::graph_nearest_neighbors_t gnn(metric);
  mock::space2d_t test{};
  prx::space_t& space{ test.space };

  std::vector<prx::abstract_node_t*> nodes;
  for (int i = 0; i < 10; ++i)
  {
    nodes.push_back(new prx::abstract_node_t());
    const Eigen::Vector2d initial_value{ Eigen::Vector2d::Ones() * i };
    nodes.back()->point = space.make_point(initial_value);
    gnn.add_node(nodes.back());
  }

  const Point query(space.make_point({ 5.1, 5.1 }));
  const double query_radius{ 2.0 };  // more than sqrt(2), less than 2sqrt(2)
  const std::vector<prx::proximity_node_t*> result_closest{ gnn.radius_and_closest_query(query, query_radius) };

  const std::size_t expected_vector_size{ 3 };
  BOOST_CHECK_MESSAGE(expected_vector_size == result_closest.size(),
                      EXPECTED_GOT(expected_vector_size, result_closest.size()));

  const Point result_close_0{ static_cast<prx::abstract_node_t*>(result_closest[0])->point };
  const Point result_close_1{ static_cast<prx::abstract_node_t*>(result_closest[1])->point };
  const Point result_close_2{ static_cast<prx::abstract_node_t*>(result_closest[2])->point };

  const Point expected_closest_to_p0{ space.make_point({ 5.0, 5.0 }) };
  const Point expected_closest_to_p1{ space.make_point({ 6.0, 6.0 }) };
  const Point expected_closest_to_p2{ space.make_point({ 4.0, 4.0 }) };

  BOOST_CHECK_MESSAGE(space.equal_points(result_close_0, expected_closest_to_p0),
                      EXPECTED_GOT(expected_closest_to_p0, result_close_0));

  BOOST_CHECK_MESSAGE(space.equal_points(result_close_1, expected_closest_to_p1),
                      EXPECTED_GOT(expected_closest_to_p1, result_close_1));

  BOOST_CHECK_MESSAGE(space.equal_points(result_close_2, expected_closest_to_p2),
                      EXPECTED_GOT(expected_closest_to_p2, result_close_2));
}