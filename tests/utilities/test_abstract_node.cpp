#define BOOST_AUTO_TEST_MAIN abstract_node
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/data_structures/abstract_node.hpp"

struct test_abstract_node_t : public prx::abstract_node_t
{
  test_abstract_node_t() : prx::abstract_node_t()
  {
  }
};

BOOST_AUTO_TEST_CASE(abstract_node_casts_correctly)
{
  test_abstract_node_t* node = new test_abstract_node_t();

  BOOST_CHECK(node->as<prx::abstract_node_t>() != nullptr);
  BOOST_CHECK(node->as<prx::proximity_node_t>() != nullptr);
}

BOOST_AUTO_TEST_CASE(abstract_node_point_access_is_correct)
{
  double x, y, theta;
  x = y = theta = 1;
  std::vector<double*> address_1 = { &x, &y, &theta };
  std::shared_ptr<prx::space_t> space{ std::make_shared<prx::space_t>("EER", address_1, "space_1") };

  std::shared_ptr<prx::abstract_node_t> node{ std::make_shared<prx::abstract_node_t>() };
  node->point = space->make_point();
  space->copy_point_from_vector(node->point, { 1.0, 2.0, 3.0 });
  BOOST_CHECK_MESSAGE(node->point->at(0) == 1.0, "Expected: 0; Got:" << node->point->at(0));
  BOOST_CHECK_MESSAGE(node->point->at(1) == 2.0, "Expected: 1; Got:" << node->point->at(1));
  BOOST_CHECK_MESSAGE(node->point->at(2) == 3.0, "Expected: 2; Got:" << node->point->at(2));
}
