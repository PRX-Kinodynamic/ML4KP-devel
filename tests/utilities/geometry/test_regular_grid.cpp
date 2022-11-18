#define BOOST_AUTO_TEST_MAIN regular_grid_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/utilities/geometry/regular_grid.hpp"

BOOST_AUTO_TEST_CASE(one_dim_simple_test)
{
  const std::size_t DIM{ 1 };
  const std::vector<std::pair<double, double>> bounds{ std::make_pair(-10.0, 10.0) };
  prx::regular_grid_t<int, DIM> grid(bounds, 20);

  const int expected_value{ 5 };
  const double cell{ 5.0 };
  grid(cell) = expected_value;

  BOOST_CHECK_MESSAGE(grid(cell) == expected_value, "Expected: " << expected_value << " got:" << grid(cell));
}

BOOST_AUTO_TEST_CASE(one_dim_mapping_test)
{
  const std::size_t DIM{ 1 };
  const std::vector<std::pair<double, double>> bounds{ std::make_pair(0.0, 10.0) };
  prx::regular_grid_t<double, DIM> grid{ bounds, 4.0 };

  grid(0.0) = 0.0;
  grid(1.0) = 1.0;
  grid(2.0) = 2.0;
  grid(3.0) = 3.0;
  grid(4.0) = 4.0;
  grid(5.0) = 5.0;
  grid(6.0) = 6.0;
  grid(7.0) = 7.0;
  grid(8.0) = 8.0;
  grid(9.0) = 9.0;

  BOOST_CHECK_MESSAGE(grid(0.0) == 2.0, "Expected: " << 2.0 << " grid(0.0) got:" << grid(0.0));
  BOOST_CHECK_MESSAGE(grid(1.0) == 2.0, "Expected: " << 2.0 << " grid(1.0) got:" << grid(1.0));
  BOOST_CHECK_MESSAGE(grid(2.0) == 2.0, "Expected: " << 2.0 << " grid(2.0) got:" << grid(2.0));
  BOOST_CHECK_MESSAGE(grid(3.0) == 4.0, "Expected: " << 4.0 << " grid(3.0) got:" << grid(3.0));
  BOOST_CHECK_MESSAGE(grid(4.0) == 4.0, "Expected: " << 4.0 << " grid(4.0) got:" << grid(4.0));
  BOOST_CHECK_MESSAGE(grid(5.0) == 7.0, "Expected: " << 7.0 << " grid(5.0) got:" << grid(5.0));
  BOOST_CHECK_MESSAGE(grid(6.0) == 7.0, "Expected: " << 7.0 << " grid(6.0) got:" << grid(6.0));
  BOOST_CHECK_MESSAGE(grid(7.0) == 7.0, "Expected: " << 7.0 << " grid(7.0) got:" << grid(7.0));
  BOOST_CHECK_MESSAGE(grid(8.0) == 9.0, "Expected: " << 9.0 << " grid(8.0) got:" << grid(8.0));
  BOOST_CHECK_MESSAGE(grid(9.0) == 9.0, "Expected: " << 9.0 << " grid(9.0) got:" << grid(9.0));
}

BOOST_AUTO_TEST_CASE(one_dim_value_test)
{
  const std::size_t DIM{ 1 };
  const std::vector<std::pair<double, double>> bounds{ std::make_pair(-10.0, 10.0) };
  prx::regular_grid_t<std::pair<int, double>, DIM> grid{ bounds, 20 };

  const std::pair<int, double> expected_value{ 3, M_PI };
  const double cell{ 5.0 };
  grid(cell) = expected_value;

  BOOST_CHECK_MESSAGE(grid(cell).first == expected_value.first,
                      "Expected: " << expected_value.first << " got:" << grid(cell).first);
  BOOST_CHECK_MESSAGE(grid(cell).second == expected_value.second,
                      "Expected: " << expected_value.second << " got:" << grid(cell).second);
}

BOOST_AUTO_TEST_CASE(two_dim_test)
{
  const std::size_t DIM{ 2 };
  // Grid in 2D of environment [0,10]x[0,10]
  const std::vector<std::pair<double, double>> bounds{ std::make_pair(0.0, 10.0), std::make_pair(0.0, 10.0) };
  // Grid has 4 cells: 2 divisions in each dimension
  prx::regular_grid_t<int, DIM> grid{ bounds, 2 };

  // Set a value in each cell
  grid(2.5, 2.5) = 1;  // \in [0,5)x[0,5)
  grid(2.5, 7.5) = 2;  // \in [0,5)x[5,10)
  grid(7.5, 2.5) = 3;  // \in [5,10)x[0,5)
  grid(7.5, 7.5) = 4;  // \in [5,10)x[5,10)

  // Check that random points in each cell return the expected values.
  for (int i = 0; i < 100; ++i)
  {
    BOOST_REQUIRE(grid(prx::uniform_random(0.0, 5.0), prx::uniform_random(0.0, 5.0)) == 1);
    BOOST_REQUIRE(grid(prx::uniform_random(0.0, 5.0), prx::uniform_random(5.0, 10.0)) == 2);
    BOOST_REQUIRE(grid(prx::uniform_random(5.0, 10.0), prx::uniform_random(0.0, 5.0)) == 3);
    BOOST_REQUIRE(grid(prx::uniform_random(5.0, 10.0), prx::uniform_random(5.0, 10.0)) == 4);
  }
}

BOOST_AUTO_TEST_CASE(multimatch_test)
{
  const std::size_t DIM{ 2 };
  using value_t = int;
  // Grid in 2D of environment [0,10]x[0,10]
  const std::vector<std::pair<double, double>> bounds{ std::make_pair(0.0, 10.0), std::make_pair(0.0, 10.0) };
  // Grid has 4 cells: 2 divisions in each dimension
  prx::regular_grid_t<value_t, DIM> grid{ bounds, 2 };

  // Set a value in each cell
  grid(2.5, 2.5) = 1;  // \in [0,5)x[0,5)
  grid(2.5, 7.5) = 2;  // \in [0,5)x[5,10)
  grid(7.5, 2.5) = 3;  // \in [5,10)x[0,5)
  grid(7.5, 7.5) = 4;  // \in [5,10)x[5,10)

  // std::vector<std::pair<prx::regular_grid_t<value_t, DIM>::key_t, value_t>> found_0 = grid<100>(2.5, 100);
  // auto found_0 = grid<100>(2.5, 100);
  // BOOST_REQUIRE(found_0.size() == 2);
  // BOOST_REQUIRE(found_0[0].second == 1 || found_0[0].second == 2);
  // BOOST_REQUIRE(found_0[1].second == 1 || found_0[1].second == 2);
  // BOOST_REQUIRE(grid(found_0[0].first) == 1 || grid(found_0[0].first) == 2);
  // BOOST_REQUIRE(grid(found_0[1].first) == 1 || grid(found_0[1].first) == 2);

  // Check that random points in each cell return the expected values.
}
