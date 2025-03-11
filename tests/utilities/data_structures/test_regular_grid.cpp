#define BOOST_AUTO_TEST_MAIN spaces_test
#include <chrono>
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/data_structures/regular_grid.hpp"
#include "prx/utilities/defs.hpp"

// namespace mock

BOOST_AUTO_TEST_CASE(cube_cell_2D_constructor)
{
  using Cell = prx::utilities::cube_cell_t<double, 2>;

  const Cell::Coordinate vertex(2.0, 2.0);
  const Cell::Coordinate lengths(1.0, 0.5);
  Cell cell(vertex, lengths);

  const Cell::Coordinate c0_expected(2.0, 2.0);  // corner
  const Cell::Coordinate c1_expected(3.0, 2.0);  // corner + (1.0, 0.0)
  const Cell::Coordinate c2_expected(2.0, 2.5);  // corner + (0.0, 0.5)
  const Cell::Coordinate c3_expected(3.0, 2.5);  // corner + (1.0, 0.5)

  const Cell::Coordinate c0_res{ cell.vertex(0) };
  const Cell::Coordinate c1_res{ cell.vertex(1) };
  const Cell::Coordinate c2_res{ cell.vertex(2) };
  const Cell::Coordinate c3_res{ cell.vertex(3) };

  BOOST_REQUIRE_MESSAGE(c0_expected.isApprox(c0_res), EXPECTED_GOT(c0_expected, c0_res));
  BOOST_REQUIRE_MESSAGE(c1_expected.isApprox(c1_res), EXPECTED_GOT(c1_expected, c1_res));
  BOOST_REQUIRE_MESSAGE(c2_expected.isApprox(c2_res), EXPECTED_GOT(c2_expected, c2_res));
  BOOST_REQUIRE_MESSAGE(c3_expected.isApprox(c3_res), EXPECTED_GOT(c3_expected, c3_res));
  // BOOST_CHECK(tree.num_edges() == 0);
  // BOOST_CHECK(tree.capacity() == 0);
}

BOOST_AUTO_TEST_CASE(cube_cell_3D_constructor)
{
  using Cell = prx::utilities::cube_cell_t<double, 3>;

  const Cell::Coordinate vertex(1.0, 2.0, 3.0);
  const Cell::Coordinate lengths(1.0, 0.5, 2.0);
  Cell cell(vertex, lengths);

  const Cell::Coordinate c0_expected(1.0, 2.0, 3.0);
  const Cell::Coordinate c1_expected(2.0, 2.0, 3.0);
  const Cell::Coordinate c2_expected(1.0, 2.5, 3.0);
  const Cell::Coordinate c3_expected(2.0, 2.5, 3.0);
  const Cell::Coordinate c4_expected(1.0, 2.0, 5.0);
  const Cell::Coordinate c5_expected(2.0, 2.0, 5.0);
  const Cell::Coordinate c6_expected(1.0, 2.5, 5.0);
  const Cell::Coordinate c7_expected(2.0, 2.5, 5.0);

  const Cell::Coordinate c0_res{ cell.vertex(0) };
  const Cell::Coordinate c1_res{ cell.vertex(1) };
  const Cell::Coordinate c2_res{ cell.vertex(2) };
  const Cell::Coordinate c3_res{ cell.vertex(3) };
  const Cell::Coordinate c4_res{ cell.vertex(4) };
  const Cell::Coordinate c5_res{ cell.vertex(5) };
  const Cell::Coordinate c6_res{ cell.vertex(6) };
  const Cell::Coordinate c7_res{ cell.vertex(7) };

  BOOST_REQUIRE_MESSAGE(c0_expected.isApprox(c0_res), EXPECTED_GOT(c0_expected, c0_res));
  BOOST_REQUIRE_MESSAGE(c1_expected.isApprox(c1_res), EXPECTED_GOT(c1_expected, c1_res));
  BOOST_REQUIRE_MESSAGE(c2_expected.isApprox(c2_res), EXPECTED_GOT(c2_expected, c2_res));
  BOOST_REQUIRE_MESSAGE(c3_expected.isApprox(c3_res), EXPECTED_GOT(c3_expected, c3_res));
  BOOST_REQUIRE_MESSAGE(c4_expected.isApprox(c4_res), EXPECTED_GOT(c4_expected, c4_res));
  BOOST_REQUIRE_MESSAGE(c5_expected.isApprox(c5_res), EXPECTED_GOT(c5_expected, c5_res));
  BOOST_REQUIRE_MESSAGE(c6_expected.isApprox(c6_res), EXPECTED_GOT(c6_expected, c6_res));
  BOOST_REQUIRE_MESSAGE(c7_expected.isApprox(c7_res), EXPECTED_GOT(c7_expected, c7_res));
  // BOOST_CHECK(tree.num_edges() == 0);
  // BOOST_CHECK(tree.capacity() == 0);
}

BOOST_AUTO_TEST_CASE(cube_cell_3D_corners)
{
  using Cell = prx::utilities::cube_cell_t<double, 3>;

  const Cell::Coordinate vertex(1.0, 2.0, 3.0);
  const Cell::Coordinate lengths(1.0, 0.5, 2.0);
  Cell cell(vertex, lengths);

  const Cell::Coordinate c0_expected(1.0, 2.0, 3.0);
  const Cell::Coordinate c1_expected(2.0, 2.0, 3.0);
  const Cell::Coordinate c2_expected(1.0, 2.5, 3.0);
  const Cell::Coordinate c3_expected(2.0, 2.5, 3.0);
  const Cell::Coordinate c4_expected(1.0, 2.0, 5.0);
  const Cell::Coordinate c5_expected(2.0, 2.0, 5.0);
  const Cell::Coordinate c6_expected(1.0, 2.5, 5.0);
  const Cell::Coordinate c7_expected(2.0, 2.5, 5.0);

  const std::vector<Cell::Coordinate> vertices{ cell.vertices() };

  BOOST_REQUIRE_MESSAGE(c0_expected.isApprox(vertices[0]), EXPECTED_GOT(c0_expected, vertices[0]));
  BOOST_REQUIRE_MESSAGE(c1_expected.isApprox(vertices[1]), EXPECTED_GOT(c1_expected, vertices[1]));
  BOOST_REQUIRE_MESSAGE(c2_expected.isApprox(vertices[2]), EXPECTED_GOT(c2_expected, vertices[2]));
  BOOST_REQUIRE_MESSAGE(c3_expected.isApprox(vertices[3]), EXPECTED_GOT(c3_expected, vertices[3]));
  BOOST_REQUIRE_MESSAGE(c4_expected.isApprox(vertices[4]), EXPECTED_GOT(c4_expected, vertices[4]));
  BOOST_REQUIRE_MESSAGE(c5_expected.isApprox(vertices[5]), EXPECTED_GOT(c5_expected, vertices[5]));
  BOOST_REQUIRE_MESSAGE(c6_expected.isApprox(vertices[6]), EXPECTED_GOT(c6_expected, vertices[6]));
  BOOST_REQUIRE_MESSAGE(c7_expected.isApprox(vertices[7]), EXPECTED_GOT(c7_expected, vertices[7]));
  // BOOST_CHECK(tree.num_edges() == 0);
  // BOOST_CHECK(tree.capacity() == 0);
}

BOOST_AUTO_TEST_CASE(regular_grid_2D_create)
{
  constexpr std::size_t Dimension{ 2 };
  using Cell = prx::utilities::cube_cell_t<double, Dimension>;
  using Grid = prx::utilities::regular_grid_t<Cell, Dimension>;

  const Cell::Coordinate min(0.0, 1.0);
  const Cell::Coordinate max(1.0, 2.0);
  const Cell::Coordinate cell_length(0.5, 0.25);  // 2 cells in x, 4 in y

  Grid grid(min, max, cell_length);

  grid(0.2, 1.2)->element() = 1.0;
  grid(0.999, 1.999)->element() = 2.0;
  BOOST_REQUIRE(grid(0.21, 1.21)->element() == 1.0);
  BOOST_REQUIRE(grid(0.9, 1.9)->element() == 2.0);
}

BOOST_AUTO_TEST_CASE(regular_grid_2D_in_bounds)
{
  constexpr std::size_t Dimension{ 2 };
  using Cell = prx::utilities::cube_cell_t<double, Dimension>;
  using Grid = prx::utilities::regular_grid_t<Cell, Dimension>;

  const Cell::Coordinate min(0.0, 1.0);
  const Cell::Coordinate max(1.0, 2.0);
  const Cell::Coordinate cell_length(0.5, 0.25);

  Grid grid(min, max, cell_length);

  BOOST_REQUIRE(grid.in_bounds(min));
  BOOST_REQUIRE(not grid.in_bounds(min - min));
  BOOST_REQUIRE(not grid.in_bounds(max));
  BOOST_REQUIRE(grid.in_bounds(max - Cell::Coordinate(0.1, 0.1)));
}

BOOST_AUTO_TEST_CASE(cube_cell_3D_grid_access)
{
  constexpr std::size_t Dimension{ 3 };
  using Cell = prx::utilities::cube_cell_t<double, Dimension>;
  using Grid = prx::utilities::regular_grid_t<Cell, Dimension>;
  const Cell::Coordinate min(0.5, 1.0, 2.0);
  const Cell::Coordinate max(2.1, 2.0, 3.0);
  const Cell::Coordinate cell_length(0.5, 0.25, 0.1);

  Grid grid(min, max, cell_length);

  for (auto cell : grid)
  {
    BOOST_REQUIRE(cell != nullptr);
  }

  BOOST_REQUIRE(grid(min) != nullptr);
  BOOST_REQUIRE(grid(max - Cell::Coordinate(0.1, 0.1, 0.1)) != nullptr);
  BOOST_REQUIRE(grid(Cell::Coordinate(1.0, 1.5, 2.5)) != nullptr);
  BOOST_REQUIRE(grid(min + Cell::Coordinate(0.01, 0.01, 0.01))->vertex(0) == min);

  const Cell::Coordinate max_min_vertex{ grid(max - Cell::Coordinate(0.01, 0.01, 0.01))->vertex(0) };
  const Cell::Coordinate expected_max_min_vertex(2.0, 1.75, 2.9);
  BOOST_REQUIRE_MESSAGE((max_min_vertex - expected_max_min_vertex).isZero(),
                        EXPECTED_GOT(expected_max_min_vertex, max_min_vertex));
}

BOOST_AUTO_TEST_CASE(cube_cell_3D_grid_to_from_file)
{
  constexpr std::size_t Dimension{ 3 };
  using prx::utilities::convert_to;
  using Cell = prx::utilities::cube_cell_t<double, Dimension>;
  using Grid = prx::utilities::regular_grid_t<Cell, Dimension>;
  const Cell::Coordinate min(0.5, 1.0, 2.0);
  const Cell::Coordinate max(2.1, 2.0, 3.0);
  const Cell::Coordinate cell_length(0.5, 0.25, 0.1);

  Grid grid(min, max, cell_length);

  grid(min + cell_length)->element() = 1;
  grid(min + cell_length * 2.1)->element() = 2;

  const std::string filename{ "/tmp/grid_test" };
  auto ignore_cell = [](const Grid::CellPtr cell) { return cell != nullptr; };
  grid.to_file(filename, ignore_cell);

  auto line_to_element = [](const std::vector<std::string> line) { return convert_to<double>(line[0]); };
  Grid grid_from_file(filename, line_to_element);

  BOOST_REQUIRE(grid_from_file(min + cell_length)->element() == grid(min + cell_length)->element());
  BOOST_REQUIRE(grid_from_file(min + cell_length * 2.1)->element() == grid(min + cell_length * 2.1)->element());
}

BOOST_AUTO_TEST_CASE(cube_cell_3D_grid_with_custom_cell_constructor)
{
  constexpr std::size_t Dimension{ 3 };
  using Cell = prx::utilities::cube_cell_t<std::string, Dimension>;
  using Grid = prx::utilities::regular_grid_t<Cell, Dimension>;
  const Cell::Coordinate min(0.5, 1.0, 2.0);
  const Cell::Coordinate max(2.1, 2.0, 3.0);
  const Cell::Coordinate cell_length(0.5, 0.25, 0.1);

  const std::string init_element{ "test" };

  Grid grid(min, max, cell_length, init_element);

  for (auto cell : grid)
  {
    BOOST_REQUIRE(cell->element() == init_element);
  }
}