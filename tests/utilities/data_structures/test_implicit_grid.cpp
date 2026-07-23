#include "math/lie_utils.hpp"
#define BOOST_AUTO_TEST_MAIN spaces_test
#include <chrono>
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/streamer.hpp"
#include "prx/utilities/data_structures/implicit_grid.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/streamer.hpp"

namespace mock
{
struct cell_t
{
  cell_t() : idx(std::numeric_limits<std::size_t>::max()) {};
  std::size_t idx;
};
}  // namespace mock

BOOST_AUTO_TEST_CASE(test_implicit_grid)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;
  Grid grid;

  LieType x0(gtsam::Rot2(0), 0);
  Tangent cell_size(0.3, 0.5);
  grid.reset(x0, cell_size);

  grid.cell(x0).idx = 0;

  LieType eps_1{ gtsam::traits<LieType>::Expmap(Eigen::Vector2d(0.1, 0.1)) };
  LieType x1(x0.compose(eps_1));
  BOOST_CHECK(grid.cell(x0).idx == 0);
  BOOST_CHECK(grid.cell(x1).idx == 0);

  LieType eps_2{ gtsam::traits<LieType>::Expmap(Eigen::Vector2d(0.4, 0.1)) };
  LieType x2(x0.compose(eps_2));
  // prx::streamer_t<LieType>::to_stream(std::cout, eps_2);
  // std::cout << "\n";

  grid.cell(x2).idx = 1;

  LieType x3(x0.compose(eps_1).compose(eps_2));
  BOOST_CHECK(grid.cell(x2).idx == 1);
  BOOST_CHECK(grid.cell(x3).idx == 1);

  LieType eps_3{ gtsam::traits<LieType>::Expmap(Eigen::Vector2d(0.1, 0.6)) };
  LieType x4(x0.compose(eps_3));
  // prx::streamer_t<LieType>::to_stream(std::cout, eps_2);
  // std::cout << "\n";

  grid.cell(x4).idx = 2;

  LieType x5(x0.compose(eps_1).compose(eps_3));
  BOOST_CHECK(grid.cell(x4).idx == 2);
  BOOST_CHECK(grid.cell(x5).idx == 2);
}

// TODO: Write checks for this test
BOOST_AUTO_TEST_CASE(test_implicit_grid_vertices)
{
  using LieType = gtsam::Pose2;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;
  using Vertex = prx::implicit_grid_t<LieType, mock::cell_t>::Vertex;

  Grid grid;

  LieType x0{ gtsam::Pose2(0, 0, 0) };
  Tangent cell_size(0.1, 0.2, 0.3);
  grid.reset(x0, cell_size);

  std::vector<Vertex> vertices{ grid.vertices(x0) };
  for (auto v : vertices)
  {
    prx::to_stream(std::cout, v);
    std::cout << "\n";
  }
}

BOOST_AUTO_TEST_CASE(test_hash)
{
  using LieType = gtsam::Pose2;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;

  Grid grid;
  grid.reset(LieType(), 0.01);

  Tangent eps0{ Tangent::Ones() * 0.1 };
  Tangent eps1{ Tangent::Ones() * 0.12 };
  Tangent eps2{ Tangent::Ones() * 0.08 };
  Tangent eps3{ Tangent::Ones() * 0.01 };
  Tangent eps4{ Tangent::Ones() * 0.21 };
  Tangent eps5{ Tangent::Ones() * 0.31 };
  Tangent eps6{ Tangent::Ones() * 0.05 };
  Tangent eps7{ Tangent::Ones() * 0.100001 };  // Small enough epsilon gets same hash as eps0: eps0 ~= eps7

  const std::size_t h0{ grid.hash(eps0) };
  const std::size_t h1{ grid.hash(eps1) };
  const std::size_t h2{ grid.hash(eps2) };
  const std::size_t h3{ grid.hash(eps3) };
  const std::size_t h4{ grid.hash(eps4) };
  const std::size_t h5{ grid.hash(eps5) };
  const std::size_t h6{ grid.hash(eps6) };
  const std::size_t h7{ grid.hash(eps7) };  // <- Gets the same hash  as h0

  PRX_DBG_VARS(h0, h1, h2, h3, h4, h5, h6, h7)

  BOOST_CHECK(h0 != h1);

  BOOST_CHECK(h0 != h2);
  BOOST_CHECK(h1 != h2);

  BOOST_CHECK(h0 != h3);
  BOOST_CHECK(h1 != h3);
  BOOST_CHECK(h2 != h3);

  BOOST_CHECK(h0 != h4);
  BOOST_CHECK(h1 != h4);
  BOOST_CHECK(h2 != h4);
  BOOST_CHECK(h3 != h4);

  BOOST_CHECK(h0 != h5);
  BOOST_CHECK(h1 != h5);
  BOOST_CHECK(h2 != h5);
  BOOST_CHECK(h3 != h5);
  BOOST_CHECK(h4 != h5);

  BOOST_CHECK(h0 != h6);
  BOOST_CHECK(h1 != h6);
  BOOST_CHECK(h2 != h6);
  BOOST_CHECK(h3 != h6);
  BOOST_CHECK(h4 != h6);
  BOOST_CHECK(h5 != h6);

  // This are the same hash
  BOOST_CHECK(h0 == h7);

  LieType x0{ gtsam::traits<LieType>::Expmap(eps0) };
  const std::size_t hx0{ grid.hash(x0) };
  PRX_DBG_VARS(h0, hx0)
}

BOOST_AUTO_TEST_CASE(test_cell_center)
{
  using LieType = Eigen::Vector2d;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;

  Grid grid;
  const LieType x0{ LieType::Zero() };
  const Tangent cell_size{ Tangent::Ones() * 0.1 };
  grid.reset(x0, cell_size);

  std::ofstream ofs("/tmp/grid.txt");
  for (double i = -1.; i < 1.0; i += 0.01)
  {
    for (double j = -1.; j < 1.0; j += 0.01)
    {
      ofs << grid.center(Tangent(i, j)).transpose() << "\n";
    }
  }
}

BOOST_AUTO_TEST_CASE(lie_hash_v2)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;

  Grid grid;

  const LieType x0{ LieType(gtsam::Rot2(2.), -1) };
  const Tangent cell_size{ Tangent::Ones() * 0.3 };
  grid.reset(x0, cell_size);

  const LieType x1{ LieType(gtsam::Rot2(1.95), -1.16) };

  const std::size_t hx0{ grid.hash(x0) };
  const std::size_t hx1{ grid.hash(x1) };
  PRX_DBG_VARS(hx0, hx1)
  BOOST_CHECK(hx0 != hx1);
}

BOOST_AUTO_TEST_CASE(test_insertion)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, int>;
  using Tangent = Grid::TangentElement;

  Grid grid;

  const LieType x0{ LieType(gtsam::Rot2(0.), 0.) };
  const Tangent cell_size{ Tangent::Ones() * 0.1 };
  grid.reset(x0, cell_size);

  int idx{ 1 };
  const LieType xtest{ LieType(gtsam::Rot2(-3.13659), -3.14159) };
  // PRX_DBG_VARS(grid.size());
  grid.cell(xtest) = idx;
  // PRX_DBG_VARS(grid.size());
  BOOST_REQUIRE_MESSAGE(grid.cell(xtest) == idx, EXPECTED_GOT(idx, grid.cell(xtest)));
  idx++;

  for (double i = -3.14159; i < 3.14159; i += 0.09)
  {
    for (double j = -3.14159; j < 3.14159; j += 0.09)
    {
      const LieType x1{ LieType(gtsam::Rot2(i), j) };
      grid.cell(x1) = idx;
      // PRX_DBG_VARS(i, j, grid.size());
      BOOST_REQUIRE_MESSAGE(grid.cell(x1) == idx, EXPECTED_GOT(idx, grid.cell(x1)));
      idx++;
    }
  }
}

BOOST_AUTO_TEST_CASE(test_state_from_tangent_space)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, int>;
  using Tangent = Grid::TangentElement;

  Grid grid;

  const LieType x0{ LieType(gtsam::Rot2(0.), 2.) };
  const Tangent cell_size{ Tangent::Ones() * 0.1 };
  grid.reset(x0, cell_size);

  auto vertices = grid.vertices(x0);
  for (auto v : vertices)
  {
    const LieType xv{ grid.state_from_vertex(v) };
    const Tangent tg{ prx::TangentBetween(xv, x0) };
    PRX_DEBUG_VARS(tg.transpose())
  }

  const LieType x1{ LieType(gtsam::Rot2(0.), 0.2) };
  std::cout << "\nx1: ";
  prx::to_stream(std::cout, x1);
  vertices = grid.vertices(x1);
  for (auto v : vertices)
  {
    const LieType xv{ grid.state_from_vertex(v) };
    std::cout << "\nv: ";
    prx::to_stream(std::cout, v);
    std::cout << "\nxv: ";
    prx::to_stream(std::cout, xv);
    const Tangent tg{ prx::TangentBetween(xv, x1) };
    std::cout << "\n";
    PRX_DEBUG_VARS(tg.transpose())
  }

  // const LieType x1{ LieType(gtsam::Rot2(0.), 0.2) };
  // const LieType xv0{ grid.state(vertices[0]) };
  // const LieType xv1{ grid.state(vertices[1]) };
  // const LieType xv2{ grid.state(vertices[2]) };
  // const LieType xv3{ grid.state(vertices[3]) };

  // prx::to_stream(std::cout, xv0);
  // prx::to_stream(std::cout, xv1);
  // prx::to_stream(std::cout, xv2);
  // prx::to_stream(std::cout, xv3);
  // gtsam::traits<LieType>::Logmap(xv0);
}

BOOST_AUTO_TEST_CASE(test_implicit_grid_reset_fg_cell_sizes)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;
  std::cout << "\n\n";
  PRX_MSG("test_implicit_grid_reset_fg_cell_sizes");

  Grid grid;

  LieType x0{ LieType() };
  LieType x1{ LieType(1, 1) };
  // LieType x1{ LieType(.15, .15) };
  // Tangent cell_size(0.1, 0.2, 0.3);
  grid.reset(x0, 0.1);

  PRX_DBG_VARS(x0)
  auto vertices = grid.vertices(x0);
  for (auto& v : vertices)
  {
    const LieType xv{ grid.state_from_vertex(v) };
    PRX_DBG_VARS(v, xv);
  }

  PRX_DBG_VARS(x1)
  vertices = grid.vertices(x1);
  PRX_DBG_VARS(vertices)
  for (auto& v : vertices)
  {
    const LieType xv{ grid.state_from_vertex(v) };
    PRX_DBG_VARS(v, xv);
  }
}

BOOST_AUTO_TEST_CASE(test_implicit_grid_points_vertices)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;

  std::cout << "\n\n";
  PRX_MSG("test_implicit_grid_points_vertices");

  Grid grid;
  const double cell_size{ 0.1 };

  LieType x0{ LieType(0.0, 0.0) };
  grid.reset(x0, cell_size);

  // LieType x1{ LieType(0.15, 2.15) };
  // LieType x2{ LieType(0.15, 1.75) };

  LieType x1{ LieType(0.15, -0.15) };
  LieType x2{ LieType(-0.15, -0.75) };

  for (auto xi : { x1, x2 })
  {
    PRX_DBG_VARS(xi);
    const Tangent tg_xi{ gtsam::traits<LieType>::Logmap(xi) };
    auto vertices = grid.vertices(xi);
    for (auto& v : vertices)
    {
      const LieType xv{ grid.state_from_vertex(v) };
      PRX_DBG_VARS(xv);
      const Tangent tg_v{ gtsam::traits<LieType>::Logmap(xv) };
      const double err{ (tg_xi - tg_v).norm() };
      PRX_DBG_VARS(err, cell_size)
      BOOST_REQUIRE(err < cell_size);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_implicit_grid_center)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Tangent = prx::implicit_grid_t<LieType, mock::cell_t>::TangentElement;

  std::cout << "\n\n";
  PRX_MSG("test_implicit_grid_points_vertices");

  Grid grid;
  const double cell_size{ 0.05 / 2. };

  LieType x0{ LieType(-1.5 - cell_size / 2, 2.0 - cell_size / 2) };
  grid.reset(x0, cell_size);

  LieType x1{ LieType(-1.51, 2.0) };
  LieType x2{ LieType(-1.49, 2.0) };
  LieType x3{ LieType(-1.53, 2.0) };
  LieType x4{ LieType(-1.56, 2.0) };
  LieType x5{ LieType(-1.45, 2.0) };

  // LieType x1{ LieType(0.15, -0.15) };
  // LieType x2{ LieType(-0.15, -0.75) };
  for (auto xi : { x0, x1, x2, x3, x4, x5 })
  {
    auto vertices = grid.vertices(xi);
    for (auto& v : vertices)
    {
      const LieType xv{ grid.state_from_vertex(v) };
      PRX_DBG_VARS(xi, xv, v);
      // const Tangent tg_v{ gtsam::traits<LieType>::Logmap(xv) };
      // PRX_DBG_VARS(err, cell_size)
      // BOOST_REQUIRE(err < cell_size);
    }
  }
}

BOOST_AUTO_TEST_CASE(lie_hash_vertex_equal_opositve_sign)
{
  using LieType = gtsam::ProductLieGroupV43<gtsam::Rot2, double>;
  using Grid = prx::implicit_grid_t<LieType, mock::cell_t>;
  using Vertex = prx::implicit_grid_t<LieType, mock::cell_t>::Vertex;

  Grid grid;

  Vertex v0(1, 1);
  Vertex v1(1, -1);
  Vertex v2(-1, 2);
  Vertex v3(-1, -2);

  const std::size_t hx0{ grid.hash(v0) };
  const std::size_t hx1{ grid.hash(v1) };
  const std::size_t hx2{ grid.hash(v2) };
  const std::size_t hx3{ grid.hash(v3) };
  PRX_DBG_VARS(hx0, hx1, hx2, hx3)
  BOOST_CHECK(hx0 != hx1);
  BOOST_CHECK(hx0 != hx2);
  BOOST_CHECK(hx0 != hx3);

  BOOST_CHECK(hx1 != hx2);
  BOOST_CHECK(hx1 != hx3);

  BOOST_CHECK(hx2 != hx3);
}