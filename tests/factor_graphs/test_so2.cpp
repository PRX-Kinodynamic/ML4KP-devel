#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/so2.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>

using SO2 = prx::fg::SO2_t;
using Vector = Eigen::Vector<double, 1>;
const double tolerance{ 1e-5 };

BOOST_AUTO_TEST_CASE(constructor_test)
{
  const SO2 t0{};                               // default constructor
  const SO2 t1{ 0.0 };                          // (vec, angle)
  const SO2 t2{ Eigen::Matrix2d::Identity() };  // (vec, Matrix2d)
  const SO2 t3{ gtsam::Rot2(0.0) };             // (vec, Rotation)

  BOOST_REQUIRE_MESSAGE(t0.equals(t1), "t0 not equals t1");
  BOOST_REQUIRE_MESSAGE(t0.equals(t2), "t0 not equals t2");
  BOOST_REQUIRE_MESSAGE(t0.equals(t3), "t0 not equals t3");
}

BOOST_AUTO_TEST_CASE(expmap_test)
{
  SO2 x0{ 1 };
  Vector v0(1);
  const SO2 xe{ SO2::Expmap(v0) };

  BOOST_REQUIRE_MESSAGE(x0.equals(xe), "x0 not equals xe");
}

// BOOST_AUTO_TEST_CASE(test_static_adjointmap)
// {
//   const double x{ 1 };
//   const double y{ 2 };
//   const double th{ 3 };
//   const Eigen::Vector3d v{ x, y, th };
//   const Eigen::Matrix<double, 3, 3> adj_res{ SE2::adjoint_map(v) };
//   Eigen::Matrix<double, 3, 3> adj_expected;
//   adj_expected << 0, -th, y, th, 0, -x, 0, 0, 0;

//   BOOST_REQUIRE_MESSAGE(adj_expected.isApprox(adj_res, tolerance), EXPECTED_GOT(adj_expected, adj_res));
// }

// BOOST_AUTO_TEST_CASE(test_adjoint_with_derivs)
// {
//   const Vector xi{ 2.3, 3.1, 1.4 };
//   SE2 T1{ 1, 2, 3 };

//   const Vector adj1{ T1.AdjointMap() * xi };
//   const Vector adj1_expected{ T1.adjoint(xi) };

//   BOOST_REQUIRE_MESSAGE(adj1_expected.isApprox(adj1, tolerance), EXPECTED_GOT(adj1_expected, adj1));

//   // Check jacobians
//   gtsam::Matrix3 actualH1, actualH2, expectedH1, expectedH2;
//   std::function<Vector(const SE2&, const Vector&)> adjoint_proxy = [&](const SE2& T, const Vector& xi) {
//     return T.adjoint(xi);
//   };

//   T1.adjoint(xi, actualH1, actualH2);
//   expectedH1 = gtsam::numericalDerivative21(adjoint_proxy, T1, xi);
//   expectedH2 = gtsam::numericalDerivative22(adjoint_proxy, T1, xi);
//   BOOST_REQUIRE_MESSAGE(expectedH1.isApprox(actualH1, tolerance), EXPECTED_GOT(expectedH1, actualH1));
//   BOOST_REQUIRE_MESSAGE(expectedH2.isApprox(actualH2, tolerance), EXPECTED_GOT(expectedH2, actualH2));
//   // Check evaluation sanity check
// }