#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>

using SE2 = prx::fg::SE2_t;
using Angle = SE2::Angle;
using Rotation = SE2::Rotation;
using Translation = SE2::Translation;
using Vector = Eigen::Vector3d;
const double tolerance{ 1e-5 };

// Adjoint specific for now, but it should be possible to generalize it
class SE2_test_factor : public gtsam::NoiseModelFactorN<SE2, Vector>
{
  using Base = gtsam::NoiseModelFactorN<SE2, Vector>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

public:
  SE2_test_factor(const gtsam::Key kse2, const gtsam::Key kvec, const Vector v, const NoiseModel& cost_model)
    : Base(cost_model, kse2, kvec), _v(v)
  {
  }

  ~SE2_test_factor() override
  {
  }

  virtual Eigen::VectorXd evaluateError(const SE2& pose, const Vector& vec,  // no-lint
                                        OptDeriv Hse2 = boost::none, OptDeriv Hv = boost::none) const override
  {
    PRX_DBG_VARS(pose, vec.transpose());
    const Vector predicted{ pose.adjoint(vec, Hse2, Hv) };
    if (Hse2)
      PRX_DBG_VARS(*Hse2)
    if (Hv)
      PRX_DBG_VARS(*Hv)
    return (predicted - _v).head(2);
  }

private:
  const Vector _v;
};

BOOST_AUTO_TEST_CASE(constructor_test)
{
  const SE2 t0{};                                                    // default constructor
  const SE2 t1{ Translation::Zero(), 0.0 };                          // (vec, angle)
  const SE2 t2{ Translation::Zero(), Rotation::Identity() };         // (vec, Rotation)
  const SE2 t3{ Translation::Zero(), Eigen::Matrix2d::Identity() };  // (vec, Matrix2d)
  const SE2 t4{ 0, 0, 0 };                                           // (vec, Rotation)
  const SE2 t5{ gtsam::Pose2(0.0, 0.0, 0.0) };                       // (vec, Rotation)

  BOOST_REQUIRE_MESSAGE(t0.equals(t1), "t0 not equals t1");
  BOOST_REQUIRE_MESSAGE(t0.equals(t2), "t0 not equals t2");
  BOOST_REQUIRE_MESSAGE(t0.equals(t3), "t0 not equals t3");
  BOOST_REQUIRE_MESSAGE(t0.equals(t4), "t0 not equals t4");
  BOOST_REQUIRE_MESSAGE(t0.equals(t5), "t0 not equals t5");
}

BOOST_AUTO_TEST_CASE(test_static_adjointmap)
{
  const double x{ 1 };
  const double y{ 2 };
  const double th{ 3 };
  const Eigen::Vector3d v{ x, y, th };
  const Eigen::Matrix<double, 3, 3> adj_res{ SE2::adjoint_map(v) };
  Eigen::Matrix<double, 3, 3> adj_expected;
  adj_expected << 0, -th, y, th, 0, -x, 0, 0, 0;

  BOOST_REQUIRE_MESSAGE(adj_expected.isApprox(adj_res, tolerance), EXPECTED_GOT(adj_expected, adj_res));
}

BOOST_AUTO_TEST_CASE(test_adjoint_with_derivs)
{
  const Vector xi{ 2.3, 3.1, 1.4 };
  SE2 T1{ 1, 2, 3 };

  const Vector adj1{ T1.AdjointMap() * xi };
  const Vector adj1_expected{ T1.adjoint(xi) };

  BOOST_REQUIRE_MESSAGE(adj1_expected.isApprox(adj1, tolerance), EXPECTED_GOT(adj1_expected, adj1));

  // Check jacobians
  gtsam::Matrix3 actualH1, actualH2, expectedH1, expectedH2;
  std::function<Vector(const SE2&, const Vector&)> adjoint_proxy = [&](const SE2& T, const Vector& xi) {
    return T.adjoint(xi);
  };

  T1.adjoint(xi, actualH1, actualH2);
  expectedH1 = gtsam::numericalDerivative21(adjoint_proxy, T1, xi);
  expectedH2 = gtsam::numericalDerivative22(adjoint_proxy, T1, xi);
  BOOST_REQUIRE_MESSAGE(expectedH1.isApprox(actualH1, tolerance), EXPECTED_GOT(expectedH1, actualH1));
  BOOST_REQUIRE_MESSAGE(expectedH2.isApprox(actualH2, tolerance), EXPECTED_GOT(expectedH2, actualH2));
  // Check evaluation sanity check
}