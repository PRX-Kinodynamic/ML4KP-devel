#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/noise_model_factors.hpp"
namespace mock
{
using Xres = Eigen::Vector<double, 1>;
using X = Eigen::Vector<double, 3>;
const double dt{ 0.01 };
struct Model3 : public prx::fg::noise_model_3factor_t<Xres, X, X>
{
  Model3()
    : prx::fg::noise_model_3factor_t<Xres, X, X>(gtsam::Key{}, gtsam::Key{}, gtsam::Key{}, nullptr, dt)
    , _A(1, 2, 3)
    , _B(4, 5, 6)
  {
  }

  virtual Xres predict(const X& x1, const X& x2) const override final
  {
    return x1.transpose() * _A * x1 + x2.transpose() * _B * x2;
  }
  const Eigen::DiagonalMatrix<double, 3> _A;
  const Eigen::DiagonalMatrix<double, 3> _B;
};

}  // namespace mock

BOOST_AUTO_TEST_CASE(noise_model_factor3_test_numerical_derivatives)
{
  mock::Model3 model;
  const mock::Xres x0(0);
  const mock::X x1(1, 1, 1);
  const mock::X x2(2, 2, 2);
  Eigen::MatrixXd H0;
  Eigen::MatrixXd H1;
  Eigen::MatrixXd H2;

  model.evaluateError(x0, x1, x2, H0, H1, H2);

  const Eigen::Matrix<double, 3, 3> A{ model._A };
  const Eigen::Matrix<double, 3, 3> B{ model._B };
  const Eigen::Matrix<double, 1, 1> H0_expected{ (Eigen::Matrix<double, 1, 1>() << -1).finished() };  // -1
  const Eigen::Matrix<double, 1, 3> H1_expected{ x1.transpose() * A + x1.transpose() * A.transpose() };  // A⋅x1+A⊤⋅x1
  const Eigen::Matrix<double, 1, 3> H2_expected{ x2.transpose() * B + x2.transpose() * B.transpose() };  // B⋅x2+B⊤⋅x2

  BOOST_REQUIRE_MESSAGE(H0_expected.isApprox(H0), EXPECTED_GOT(H0_expected, H0));
  BOOST_REQUIRE_MESSAGE(H1_expected.isApprox(H1), EXPECTED_GOT(H1_expected, H1));
  BOOST_REQUIRE_MESSAGE(H2_expected.isApprox(H2), EXPECTED_GOT(H2_expected, H2));
}
