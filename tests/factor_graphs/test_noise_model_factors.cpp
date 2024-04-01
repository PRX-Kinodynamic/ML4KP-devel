#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"

namespace mock
{
using X = Eigen::Vector<double, 3>;
class Model3 : public noise_model_3factor_t<X, X, X>
{
  Model3() : noise_model_3factor_t(gtsam::Key{}, gtsam::Key{}, gtsam::Key{}, nullptr, 0.1), _A(1, 2, 3), _B(4, 5, 6)
  {
  }

  virtual predict(const X& x1, const X& x2) override final
  {
    return x.transpose() * _A * x + y.transpose() * _B * y;
  }
  const Eigen::DiagonalMatrix<double, 3> _A;
  const Eigen::DiagonalMatrix<double, 3> _B;
}

}  // namespace mock

BOOST_AUTO_TEST_CASE(noise_model_factor3_error_with_derivatives_test)
{
  mock::Model3 model();
  const mock::X x0(1, 1, 1);
  const mock::X x1(2, 2, 2);
  const mock::X x2(3, 3, 3);
  model.predict() BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}
