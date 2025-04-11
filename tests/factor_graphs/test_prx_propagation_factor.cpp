#define BOOST_AUTO_TEST_MAIN prx_prop_factor_test
#include <string>
#include <boost/test/unit_test.hpp>

#include "prx/simulation/plant.hpp"

#include "prx/utilities/defs.hpp"

#include "prx/factor_graphs/factors/prx_propagation_factor.hpp"

const double tolerance{ 1e-5 };

namespace mock
{
// simple linear plant to easy test if derivatives are correct.
class linear_plant_t : public prx::plant_t
{
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  linear_plant_t(const std::string& path) : plant_t(path), _A(Eigen::Matrix4d::Identity()), _B(Eigen::Matrix4d::Zero())
  {
    state_memory = { &_x[0], &_x[1], &_x[2], &_x[3] };
    state_space = new prx::space_t("EEEE", state_memory, "dummy");

    derivative_memory = { &_xdot[0], &_xdot[1], &_xdot[2], &_xdot[3] };
    derivative_space = new prx::space_t("EEEE", derivative_memory, "dummy");

    control_memory = { &_u[0], &_u[1], &_u[2], &_u[3] };
    input_control_space = new prx::space_t("EEEE", control_memory, "dummy");
  }

  ~linear_plant_t() override
  {
  }

  // Ax + Bu
  virtual void propagate(const double step) override final
  {
    _A(0, 2) = step;
    _A(1, 3) = step;
    _B(0, 0) = step * step / 2.0;
    _B(1, 1) = step * step / 2.0;
    _B(2, 0) = step;
    _B(2, 1) = step;
    // PRX_DBG_VARS(_A);
    _x = _A * _x + _B * _u;
    // _x = _x + _xdot * step;
  }

private:
  virtual void update_configuration() override {};
  virtual void compute_derivative() override {};

  Eigen::Matrix4d _A;
  Eigen::Matrix4d _B;
  Eigen::Vector4d _x;
  Eigen::Vector4d _xdot;
  Eigen::Vector4d _u;
};
}  // namespace mock
PRX_REGISTER_SYSTEM(mock::linear_plant_t, mock_linear_plant)

BOOST_AUTO_TEST_CASE(prx_factor_derivs)
{
  using PrxFactor = prx::fg::plant_propagation_StateStateCtrl_factor_t<Eigen::Vector4d, Eigen::Vector4d, double>;
  prx::simulation_step = 0.1;

  const Eigen::Vector4d x0{ Eigen::Vector4d(1, 2, 3, 4) };
  const Eigen::Vector4d x1{ Eigen::Vector4d(2, 3, 4, 5) };
  const Eigen::Vector4d u{ Eigen::Vector4d::Ones() };
  const double dt{ 0.1 };

  Eigen::MatrixXd Hx1{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd Hx0{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd Hu{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd Hdt{ Eigen::Vector4d::Zero() };

  PrxFactor factor(0, 1, 2, 3, nullptr, "mock_linear_plant");
  factor.evaluateError(x1, x0, u, dt, Hx1, Hx0, Hu, Hdt);

  Eigen::Matrix4d Hx0_expected{ Eigen::Matrix4d::Identity() };
  Eigen::Matrix4d Hu_expected{ Eigen::Matrix4d::Zero() };

  Hx0_expected(0, 2) = dt;
  Hx0_expected(1, 3) = dt;
  Hu_expected(0, 0) = dt * dt / 2.0;
  Hu_expected(1, 1) = dt * dt / 2.0;
  Hu_expected(2, 0) = dt;
  Hu_expected(2, 1) = dt;

  Eigen::Matrix4d Hx1_err{ Hx1 + Eigen::Matrix4d::Identity() };  // Hx1 - (-I)
  Eigen::Matrix4d Hx0_err{ Hx0 - Hx0_expected };                 // Hx1 - (A)
  Eigen::Matrix4d Hu_err{ Hu - Hu_expected };                    // Hx1 - (B)
  // Eigen::Matrix4d Hdt_err{ Hdt - Hdt_expected };                 // Hx1 - (A)
  PRX_DBG_VARS(Hu);
  PRX_DBG_VARS(Hu_expected);
  PRX_DBG_VARS(Hu_err);

  BOOST_REQUIRE(Hx1_err.isZero());
  BOOST_REQUIRE(Hx0_err.isZero());
  BOOST_REQUIRE(Hu_err.isZero());
}

BOOST_AUTO_TEST_CASE(prx_factor_StateCteCtrl_derivs)
{
  using PrxFactor = prx::fg::plant_propagation_StateCteCtrl_factor_t<Eigen::Vector4d, Eigen::Vector4d, double>;
  prx::simulation_step = 0.1;

  const Eigen::Vector4d x0{ Eigen::Vector4d(1, 2, 3, 4) };
  const Eigen::Vector4d x1{ Eigen::Vector4d(2, 3, 4, 5) };
  const Eigen::Vector4d u{ Eigen::Vector4d::Ones() };
  const double dt{ 0.1 };

  Eigen::MatrixXd Hx1{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd Hu{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd Hdt{ Eigen::Vector4d::Zero() };

  PrxFactor factor(0, x0, 2, 3, nullptr, "mock_linear_plant");
  factor.evaluateError(x1, u, dt, Hx1, Hu, Hdt);

  Eigen::Matrix4d Hu_expected{ Eigen::Matrix4d::Zero() };

  Hu_expected(0, 0) = dt * dt / 2.0;
  Hu_expected(1, 1) = dt * dt / 2.0;
  Hu_expected(2, 0) = dt;
  Hu_expected(2, 1) = dt;

  Eigen::Matrix4d Hx1_err{ Hx1 + Eigen::Matrix4d::Identity() };  // Hx1 - (-I)
  Eigen::Matrix4d Hu_err{ Hu - Hu_expected };                    // Hx1 - (B)
  // Eigen::Matrix4d Hdt_err{ Hdt - Hdt_expected };                 // Hx1 - (A)
  PRX_DBG_VARS(Hu);
  PRX_DBG_VARS(Hu_expected);
  PRX_DBG_VARS(Hu_err);

  PRX_DBG_VARS(Hx1);
  PRX_DBG_VARS(Hx1_err);

  BOOST_REQUIRE(Hx1_err.isZero());
  BOOST_REQUIRE(Hu_err.isZero());
}

BOOST_AUTO_TEST_CASE(prx_factor_CteCteCtrl_derivs)
{
  using PrxFactor = prx::fg::plant_propagation_CteCteCtrl_factor_t<Eigen::Vector4d, Eigen::Vector4d, double>;
  prx::simulation_step = 0.1;

  const Eigen::Vector4d x0{ Eigen::Vector4d(1, 2, 3, 4) };
  const Eigen::Vector4d x1{ Eigen::Vector4d(2, 3, 4, 5) };
  const Eigen::Vector4d u{ Eigen::Vector4d::Ones() };
  const double dt{ 0.1 };

  Eigen::MatrixXd Hu{ Eigen::Matrix4d::Zero() };
  Eigen::MatrixXd Hdt{ Eigen::Vector4d::Zero() };

  PrxFactor factor(x1, x0, 2, 3, nullptr, "mock_linear_plant");
  factor.evaluateError(u, dt, Hu, Hdt);

  Eigen::Matrix4d Hu_expected{ Eigen::Matrix4d::Zero() };

  Hu_expected(0, 0) = dt * dt / 2.0;
  Hu_expected(1, 1) = dt * dt / 2.0;
  Hu_expected(2, 0) = dt;
  Hu_expected(2, 1) = dt;

  Eigen::Matrix4d Hu_err{ Hu - Hu_expected };  // Hx1 - (B)
  // Eigen::Matrix4d Hdt_err{ Hdt - Hdt_expected };                 // Hx1 - (A)
  PRX_DBG_VARS(Hu);
  PRX_DBG_VARS(Hu_expected);
  PRX_DBG_VARS(Hu_err);

  BOOST_REQUIRE(Hu_err.isZero());
}
