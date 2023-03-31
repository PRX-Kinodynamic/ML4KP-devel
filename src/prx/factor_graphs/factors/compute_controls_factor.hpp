#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/factor_graphs/utilities/prx_symbols.hpp"

namespace prx
{

/**
 * space_limit_factor_t is a class which enforces limits to states
 */
class compute_controls_factor_t : public gtsam::NoiseModelFactor2<Eigen::VectorXd, Eigen::VectorXd>
{
private:
  using This = compute_controls_factor_t;
  using Base = gtsam::NoiseModelFactor2<Eigen::VectorXd, Eigen::VectorXd>;
  system_ptr_t system_ptr;

public:
  /**
   * Construct from joint limits
   * @param q_key joint value key
   * @param cost_model noise model
   * @param lower_limit joint lower limit
   * @param upper_limit joint upper limit
   */
  compute_controls_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key q_key, gtsam::Key u_key,
                            system_ptr_t _sys_ptr)
    : Base(cost_model, q_key, u_key)
  {
    system_ptr = _sys_ptr;
  }

  virtual ~compute_controls_factor_t()
  {
  }

public:
  Eigen::VectorXd compute_error(const Eigen::VectorXd& xt0, const Eigen::VectorXd& ut1) const
  {
    auto ss = system_ptr->get_state_space();
    auto cs = system_ptr->get_control_space();
    auto ss_dim = ss->get_dimension();
    auto cs_dim = cs->get_dimension();
    Eigen::VectorXd error(cs_dim);

    // ss -> copy_from_vector(xt0);
    double mass = 1.0;
    double g = 9.81;
    double l1 = 1.0;
    double l2 = 1.0;
    double I1 = 0.2;
    double I2 = 1.0;
    double d1 = 1.0;  // Damping
    double d2 = 1.0;
    const double theta2 = xt0[1];
    const double theta1 = xt0[0] - M_PI / 2.0;
    const double theta1dot = xt0[2];
    const double theta2dot = xt0[3];

    const double lc1 = l1 / 2.0;
    const double lc2 = l2 / 2.0;
    // TODO: Change to m1 & m2
    double m = mass;

    Eigen::Vector2d th_dot;
    Eigen::Matrix2d M;

    const double d11 = m * lc1 * lc1 + m * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc1 * cos(theta2)) + I1 + I2;
    const double d22 = m * lc2 * lc2 + I2;
    const double d12 = m * (lc2 * lc2 + l1 * lc2 * cos(theta2)) + I2;
    const double d21 = d12;

    const double c1 = -m * l1 * lc2 * theta2dot * theta2dot * sin(theta2) -
                      (2.0 * m * l1 * lc2 * theta1dot * theta2dot * sin(theta2));
    const double c2 = m * l1 * lc2 * theta1dot * theta1dot * sin(theta2);

    Eigen::Vector2d C;
    Eigen::Vector2d G;
    C(0) = c1;
    C(1) = c2;

    th_dot(0) = theta1dot;
    th_dot(1) = theta2dot;
    M(0, 0) = d11;
    M(1, 0) = d21;
    M(0, 1) = d12;
    M(1, 1) = d22;

    const double g1 = (m * lc1 + m * l1) * g * cos(theta1) + (m * lc2 * g * cos(theta1 + theta2));
    const double g2 = m * lc2 * g * cos(theta1 + theta2);
    G << g1, g2;
    // auto M = system_ptr -> get_mass_matrix();
    // auto C = system_ptr -> get_coriolis_vector();
    // Eigen::VectorXd G = system_ptr -> get_gravity_vector();
    Eigen::VectorXd u = M * th_dot + C + G;
    // Eigen::VectorXd u = G ;
    // Eigen::VectorXd u = Eigen::VectorXd::Zero(cs_dim);
    // std::cout << "u: " << u.transpose() << std::endl;
    // std::cout << "ut1: " << ut1.transpose() << std::endl;
    error = ut1 - u;
    // PRX_DEBUG_PRINT
    return error;
    // cs -> copy_from_vector(ut1);
    // system_ptr -> compute_control();

    // cs -> difference(error_pt, xt, error_pt);
    // cs -> copy_vector_from_point(error, error_pt);
  }

  /**
   * Evaluate joint limit errors
   *
   * @param q joint value
   */
  gtsam::Vector evaluateError(const Eigen::VectorXd& X1, const Eigen::VectorXd& X2,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none) const override
  {
    // auto error = compute_error(X1, X2);
    // std::cout << "prop error: " << error.transpose() << std::endl;
    if (H1)
    {
      std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
          std::bind(&compute_controls_factor_t::compute_error, this, std::placeholders::_1, X2);
      *H1 = math_functions::differentiate(fp, X1);
    }

    if (H2)
    {
      std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
          std::bind(&compute_controls_factor_t::compute_error, this, X1, std::placeholders::_1);
      *H2 = math_functions::differentiate(fp, X2);
    }
    return compute_error(X1, X2);
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string& s = "", const gtsam::KeyFormatter& kf = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << s << "compute_controls_factor";
    Base::print("", kf);
  }
};

}  // namespace prx