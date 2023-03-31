#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{

class state_propagation_factor_t : public gtsam::NoiseModelFactor3<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
{
public:
  /**
   * @brief      Constructs a new instance xt1 = xt0 + xdt1 * dt
   *
   * @param[in]  gtsam     the cost mode
   * @param[in]  xt0_key   The xt0 key
   * @param[in]  xt1_key   The xt1 key
   * @param[in]  xdt1_key  The xdt1 key
   * @param[in]  _sys_ptr  The system pointer
   */
  state_propagation_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key xt0_key,
                             gtsam::Key xt1_key,  // gtsam::Key xdt1_key,
                             gtsam::Key ut1_key, int _dim_i, system_ptr_t _sys_ptr)
    : Base(cost_model, xt0_key, xt1_key, ut1_key)
  {
    // ltv = std::dynamic_pointer_cast<ltv_t>(_sys_ptr);
    ltv = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
    // ltv = _sys_ptr;
    prx_assert(ltv != nullptr, "Can't cast to plant_t!");

    auto ss = ltv->get_state_space();
    xt = ss->make_point();
    ut = ltv->get_control_space()->make_point();
    error_pt = ss->make_point();
    dim_i = _dim_i;

    prx_assert(0 <= dim_i < ss->get_dimension(), "dim_i must be \\in [0, " << ss->get_dimension() << ").");
  }

  virtual ~state_propagation_factor_t()
  {
  }

public:
  virtual Eigen::VectorXd evaluateError(const X1&, const X2&, const X3&,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none) const override;

  Eigen::VectorXd compute_error(Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1) const;

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << s << "state_propagation_factor";
    Base::print("", keyFormatter);
  }

private:
  using This = state_propagation_factor_t;
  using Base = gtsam::NoiseModelFactor3<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
  // std::shared_ptr<ltv_t> ltv;
  std::shared_ptr<plant_t> ltv;

  space_point_t xt;
  space_point_t ut;
  space_point_t error_pt;

  space_point_t mem_aux;
  int dim_i;
};

}  // namespace prx