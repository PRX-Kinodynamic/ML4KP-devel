#pragma once
#include <algorithm>
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
// #include "prx/factor_graphs/utilities/prx_symbols.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
namespace fg
{
template <Eigen::Index Xdim, Eigen::Index Udim, Eigen::Index THdim>
class mj_friction_propagation_t : public noise_model_4factor_t<Xdim, Xdim, Udim, THdim>
{
  using Base = noise_model_4factor_t<Xdim, Xdim, Udim, THdim>;

public:
  using State = Eigen::Vector<double, Xdim>;
  // using State1 = Eigen::Vector<double, X_DIM>;
  using Control = Eigen::Vector<double, Udim>;
  // using Time = Eigen::Vector<double, 1>;
  using Theta = Eigen::Vector<double, THdim>;
  using MjFunction = std::function<void(const State& x0, const State& x1, const Control& u, const Theta& th)>;

  mj_friction_propagation_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key u_key, gtsam::Key th_key,
                            const gtsam::noiseModel::Base::shared_ptr& cost_model,
                            const std::shared_ptr<system_group_t>& sg, MjFunction mjfn, const Eigen::Index x_dim,
                            const Eigen::Index u_dim, const Eigen::Index p_dim)
    : Base(x0_key, x1_key, u_key, th_key, cost_model, x_dim, x_dim, u_dim, p_dim, x_dim)
    , x1p(State::Zero(x_dim))
    , _mjfn(mjfn)
    , _sg(sg)
  {
  }
  template <Eigen::Index StateDim = Xdim, std::enable_if_t<(StateDim != Eigen::Dynamic), bool> = true>
  mj_friction_propagation_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key u_key, gtsam::Key th_key,
                            const gtsam::noiseModel::Base::shared_ptr& cost_model,
                            const std::shared_ptr<system_group_t>& sg, MjFunction mjfn)
    : mj_friction_propagation_t(x0_key, x1_key, u_key, th_key, cost_model, sg, mjfn, Xdim, Udim, THdim, Xdim)
  {
  }

  virtual State compute_error(const State& x0, const State& x1, const Control& u0, const Theta& theta) const override
  {
    // const space_t* ss = _sg->get_state_space();
    // const space_t* cs = _sg->get_control_space();
    // // const space_t* ps = _sg->get_parameter_space();
    // const std::size_t ss_dim{ ss->get_dimension() };
    // const std::size_t cs_dim{ cs->get_dimension() };
    // const std::size_t ps_dim{ ps->get_dimension() };

    // for (auto xi : x0)
    // {
    //   printf("%.5f ", xi);
    // }
    // printf("\n");
    // PRX_DEBUG_VAR_3(x0.transpose(), u0.transpose(), theta.transpose());
    // PRX_DEBUG_VAR_1(simulation_step);
    // _mjfn(x0, x1, u0, theta);
    // State zero{ 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 1.00000, 0.00000, 0.00000, 0.00000, 0.00000,
    //             0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000 };
    // _sg->propagate(zero, Control::Zero(), simulation_step, x1p);

    _sg->propagate(x0, u0, simulation_step, x1p);
    const State error{ x1 - x1p };
    return error;
  }

private:
  std::shared_ptr<system_group_t> _sg;
  mutable State x1p;
  MjFunction _mjfn;
};

template <Eigen::Index Xdim, Eigen::Index Udim, Eigen::Index THdim, Evals Evaluations = 4>
class mj_friction_estimation_t : public gtsam::NoiseModelFactor1<Eigen::Vector<double, THdim>>
{
protected:
  using Base = gtsam::NoiseModelFactor1<Eigen::Vector<double, THdim>>;

public:
  using Error = Eigen::Vector<double, 1>;
  using State = Eigen::Vector<double, Xdim>;
  using Control = Eigen::Vector<double, Udim>;
  using Theta = Eigen::Vector<double, THdim>;
  using MjFunction = std::function<void(const State& x0, const State& x1, const Control& u, const Theta& th)>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  using PartialFn = std::function<State(const Theta&)>;
  using SystemGroupPtr = std::shared_ptr<system_group_t>;

  template <typename X0, typename X1, typename U0>
  mj_friction_estimation_t(gtsam::Key th_key, const NoiseModel& cost_model, const SystemGroupPtr& sg, MjFunction mjfn,
                           const Eigen::Index x_dim, const Eigen::Index u_dim, const Eigen::Index th_dim, const X0& x0,
                           const X1& x1, const U0& u0, const double h = prx::simulation_step)
    : Base(cost_model, th_key)
    , _x0(State::Zero(x_dim))
    , _x1(State::Zero(x_dim))
    , _x1p(State::Zero(x_dim))
    , _u0(Control::Zero(u_dim))
    , _mjfn(mjfn)
    , _sg(sg)
    , derivative(h, th_dim, 1)
  {
    sg->get_state_space()->copy(_x0, x0);
    sg->get_state_space()->copy(_x1, x1);
    sg->get_control_space()->copy(_u0, u0);
  }
  // template <Eigen::Index StateDim = Xdim, std::enable_if_t<(StateDim != Eigen::Dynamic), bool> = true>
  // mj_friction_estimation_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key u_key, gtsam::Key th_key,
  //                          const gtsam::noiseModel::Base::shared_ptr& cost_model,
  //                          const std::shared_ptr<system_group_t>& sg, MjFunction mjfn)
  //   : mj_friction_estimation_t(x0_key, x1_key, u_key, th_key, cost_model, sg, mjfn, Xdim, Udim, THdim, Xdim)
  // {
  // }

  virtual Eigen::VectorXd evaluateError(const Theta& theta,  // no-lint
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const Error error{ compute_error(theta) };
    if (H0)
    {
      derivative._model = [&](const Theta& theta_p) { return compute_error(theta_p); };
      *H0 = derivative(theta);
    }
    // PRX_DEBUG_VAR_1(error);
    return error;
  }
  virtual Error compute_error(const Theta& theta) const
  {
    // for (auto xi : x0)
    // {
    //   printf("%.5f ", xi);
    // }
    // printf("\n");
    // using namespace std::this_thread;  // sleep_for, sleep_until
    // using namespace std::chrono;
    // PRX_DEBUG_VAR_1(theta);
    // sleep_for(milliseconds(10));
    _mjfn(_x0, _x1, _u0, theta);

    _sg->propagate(_x0, _u0, prx::simulation_step, _x1p);
    const State error{ _x1 - _x1p };

    // PRX_DEBUG_VAR_1("~-~-~-~-~-~-~-~-~-~-~-~-");
    // PRX_DEBUG_VAR_1(_sg->get_parameter_space());
    // PRX_DEBUG_VAR_1(_x0.transpose());
    // PRX_DEBUG_VAR_1(_u0.transpose());
    // PRX_DEBUG_VAR_1(_x1.transpose());
    // PRX_DEBUG_VAR_1(_x1p.transpose());
    // PRX_DEBUG_VAR_2(theta, error.norm());
    return Error{ error.norm() };
  }

private:
  std::shared_ptr<system_group_t> _sg;
  mutable State _x1p;
  MjFunction _mjfn;

  State _x0;
  State _x1;
  Control _u0;
  mutable prx::math::first_order_derivative_t<PartialFn, Theta, Evaluations> derivative;
};
}  // namespace fg
}  // namespace prx
