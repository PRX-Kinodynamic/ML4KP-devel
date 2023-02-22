#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/utilities/math/math_functions.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

#include "prx/factor_graphs/factors/noise_model_factor.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
class propagation_factor_t : public gtsam::NoiseModelFactor3<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
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
  propagation_factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key xt0_key,
                       gtsam::Key xt1_key,  // gtsam::Key xdt1_key,
                       gtsam::Key ut1_key, system_ptr_t _sys_ptr)
    : Base(cost_model, xt0_key, xt1_key, ut1_key)
  {
    // ltv = std::dynamic_pointer_cast<ltv_t>(_sys_ptr);
    ltv = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
    // ltv = _sys_ptr;
    prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
    xt = ltv->get_state_space()->make_point();
    ut = ltv->get_control_space()->make_point();
    error_pt = ltv->get_state_space()->make_point();
  }

  virtual ~propagation_factor_t()
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
    std::cout << s << "propagation_factor";
    Base::print("", keyFormatter);
  }

private:
  using This = propagation_factor_t;
  using Base = gtsam::NoiseModelFactor3<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
  // std::shared_ptr<ltv_t> ltv;
  std::shared_ptr<plant_t> ltv;

  space_point_t xt;
  space_point_t ut;
  space_point_t error_pt;

  Eigen::VectorXd x_plus;
  Eigen::VectorXd x_minus;
  Eigen::VectorXd xd_plus;
  Eigen::VectorXd xd_minus;

  Eigen::VectorXd u_plus;
  Eigen::VectorXd u_minus;
  Eigen::VectorXd ud_plus;
  Eigen::VectorXd ud_minus;

  space_point_t mem_aux;
};

template <Eigen::Index X_DIM, Eigen::Index U_DIM, Eigen::Index THETA_DIM>
class propagation_factor_5_t
  : public gtsam::NoiseModelFactor5<Eigen::Vector<double, X_DIM>, Eigen::Vector<double, X_DIM>,
                                    Eigen::Vector<double, U_DIM>, Eigen::Vector<double, 1>,
                                    Eigen::Vector<double, THETA_DIM>>
{
  template <typename T, Eigen::Index DIM>
  using partial_fn = std::function<Eigen::Vector<double, DIM>(const T&)>;
  using VALUE1 = Eigen::Vector<double, X_DIM>;
  using VALUE2 = Eigen::Vector<double, X_DIM>;
  using VALUE3 = Eigen::Vector<double, U_DIM>;
  using VALUE4 = Eigen::Vector<double, 1>;
  using VALUE5 = Eigen::Vector<double, THETA_DIM>;
  using Base = gtsam::NoiseModelFactor5<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5>;

public:
  propagation_factor_5_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key u0_key, gtsam::Key time_key,
                         gtsam::Key theta_key, const gtsam::noiseModel::Base::shared_ptr& cost_model,
                         const std::shared_ptr<system_group_t>& sg)
    : Base(cost_model, x0_key, x1_key, u0_key, time_key, theta_key)
    , derivative_x0(partial_x0, 0.01)
    , derivative_x1(partial_x1, 0.01)
    , derivative_u0(partial_u0, 0.01)
    , derivative_t0(partial_t0, 0.01)
    , derivative_theta0(partial_theta0, 0.01)
  {
    // plan.clear();
    // , plan(sg->get_control_space())
    plan = std::make_shared<plan_t>(sg->get_control_space());
    plan->append_onto_back(0.0);
    _sg = sg;
    pt_x0 = sg->get_state_space()->make_point();
    pt_x1 = sg->get_state_space()->make_point();
  }

  virtual ~propagation_factor_5_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const VALUE1& x0, const VALUE2& x1, const VALUE3& u0, const VALUE4& t0,
                                        const VALUE5& theta0, boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H4 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H5 = boost::none) const override
  {
    auto error = compute_error(x0, x1, u0, t0, theta0);
    if (H1)
    {
      derivative_x0.model = [&](const VALUE1& _x0) { return compute_error(_x0, x1, u0, t0, theta0); };
      *H1 = derivative_x0(x0);
    }

    if (H2)
    {
      derivative_x1.model = [&](const VALUE2& _x1) { return compute_error(x0, _x1, u0, t0, theta0); };
      *H2 = derivative_x1(x1);
    }

    if (H3)
    {
      derivative_u0.model = [&](const VALUE3& _u0) { return compute_error(x0, x1, _u0, t0, theta0); };
      *H3 = derivative_u0(u0);
    }

    if (H4)
    {
      derivative_t0.model = [&](const VALUE4& _t0) { return compute_error(x0, x1, u0, _t0, theta0); };
      *H4 = derivative_t0(t0);
    }

    if (H5)
    {
      derivative_theta0.model = [&](const VALUE5& _theta0) { return compute_error(x0, x1, u0, t0, _theta0); };
      *H5 = derivative_theta0(theta0);
    }
    return error;
  }

  Eigen::VectorXd compute_error(const VALUE1& x0, const VALUE2& x1, const VALUE3& u0, const VALUE4& t0,
                                const VALUE5& theta0) const
  {
    const space_t* ss = _sg->get_state_space();
    const space_t* cs = _sg->get_control_space();
    const space_t* ps = _sg->get_parameter_space();
    const std::size_t ss_dim{ ss->get_dimension() };
    const std::size_t cs_dim{ cs->get_dimension() };
    const std::size_t ps_dim{ ps->get_dimension() };

    ss->copy(pt_x0, x0);  // x0
    cs->copy(plan->front().control, u0);
    plan->front().duration = t0[0];
    ps->copy_from(theta0);  // \theta_0

    // Eigen::VectorXd error{ Eigen::VectorXd::Zero(ss_dim) };
    // Eigen::VectorXd dbg{ Eigen::VectorXd::Zero(ss_dim) };

    _sg->propagate(pt_x0, *plan, pt_x1);

    Eigen::VectorXd error{ pt_x1->vector() - x1 };

    // std::cout << "plan: " << plan << std::endl;
    // std::cout << x0_pt << " ==> " << x1_prop_pt << std::endl;
    // std::cout << xt0.transpose() << " ==> " << xt1.transpose() << std::endl;
    // std::cout << "Error: " << error.transpose() << std::endl;

    return error;  // * std::pow(1.1, t);
  }

private:
  partial_fn<VALUE1, X_DIM> partial_x0;
  partial_fn<VALUE2, X_DIM> partial_x1;
  partial_fn<VALUE3, X_DIM> partial_u0;
  partial_fn<VALUE4, X_DIM> partial_t0;
  partial_fn<VALUE5, X_DIM> partial_theta0;

  mutable math::first_order_derivative_t<partial_fn<VALUE1, X_DIM>, VALUE1, 4> derivative_x0;
  mutable math::first_order_derivative_t<partial_fn<VALUE2, X_DIM>, VALUE2, 4> derivative_x1;
  mutable math::first_order_derivative_t<partial_fn<VALUE3, X_DIM>, VALUE3, 4> derivative_u0;
  mutable math::first_order_derivative_t<partial_fn<VALUE4, X_DIM>, VALUE4, 4> derivative_t0;
  mutable math::first_order_derivative_t<partial_fn<VALUE5, X_DIM>, VALUE5, 4> derivative_theta0;

  std::shared_ptr<system_group_t> _sg;
  std::shared_ptr<plan_t> plan;

  space_point_t pt_x0;
  space_point_t pt_x1;
};

// Implements X_{t+1} = X_t + f(X_t, U_t)
template <Eigen::Index X_DIM, Eigen::Index U_DIM>
class propagation_factor_XU_t
  : public gtsam::NoiseModelFactor3<Eigen::Vector<double, X_DIM>, Eigen::Vector<double, X_DIM>,
                                    Eigen::Vector<double, U_DIM>>
{
  // template <typename T, Eigen::Index DIM>
  // using partial_fn = std::function<Eigen::Vector<double, DIM>(const T&)>;
  using X = Eigen::Vector<double, X_DIM>;
  using U = Eigen::Vector<double, U_DIM>;
  using Base = gtsam::NoiseModelFactor3<Eigen::Vector<double, X_DIM>, Eigen::Vector<double, X_DIM>,
                                        Eigen::Vector<double, U_DIM>>;
  // static const int X_DIM = Eigen::Dynamic;
  // static const int U_DIM = Eigen::Dynamic;
  using X_partial_fn = std::function<X(const X&)>;
  using U_partial_fn = std::function<X(const U&)>;

public:
  /**
   * @brief      This implements X_{t+1} = X_t + f(X_t, U_t, \theta, \tau).
   *             Where \theta and \tau (params and duration of propagation) are constant. The plants parameters remain
   *             unchanged, whatever is in the parameter space is used.
   *
   * @param[in]  x0_key      The x_0 key
   * @param[in]  x1_key      The x_1 key
   * @param[in]  u0_key      The u_0 key
   * @param[in]  time_step   The time step
   * @param[in]  cost_model  The cost model
   * @param[in]  sg          System group
   */
  propagation_factor_XU_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key u0_key, const double tau,
                          const gtsam::noiseModel::Base::shared_ptr& cost_model,
                          const std::shared_ptr<system_group_t>& sg)
    : Base(cost_model, x0_key, x1_key, u0_key)
    , derivative_x0(partial_x0, 0.01)
    , derivative_x1(partial_x1, 0.01)
    , derivative_u0(partial_u0, 0.01)
    , _tau(tau)
    , x1_out(X::Zero())
  {
    // plan = std::make_shared<plan_t>(sg->get_control_space());
    // plan->append_onto_back(0.0);
    _sg = sg;
    // pt_x0 = sg->get_state_space()->make_point();
    // pt_x1 = sg->get_state_space()->make_point();
  }

  virtual ~propagation_factor_XU_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X& x0, const X& x1, const U& u0,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none) const override
  {
    const Eigen::VectorXd error{ compute_error(x0, x1, u0) };
    // PRX_DEBUG_VAR_1(error);
    if (H1)
    {
      derivative_x0.model = [&](const X& _x0) { return compute_error(_x0, x1, u0); };
      *H1 = derivative_x0(x0);
    }

    if (H2)
    {
      derivative_x1.model = [&](const X& _x1) { return compute_error(x0, _x1, u0); };
      *H2 = derivative_x1(x1);
    }

    if (H3)
    {
      derivative_u0.model = [&](const U& _u0) { return compute_error(x0, x1, _u0); };
      *H3 = derivative_u0(u0);
    }

    return error;
  }

  Eigen::VectorXd compute_error(const X& x0, const X& x1, const U& u0) const
  {
    // const space_t* ss = _sg->get_state_space();
    // const space_t* cs = _sg->get_control_space();
    // const std::size_t ss_dim{ ss->get_dimension() };
    // const std::size_t cs_dim{ cs->get_dimension() };

    // ss->copy(pt_x0, x0);
    // cs->copy(plan->front().control, u0);
    // plan->front().duration = _time_step;
    // X x1_out;

    _sg->propagate(x0, u0, _tau, x1_out);

    const Eigen::VectorXd error{ x1_out - x1 };

    return error;
  }

private:
  X_partial_fn partial_x0;
  X_partial_fn partial_x1;
  U_partial_fn partial_u0;

  mutable math::first_order_derivative_t<X_partial_fn, X, 4> derivative_x0;
  mutable math::first_order_derivative_t<X_partial_fn, X, 4> derivative_x1;
  mutable math::first_order_derivative_t<U_partial_fn, U, 4> derivative_u0;

  mutable X x1_out;
  std::shared_ptr<system_group_t> _sg;
  // std::shared_ptr<plan_t> plan;
  // space_point_t pt_x0;
  // space_point_t pt_x1;

  const double _tau;
};

// Implements X_{t+1} = X_t + f(X_t, U_t, tau)
template <Eigen::Index X_dim, Eigen::Index U_dim>
class propagation_factor_XUTau_t : public fg::noise_model_4factor_t<X_dim, X_dim, U_dim, 1>
{
  using Base = fg::noise_model_4factor_t<X_dim, X_dim, U_dim, 1>;

  using X = typename Base::X0;
  using U = typename Base::X2;
  using Tau = typename Base::X3;

public:
  /**
   * @brief      This implements X_{t+1} = X_t + f(X_t, U_t, \theta, \tau).
   *             Where \theta and \tau (params and duration of propagation) are constant. The plants parameters remain
   *             unchanged, whatever is in the parameter space is used.
   *
   * @param[in]  x0_key      The x_0 key
   * @param[in]  x1_key      The x_1 key
   * @param[in]  u0_key      The u_0 key
   * @param[in]  time_step   The time step
   * @param[in]  cost_model  The cost model
   * @param[in]  sg          System group
   */
  propagation_factor_XUTau_t(gtsam::Key x0_key, gtsam::Key x1_key, gtsam::Key u0_key, gtsam::Key tau_key,
                             const gtsam::noiseModel::Base::shared_ptr& cost_model,
                             const std::shared_ptr<system_group_t>& sg)
    : Base(x0_key, x1_key, u0_key, tau_key, cost_model)
  {
    _sg = sg;
  }

  virtual X compute_error(const X& x0, const X& x1, const U& u0, const Tau& tau) const override
  {
    if (tau[0] > 0)
    {
      _sg->propagate(x0, u0, tau[0], x1_out);
    }
    else
    {
      x1_out = X::Ones() * 1000;  // Make the error really big
    }

    const X error{ x1_out - x1 };

    return error;
  }

private:
  mutable X x1_out;
  std::shared_ptr<system_group_t> _sg;
};

class propagation_factor_4_t
  : public gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
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
  propagation_factor_4_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key xt0_key,
                         gtsam::Key xt1_key,  // gtsam::Key xdt1_key,
                         gtsam::Key ut1_key, gtsam::Key time_key, system_ptr_t _sys_ptr)
    : Base(cost_model, xt0_key, xt1_key, ut1_key, time_key), plan(_sys_ptr->get_control_space())
  {
    // prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
    xt = _sys_ptr->get_state_space()->make_point();
    ut = _sys_ptr->get_control_space()->make_point();
    error_pt = _sys_ptr->get_state_space()->make_point();
    sys_ptr = _sys_ptr;
  }

  propagation_factor_4_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key xt0_key,
                         gtsam::Key xt1_key,  // gtsam::Key xdt1_key,
                         gtsam::Key ut1_key, gtsam::Key time_key, std::shared_ptr<system_group_t> _sg, int _t = 0)
    : Base(cost_model, xt0_key, xt1_key, ut1_key, time_key), plan(_sg->get_control_space())
  {
    // prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
    sg = _sg;
    xt = sg->get_state_space()->make_point();
    ut = sg->get_control_space()->make_point();
    error_pt = sg->get_state_space()->make_point();
    x1_fg_pt = sg->get_state_space()->make_point();
    x1_prop_pt = sg->get_state_space()->make_point();
    x0_pt = sg->get_state_space()->make_point();
    t = _t;
  }

  virtual ~propagation_factor_4_t()
  {
  }

public:
  virtual Eigen::VectorXd evaluateError(const X1&, const X2&, const X3&, const X4&,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H4 = boost::none) const override;

  Eigen::VectorXd compute_error(Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1, Eigen::VectorXd t) const;

private:
  using This = propagation_factor_t;
  using Base = gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
  // std::shared_ptr<ltv_t> ltv;

  space_point_t xt;
  space_point_t ut;
  space_point_t error_pt;

  int t;
  // Eigen::VectorXd x_prop;
  Eigen::VectorXd x_minus;
  Eigen::VectorXd xd_plus;
  Eigen::VectorXd xd_minus;

  Eigen::VectorXd u_plus;
  Eigen::VectorXd u_minus;
  Eigen::VectorXd ud_plus;
  Eigen::VectorXd ud_minus;

  space_point_t x0_pt;
  space_point_t mem_aux;
  space_point_t x1_fg_pt;
  space_point_t x1_prop_pt;

  system_ptr_t sys_ptr;
  std::shared_ptr<system_group_t> sg;
  mutable plan_t plan;
};

class propagation_witness_factor_t
  : public gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
{
public:
  propagation_witness_factor_t(const gtsam::noiseModel::Base::shared_ptr& _cost_model, gtsam::Key _xt0_key,
                               gtsam::Key _xt1_key, gtsam::Key _ut0_key, gtsam::Key _time_key, Eigen::VectorXd _witness,
                               double _radius, std::shared_ptr<system_group_t> _sg)
    : Base(_cost_model, _xt0_key, _xt1_key, _ut0_key, _time_key)
    , plan(_sg->get_control_space())
    , witnesses(_witness)
    , radius(_radius)
  {
    sg = _sg;
    // plan.copy_onto_back(ut0, time[0]);
    x_aux_pt = _sg->get_state_space()->make_point();
  }

  virtual ~propagation_witness_factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X1&, const X2&, const X3&, const X4&,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H4 = boost::none) const override;

  Eigen::VectorXd compute_error(Eigen::VectorXd x0, Eigen::VectorXd x1, Eigen::VectorXd u0, Eigen::VectorXd t0) const;

private:
  using This = propagation_witness_factor_t;
  using Base = gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;

  double radius;
  std::shared_ptr<system_group_t> sg;
  space_point_t x_aux_pt;
  Eigen::VectorXd witnesses;
  mutable plan_t plan;
};

class propagation_factor_1_t : public gtsam::NoiseModelFactor1<Eigen::VectorXd>
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
  propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr& _cost_model, gtsam::Key _xt0_key,
                         Eigen::VectorXd _xt1, Eigen::VectorXd _ut0, Eigen::VectorXd _time, Eigen::VectorXd _params,
                         std::shared_ptr<system_group_t> _sg)
    : Base(_cost_model, _xt0_key)
    , plan(_sg->get_control_space())
    , xt0(1)
    , xt1(_xt1)
    , ut0(_ut0)
    , time(_time)
    , params(_params)
    , unknown_type(X0)
  {
    sg = _sg;
    plan.copy_onto_back(ut0, time[0]);
    x_aux_pt = _sg->get_state_space()->make_point();
  }

  propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr& _cost_model, Eigen::VectorXd _xt0,
                         gtsam::Key _xt1_key, Eigen::VectorXd _ut0, Eigen::VectorXd _time, Eigen::VectorXd _params,
                         std::shared_ptr<system_group_t> _sg)
    : Base(_cost_model, _xt1_key)
    , plan(_sg->get_control_space())
    , xt0(_xt0)
    , xt1(1)
    , ut0(_ut0)
    , time(_time)
    , params(_params)
    , unknown_type(X1)
  {
    sg = _sg;
    plan.copy_onto_back(ut0, time[0]);
    x_aux_pt = _sg->get_state_space()->make_point();
  }

  propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr& _cost_model, Eigen::VectorXd _xt0,
                         Eigen::VectorXd _xt1, gtsam::Key _ut0_key, Eigen::VectorXd _time, Eigen::VectorXd _params,
                         std::shared_ptr<system_group_t> _sg)
    : Base(_cost_model, _ut0_key)
    , plan(_sg->get_control_space())
    , xt0(_xt0)
    , xt1(_xt1)
    , ut0(1)
    , time(_time)
    , params(_params)
    , unknown_type(U0)
  {
    sg = _sg;
    x_aux_pt = _sg->get_state_space()->make_point();
  }

  propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr& _cost_model, Eigen::VectorXd _xt0,
                         Eigen::VectorXd _xt1, Eigen::VectorXd _ut0, gtsam::Key _time_key, Eigen::VectorXd _params,
                         std::shared_ptr<system_group_t> _sg)
    : Base(_cost_model, _time_key)
    , plan(_sg->get_control_space())
    , xt0(_xt0)
    , xt1(_xt1)
    , ut0(_ut0)
    , time(1)
    , params(_params)
    , unknown_type(TIME)
  {
    sg = _sg;
    x_aux_pt = _sg->get_state_space()->make_point();
  }

  propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr& _cost_model, Eigen::VectorXd _xt0,
                         Eigen::VectorXd _xt1, Eigen::VectorXd _ut0, Eigen::VectorXd _time, gtsam::Key _params_key,
                         std::shared_ptr<system_group_t> _sg)
    : Base(_cost_model, _params_key)
    , plan(_sg->get_control_space())
    , xt0(_xt0)
    , xt1(_xt1)
    , ut0(_ut0)
    , time(_time)
    , params(1)
    , unknown_type(PARAMS)
  {
    sg = _sg;
    plan.copy_onto_back(ut0, time[0]);
    x_aux_pt = _sg->get_state_space()->make_point();
  }

  virtual ~propagation_factor_1_t()
  {
  }

public:
  virtual Eigen::VectorXd evaluateError(const X&, boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override;

  Eigen::VectorXd compute_error_x0(Eigen::VectorXd vec) const;
  Eigen::VectorXd compute_error_x1(Eigen::VectorXd vec) const;
  Eigen::VectorXd compute_error_u0(Eigen::VectorXd vec) const;
  Eigen::VectorXd compute_error_ti(Eigen::VectorXd vec) const;
  Eigen::VectorXd compute_error_pa(Eigen::VectorXd vec) const;

private:
  enum prop_unknown_t
  {
    X0,
    X1,
    U0,
    TIME,
    PARAMS
  };
  using This = propagation_factor_1_t;
  using Base = gtsam::NoiseModelFactor1<Eigen::VectorXd>;
  // std::shared_ptr<ltv_t> ltv;

  space_point_t x_aux_pt;

  prop_unknown_t unknown_type;

  Eigen::VectorXd xt0;
  Eigen::VectorXd xt1;
  Eigen::VectorXd ut0;
  Eigen::VectorXd time;
  Eigen::VectorXd params;
  std::shared_ptr<system_group_t> sg;
  mutable plan_t plan;
};
}  // namespace prx