#pragma once

#include <functional>

#include "prx/simulation/controller.hpp"
#include "prx/simulation/controllers/lqr.hpp"
#include "prx/simulation/plant.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"
namespace prx
{
namespace simulation
{
using namespace std::placeholders;
template <uint8_t Evaluations = 5, int8_t MinDifference = -1,
          typename Delta = math::derivative_input_types<Eigen::VectorXd>>
class lqr_controller_t : public controller_t
{
public:
  using LQR = lqr_t<Eigen::Dynamic, Eigen::Dynamic>;
  using MatrixA = LQR::MatrixA;
  using MatrixB = LQR::MatrixB;

  using MatrixQ = LQR::MatrixQ;
  using MatrixR = LQR::MatrixR;

  using VectorU = LQR::VectorU;
  using VectorX = LQR::VectorX;

  using DynamicFunctionX = std::function<VectorX(const VectorX&)>;
  using DynamicFunctionU = std::function<VectorX(const VectorU&)>;

  using DerivA = prx::math::first_order_derivative_t<DynamicFunctionX, VectorX, Evaluations, MinDifference, Delta>;
  using DerivB = prx::math::first_order_derivative_t<DynamicFunctionU, VectorU, Evaluations, MinDifference, Delta>;

  using Diff = LQR::Diff;
  // inline static Diff DefaultDiff = [](const VectorX& a, const VectorX& b) { return a - b; };
  // using Operations = std::function<VectorX>
  template <typename MatQ, typename MatR, typename VecX, typename VecU>
  lqr_controller_t(system_ptr_t system_ptr, const std::string& path, MatQ q, MatR r, VecX x0, VecU u0,
                   Diff diff = LQR::DefaultDiff, bool normalize_AB = false)
    : controller_t(system_ptr, path)
    , _Xdim(system_ptr->get_state_space()->size())
    , _Udim(system_ptr->get_control_space()->size())
    , _x0(x0)
    , _u0(u0)
    , _dynamic_function_x(std::bind(&lqr_controller_t::dynamics, this, _1, _u0))
    , _dynamic_function_u(std::bind(&lqr_controller_t::dynamics, this, _x0, _1))
    , _A_deriv(_dynamic_function_x, prx::simulation_step, _Xdim, _Xdim)
    , _B_deriv(_dynamic_function_u, prx::simulation_step, _Udim, _Xdim)
    , _lqr(_Xdim, _Udim, diff)
    , _x(VectorX::Zero(_Xdim))
    , _x_ref(x0)
  {
    prx_assert(prx::simulation_step > 0, "simulation step not set!");
    prx_assert(system_ptr->get_system_type() == plant_type::ANALYTICAL,
               "lqr_controller_t only supports plant_type::ANALYTICAL plants");
    _lqr.Q() = q;
    _lqr.R() = r;
    _lqr.A() = _A_deriv(x0);
    _lqr.B() = _B_deriv(u0);

    if (normalize_AB)
      normalize();
    _lqr.compute_K();
  }

  template <typename MatK, typename VecX>
  lqr_controller_t(system_ptr_t system_ptr, const std::string& path, const MatK k, const VecX xref,
                   Diff diff = LQR::DefaultDiff)
    : controller_t(system_ptr, path)
    , _Xdim(system_ptr->get_state_space()->size())
    , _Udim(system_ptr->get_control_space()->size())
    , _x0(VectorX::Zero(_Xdim))
    , _u0(VectorU::Zero(_Udim))
    , _lqr(k)
    , _x(VectorX::Zero(_Xdim))
    , _x_ref(xref)
    , _dynamic_function_x(std::bind(&lqr_controller_t::dynamics, this, _1, _u0))
    , _dynamic_function_u(std::bind(&lqr_controller_t::dynamics, this, _x0, _1))
    , _A_deriv(_dynamic_function_x, prx::simulation_step, _Xdim, _Xdim)
    , _B_deriv(_dynamic_function_u, prx::simulation_step, _Udim, _Xdim)
  {
  }

  virtual ~lqr_controller_t()
  {
  }

  virtual void compute_control()
  {
    _plant->get_state_space()->copy_to(_x);
    const VectorU ctrl{ _lqr(_x, _x_ref) };
    _plant->get_control_space()->copy_from(ctrl);
  }

  virtual void compute_controls()
  {
    compute_control();
  }

  void normalize()
  {
    const std::size_t xdim{ _plant->get_state_space()->size() };
    const std::size_t udim{ _plant->get_control_space()->size() };

    Eigen::VectorXd Tx_diag{ Eigen::VectorXd::Zero(xdim) };
    Eigen::VectorXd Tu_diag{ Eigen::VectorXd::Zero(udim) };

    MatrixA Tx{ MatrixA::Zero(xdim, xdim) }, Tx_inv{ MatrixA::Zero(xdim, xdim) };
    MatrixB Tu{ MatrixB::Zero(udim, udim) }, Tu_inv{ MatrixB::Zero(udim, udim) };

    auto ss_ub = _plant->get_state_space()->get_upper_bounds();
    auto ss_lb = _plant->get_state_space()->get_lower_bounds();
    auto cs_ub = _plant->get_control_space()->get_upper_bounds();
    auto cs_lb = _plant->get_control_space()->get_lower_bounds();

    for (int i = 0; i < xdim; ++i)
    {
      const double abs_max{ std::max(std::fabs(ss_ub[i]), std::fabs(ss_lb[i])) };
      PRX_DBG_VARS(abs_max);
      Tx_diag[i] = abs_max;
      // PRX_DBG_VARS(Tx_inv.diagonal());
    }
    for (int i = 0; i < udim; ++i)
    {
      const double abs_max{ std::max(std::fabs(cs_ub[i]), std::fabs(cs_lb[i])) };
      PRX_DBG_VARS(abs_max);
      Tu_diag[i] = abs_max;
      // PRX_DBG_VARS(Tu.diagonal());
      // PRX_DBG_VARS(Tu_inv.diagonal());
    }

    Tx.diagonal() = Tx_diag;
    Tu.diagonal() = Tu_diag;
    Tx_inv.diagonal() = (1.0 / Tx_diag.array()).matrix();
    Tu_inv.diagonal() = (1.0 / Tu_diag.array()).matrix();
    _lqr.A() = Tx_inv * _lqr.A() * Tx;
    _lqr.B() = Tx_inv * _lqr.B() * Tu;
  }

  // get the underling LQR
  inline LQR& lqr()
  {
    return _lqr;
  }

protected:
  // \dot{x_t} = dynamics(x_t, u_t) --> x_{t+1} = x_t + \dot{x_t}dt
  VectorX dynamics(const VectorX& x_in, const VectorU& u_in)
  {
    _plant->get_state_space()->copy_from(x_in);
    _plant->get_control_space()->copy_from(u_in);
    _plant->propagate(prx::simulation_step * prx::simulation_step);
    _plant->get_derivative_space()->copy_to(_x);
    return _x;
  };

  const Eigen::Index _Xdim;
  const Eigen::Index _Udim;

  const VectorX _x0;
  const VectorU _u0;

  DynamicFunctionX _dynamic_function_x;
  DynamicFunctionU _dynamic_function_u;

  DerivA _A_deriv;
  DerivB _B_deriv;

  LQR _lqr;

  VectorX _x;
  VectorX _x_ref;
};
}  // namespace simulation
}  // namespace prx
