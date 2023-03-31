#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
class lqr_t : public controller_t
{
public:
  // template<class S>
  lqr_t(const system_ptr_t& _sys_ptr, std::string _name) : controller_t(_sys_ptr, _name)
  {
    int n = plant->get_state_space()->get_dimension();
    int m = plant->get_control_space()->get_dimension();
    X.resize(n);
    U.resize(m);
    X_goal.resize(n);
    X_goal = Eigen::VectorXd::Zero(n);
    goal = plant->get_state_space()->make_point();
    ltv = std::make_shared<ltv_t>(plant);
    u = plant->get_control_space()->make_point();
  }

  lqr_t(const system_ptr_t& _sys_ptr, Eigen::MatrixXd _Q, Eigen::MatrixXd _R, std::string _name)
    : lqr_t(_sys_ptr, _name)  //, Q(_Q), R(_R)
                              // : controller_t(_plant, _name)
  {
    set_Q(_Q);
    set_R(_R);
  }

  void set_Q(Eigen::MatrixXd _Q)
  {
    Q = _Q;
  }

  void set_R(Eigen::MatrixXd _R)
  {
    R = _R;
  }

  void set_goal(Eigen::VectorXd _x_goal, Eigen::VectorXd _u_goal)
  {
    ltv->get_control_space()->copy(u, _u_goal);
    set_goal(_x_goal);
    // X_goal = _goal;
  }

  void set_goal(Eigen::VectorXd _x_goal)
  {
    X_goal = _x_goal;
    ltv->get_state_space()->copy(goal, X_goal);
    ltv->linearize(goal, u);
  }

  void set_goal(space_point_t _x_goal, space_point_t _u_goal)
  {
    ltv->get_control_space()->copy(u, _u_goal);
    set_goal(_x_goal);
  }

  void set_goal(const space_point_t& _goal) override
  {
    ltv->get_state_space()->copy(X_goal, _goal);
    ltv->get_state_space()->copy(goal, _goal);
    ltv->linearize(goal, u);
  }

  virtual ~lqr_t();

  using controller_t::compute_controls;
  virtual void compute_controls() override;

  void compute_K();

  void set_K(Eigen::MatrixXd k)
  {
    K = k;
  }
  Eigen::MatrixXd get_K()
  {
    return K;
  }

  std::shared_ptr<ltv_t> get_linearized_plant()
  {
    return ltv;
  }

protected:
  Eigen::MatrixXd K;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;

  Eigen::VectorXd X;
  Eigen::VectorXd U;

  Eigen::VectorXd X_goal;

  std::shared_ptr<ltv_t> ltv;

  space_point_t u;
};
}  // namespace prx
