
#include "prx/factor_graphs/factors/propagation_factor.hpp"

namespace prx
{

Eigen::VectorXd propagation_factor_t::compute_error(Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1) const
{
  auto ss = ltv->get_state_space();
  auto cs = ltv->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
  Eigen::VectorXd dbg = Eigen::VectorXd::Zero(ss_dim);

  ss->copy_from(xt0);
  // ss -> enforce_bounds();
  cs->copy_from(ut1);
  // cs -> enforce_bounds();
  // ltv -> compute_control();
  ltv->compute_derivative();

  ltv->propagate(simulation_step);
  ss->copy_to(error_pt);
  ss->copy(xt, xt1);
  ss->copy_to(dbg);

  // ss -> difference(error_pt, error_pt, xt);
  ss->difference(error_pt, xt, error_pt);
  ss->copy(error, error_pt);

  return error;
}

// gtsam::Key xt0_key, gtsam::Key xt1_key, gtsam::Key xdt1_key,
Eigen::VectorXd propagation_factor_t::evaluateError(const X1& xt0, const X2& xt1, const X3& ut1,
                                                    boost::optional<Eigen::MatrixXd&> H1,
                                                    boost::optional<Eigen::MatrixXd&> H2,
                                                    boost::optional<Eigen::MatrixXd&> H3) const
{
  auto error = compute_error(xt0, xt1, ut1);
  // std::cout << "prop error: " << error.transpose() << std::endl;
  if (H1)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_t::compute_error, this, std::placeholders::_1, xt1, ut1);
    *H1 = math_functions::differentiate(fp, xt0);
  }

  if (H2)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_t::compute_error, this, xt0, std::placeholders::_1, ut1);
    *H2 = math_functions::differentiate(fp, xt1);
  }

  if (H3)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_t::compute_error, this, xt0, xt1, std::placeholders::_1);
    *H3 = math_functions::differentiate(fp, ut1);
  }

  // std::cout << "error: " << error.transpose() << std::endl;
  return error;
}

// Eigen::VectorXd propagation_factor_5_t::compute_error(const VALUE1& x0, const VALUE2& x1, const VALUE3& u0,
//                                                       const VALUE4& t0, const VALUE5& theta0) const
// {
//   auto ss = sg->get_state_space();
//   auto cs = sg->get_control_space();
//   auto ss_dim = ss->get_dimension();
//   auto cs_dim = cs->get_dimension();
//   Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
//   Eigen::VectorXd dbg = Eigen::VectorXd::Zero(ss_dim);

//   plan.clear();
//   plan.copy_onto_back(ut1, t01[0]);
//   ss->copy(x0_pt, xt0);

//   sg->propagate(x0_pt, plan, x1_prop_pt);

//   // ss -> copy_vector_from_point(x_prop, x1_prop_pt);
//   auto x_prop = x1_prop_pt->vector<>();
//   error = x_prop - xt1;
//   // ss -> copy_point_from_vector(x1_fg_pt, xt1);
//   // // ss -> copy_vector<>(dbg);

//   // ss -> difference(error_pt, x1_prop_pt, x1_fg_pt);
//   // ss -> difference(error_pt, x1_fg_pt, x1_prop_pt);
//   // ss -> copy_vector_from_point(error, error_pt);
//   // std::cout << "plan: " << plan << std::endl;
//   // std::cout << x0_pt << " ==> " << x1_prop_pt << std::endl;
//   // std::cout << xt0.transpose() << " ==> " << xt1.transpose() << std::endl;
//   // std::cout << "Error: " << error.transpose() << std::endl;

//   return error;  // * std::pow(1.1, t);
// }

// Eigen::VectorXd propagation_factor_5_t::evaluateError(
//     const VALUE1& x0, const VALUE2& x1, const VALUE3& u0, const VALUE4& t0, const VALUE5& theta0,
//     boost::optional<Eigen::MatrixXd&> H1, boost::optional<Eigen::MatrixXd&> H2, boost::optional<Eigen::MatrixXd&> H3,
//     boost::optional<Eigen::MatrixXd&> H4, boost::optional<Eigen::MatrixXd&> H5) const
// {
//   auto error = compute_error(x0, x1, u0, t0, theta0);
//   if (H1)
//   {
//     *H1 = derivative_x0(x0);
//   }

//   if (H2)
//   {
//     *H2 = derivative_x1(x1);
//   }

//   if (H3)
//   {
//     *H3 = derivative_u0(u0);
//   }

//   if (H4)
//   {
//     *H4 = derivative_t0(t0);
//   }

//   if (H5)
//   {
//     *H5 = derivative_theta0(theta0);
//   }

//   // std::cout << "error: " << error.transpose() << std::endl;
//   return error;
// }

Eigen::VectorXd propagation_factor_4_t::compute_error(Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1,
                                                      Eigen::VectorXd t01) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
  Eigen::VectorXd dbg = Eigen::VectorXd::Zero(ss_dim);

  plan.clear();
  plan.copy_onto_back(ut1, t01[0]);
  ss->copy(x0_pt, xt0);

  sg->propagate(x0_pt, plan, x1_prop_pt);

  // ss -> copy_vector_from_point(x_prop, x1_prop_pt);
  auto x_prop = x1_prop_pt->vector<>();
  error = x_prop - xt1;
  // ss -> copy_point_from_vector(x1_fg_pt, xt1);
  // // ss -> copy_vector<>(dbg);

  // ss -> difference(error_pt, x1_prop_pt, x1_fg_pt);
  // ss -> difference(error_pt, x1_fg_pt, x1_prop_pt);
  // ss -> copy_vector_from_point(error, error_pt);
  // std::cout << "plan: " << plan << std::endl;
  // std::cout << x0_pt << " ==> " << x1_prop_pt << std::endl;
  // std::cout << xt0.transpose() << " ==> " << xt1.transpose() << std::endl;
  // std::cout << "Error: " << error.transpose() << std::endl;

  return error;  // * std::pow(1.1, t);
}

Eigen::VectorXd propagation_factor_4_t::evaluateError(const X1& xt0, const X2& xt1, const X3& ut1, const X4& t01,
                                                      boost::optional<Eigen::MatrixXd&> H1,
                                                      boost::optional<Eigen::MatrixXd&> H2,
                                                      boost::optional<Eigen::MatrixXd&> H3,
                                                      boost::optional<Eigen::MatrixXd&> H4) const
{
  auto error = compute_error(xt0, xt1, ut1, t01);
  if (H1)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_4_t::compute_error, this, std::placeholders::_1, xt1, ut1, t01);
    *H1 = math_functions::differentiate(fp, xt0);
  }

  if (H2)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_4_t::compute_error, this, xt0, std::placeholders::_1, ut1, t01);
    *H2 = math_functions::differentiate(fp, xt1);
  }

  if (H3)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_4_t::compute_error, this, xt0, xt1, std::placeholders::_1, t01);
    *H3 = math_functions::differentiate(fp, ut1);
  }

  if (H4)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_factor_4_t::compute_error, this, xt0, xt1, ut1, std::placeholders::_1);
    *H4 = math_functions::differentiate(fp, ut1);
  }

  // std::cout << "error: " << error.transpose() << std::endl;
  return error;
}

Eigen::VectorXd propagation_factor_1_t::compute_error_x0(Eigen::VectorXd vec) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

  ss->copy(x_aux_pt, vec);

  sg->propagate(x_aux_pt, plan, x_aux_pt);

  auto x_prop = x_aux_pt->vector<>();
  error = x_prop - xt1;
  return error;
};

Eigen::VectorXd propagation_factor_1_t::compute_error_x1(Eigen::VectorXd vec) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

  ss->copy(x_aux_pt, xt0);

  sg->propagate(x_aux_pt, plan, x_aux_pt);

  auto x_prop = x_aux_pt->vector<>();
  error = x_prop - vec;
  return error;
};

Eigen::VectorXd propagation_factor_1_t::compute_error_u0(Eigen::VectorXd vec) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

  ss->copy(x_aux_pt, xt0);

  plan.clear();
  plan.copy_onto_back(vec, time[0]);

  sg->propagate(x_aux_pt, plan, x_aux_pt);

  auto x_prop = x_aux_pt->vector<>();
  error = x_prop - xt1;
  return error;
};

Eigen::VectorXd propagation_factor_1_t::compute_error_ti(Eigen::VectorXd vec) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();
  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

  ss->copy(x_aux_pt, xt0);

  plan.clear();
  plan.copy_onto_back(ut0, vec[0]);

  sg->propagate(x_aux_pt, plan, x_aux_pt);

  auto x_prop = x_aux_pt->vector<>();
  error = x_prop - xt1;
  return error;
};

Eigen::VectorXd propagation_factor_1_t::compute_error_pa(Eigen::VectorXd vec) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ps = sg->get_parameter_space();

  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();

  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

  ps->copy_from(vec);
  ss->copy(x_aux_pt, xt0);
  // PRX_DEBUG_PRINT
  // std::cout << "vec: " << vec.transpose();
  // PRX_DEBUG_PRINT
  // std::cout << "ps: " << (*ps);
  // std::cout << std::fixed << std::setprecision(10);
  // std::cout << "plan: " << plan;
  // std::cout << "\n\tx0: " << xt0.transpose();
  sg->propagate(x_aux_pt, plan, x_aux_pt);
  // PRX_DEBUG_PRINT
  // std::cout << "ps: " << (*ps);
  // std::cout << "\n\tx1: " << xt1.transpose();
  // PRX_DEBUG_PRINT

  auto x_prop = x_aux_pt->vector<>();
  error = x_prop - xt1;
  // std::cout << "\n\tx^: " << x_prop.transpose();
  // std::cout << "\terror: " << error.transpose();
  // std::cout << "\n";
  return error;
};

Eigen::VectorXd propagation_factor_1_t::evaluateError(const X& vec, boost::optional<Eigen::MatrixXd&> H1) const
{
  std::function<Eigen::VectorXd(Eigen::VectorXd)> fn_to_use;
  switch (unknown_type)
  {
    case X0:
      fn_to_use = std::bind(&propagation_factor_1_t::compute_error_x0, this, std::placeholders::_1);
      break;
    case X1:
      fn_to_use = std::bind(&propagation_factor_1_t::compute_error_x1, this, std::placeholders::_1);
      break;
    case U0:
      fn_to_use = std::bind(&propagation_factor_1_t::compute_error_u0, this, std::placeholders::_1);
      break;
    case TIME:
      fn_to_use = std::bind(&propagation_factor_1_t::compute_error_ti, this, std::placeholders::_1);
      break;
    case PARAMS:
      fn_to_use = std::bind(&propagation_factor_1_t::compute_error_pa, this, std::placeholders::_1);
      break;
    default:
      prx_throw("Invalid propagation type!");
  }
  auto error = fn_to_use(vec);

  if (H1)
  {
    *H1 = math_functions::differentiate(fn_to_use, vec);
  }

  return error;
}

Eigen::VectorXd propagation_witness_factor_t::compute_error(Eigen::VectorXd x0, Eigen::VectorXd x1, Eigen::VectorXd u0,
                                                            Eigen::VectorXd t0) const
{
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  // auto ps = sg -> get_parameter_space();

  auto ss_dim = ss->get_dimension();
  auto cs_dim = cs->get_dimension();

  Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);

  // ss -> copy_from_vector(x0);
  // cs -> copy_from_vector(u0);
  plan.clear();
  plan.copy_onto_back(u0, t0[0]);
  ss->copy(x_aux_pt, x0);

  sg->propagate(x_aux_pt, plan, x_aux_pt);

  auto x_prop = x_aux_pt->vector<>();
  error = x_prop - x1;
  if (error.norm() < radius)
  {
    error = Eigen::VectorXd::Zero(ss_dim);
  }
  return error;
}

Eigen::VectorXd propagation_witness_factor_t::evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4,
                                                            boost::optional<Eigen::MatrixXd&> H1,
                                                            boost::optional<Eigen::MatrixXd&> H2,
                                                            boost::optional<Eigen::MatrixXd&> H3,
                                                            boost::optional<Eigen::MatrixXd&> H4) const
{
  auto error = compute_error(x1, x2, x3, x4);

  if (H1)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_witness_factor_t::compute_error, this, std::placeholders::_1, x2, x3, x4);
    *H1 = math_functions::differentiate(fp, x1);
  }

  if (H2)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_witness_factor_t::compute_error, this, x1, std::placeholders::_1, x3, x4);
    *H2 = math_functions::differentiate(fp, x2);
  }
  if (H3)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_witness_factor_t::compute_error, this, x1, x2, std::placeholders::_1, x4);
    *H3 = math_functions::differentiate(fp, x3);
  }
  if (H4)
  {
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fp =
        std::bind(&propagation_witness_factor_t::compute_error, this, x1, x2, x3, std::placeholders::_1);
    *H4 = math_functions::differentiate(fp, x4);
  }

  return error;
}

}  // namespace prx