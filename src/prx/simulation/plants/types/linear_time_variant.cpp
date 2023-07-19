#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{
ltv_t::ltv_t(std::string _path) : lti_t(_path)
{
}

ltv_t::~ltv_t()
{
}

bool ltv_t::linearize_numerical(space_point_t xt, space_point_t ut, double epsilon)
{
  unsigned ss_dim = get_state_space()->get_dimension();
  unsigned cs_dim = get_control_space()->get_dimension();
  if (x.size() == 0 || u.size() == 0)
  {
    A.resize(ss_dim, ss_dim);
    B.resize(ss_dim, cs_dim);
    x.resize(ss_dim);
    x_plus.resize(ss_dim);
    x_minus.resize(ss_dim);
    xd_plus.resize(ss_dim);
    xd_minus.resize(ss_dim);
    u.resize(cs_dim);
    u_plus.resize(cs_dim);
    u_minus.resize(cs_dim);
    ud_plus.resize(cs_dim);
    ud_minus.resize(cs_dim);
    mem_aux = get_state_space()->make_point();
  }
  get_state_space()->copy_to_point(mem_aux);

  get_state_space()->copy_vector_from_point(x, xt);
  get_control_space()->copy_from_point(ut);

  for (int i = 0; i < ss_dim; i++)
  {
    x_plus = x;   // + simulation_step;
    x_minus = x;  // - simulation_step;
    x_plus(i) += epsilon;
    x_minus(i) -= epsilon;

    // std::cout << "x: " << x.transpose() << std::endl;
    // std::cout << "x_plus: "  << x_plus.transpose() << std::endl;
    // std::cout << "x_minus: " << x_minus.transpose() << std::endl;

    get_state_space()->copy_from_vector(x_plus);
    propagate(epsilon);
    // get_state_space() -> copy_to_vector(xd_plus);
    get_derivative_space()->copy_to_vector(xd_plus);

    get_state_space()->copy_from_vector(x_minus);
    propagate(epsilon);
    // get_state_space() -> copy_to_vector(xd_minus);
    get_derivative_space()->copy_to_vector(xd_minus);

    // std::cout << "xd_plus: "  << xd_plus.transpose() << std::endl;
    // std::cout << "xd_minus: " << xd_minus.transpose() << std::endl;
    // std::cout << "DIFF: " << (( xd_plus - xd_minus) / ( 2. * epsilon ) ).transpose() << std::endl;

    A.col(i) = (xd_plus - xd_minus) / (2. * epsilon);
  }
  get_state_space()->copy_from_vector(x);
  get_control_space()->copy_vector_from_point(u, ut);

  // std::cout << "A: " << A << std::endl;

  for (int i = 0; i < cs_dim; i++)
  {
    u_plus = u;   // + simulation_step;
    u_minus = u;  // - simulation_step;
    u_plus[i] += epsilon;
    u_minus[i] -= epsilon;

    get_control_space()->copy_from_vector(u_plus);
    // compute_derivative();
    propagate(epsilon);
    get_derivative_space()->copy_to_vector(xd_plus);

    get_control_space()->copy_from_vector(u_minus);
    propagate(epsilon);
    get_derivative_space()->copy_to_vector(xd_minus);

    // std::cout << "col: " << ( ud_plus - ud_minus) / ( 2. * simulation_step ) << std::endl;
    // B.col(i) = ( ud_plus - ud_minus) / ( 2. * simulation_step );
    B.col(i) = (xd_plus - xd_minus) / (2. * epsilon);
  }

  get_state_space()->copy_from_point(mem_aux);
  // A = Eigen::MatrixXd::Identity(3,3);
  // std::cout << "B: " << B << std::endl;

  return true;
}

}  // namespace prx