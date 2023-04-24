#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"

#include <memory>

namespace prx
{
class ltv_t : public lti_t
{
public:
  ltv_t(const ltv_t& other) = default;
  ltv_t(std::string _path);

  ltv_t(const system_ptr_t& _sys_ptr) : lti_t(_sys_ptr)
  {
  }

  virtual ~ltv_t();

  using lti_t::linearize;
  /**
   * @brief      Generate the analitical linearization of the system around xt and ut. This is
   *             populate matrices A, B.
   *
   * @return     True if successful.
   */
  // virtual bool linearize(space_point_t xt, space_point_t ut, double epsilon = 1e-3) override
  virtual bool linearize(space_point_t xt, space_point_t ut, double epsilon = 1e-3)
  {
    // return plant->linearize(A, B, C, D, xt, ut);
    //   return true;
    return linearize_numerical(xt, ut);
  }

  // Eigen::MatrixXd get_A() const {return A;};
  // Eigen::MatrixXd get_B() const {return B;};

protected:
  virtual bool linearize_numerical(space_point_t xt, space_point_t ut, double epsilon = 1e-3);

  virtual void compute_derivative() override
  {
    linear_derivative();
  }

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

}  // namespace prx