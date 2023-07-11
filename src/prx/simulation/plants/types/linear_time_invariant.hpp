#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/system_factory.hpp"

#include <memory>

namespace prx
{
class lti_t : public plant_t
{
public:
  lti_t(const lti_t& other) = default;
  lti_t(std::string _path);

  lti_t(const system_ptr_t& _sys_ptr) : plant_t(_sys_ptr)
  {
    auto _plant = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
    prx_assert(_plant != nullptr, "Problem casting to a plant_t");
    plant = _plant;
  }

  virtual ~lti_t();

  /**
   * @brief      Generate the analitical linearization of the system. This is
   *             populate matrices A, B, C & D.
   *
   * @return     True if successful. Some system might not be linearizable or might need to be done numerically.
   */
  virtual bool linearize()
  {
    return plant->linearize(A, B, C, D);
    // return true;
  }

  /**
   * @brief      Generate the analitical linearization of the system around x0. This is
   *             populate matrices A, B, C & D.
   *
   * @return     True if successful.
   */
  // virtual bool linearize(space_point_t x0)
  // {
  // 	prx_throw("No implementation of lti_t::linearize(x0)!");
  // 	return false;
  // }

  /**
   * @brief      Check that dimentions match. Given p inputs,
   *             q outputs and n state variables, matrices
   *             A, B, C & D are respectively: n x n, n x p,
   *             q x n, q x p. In this implementation, q is not
   *             directively known. However, given p & n as well
   *             as matrices C & D q can be determined.
   *
   * @return     { description_of_the_return_value }
   */
  virtual bool check();

  /**
   * @brief      Compute the derivative without computing y(t).
   *             		\dot{x}(t) = Ax(t) + Bu(t)
   *             The derivative (\dot{x}) is saved in the
   *             state space while the ouput returned.
   *
   * @return     The output \dot{x}(t).
   */
  void linear_derivative();

  /**
   * @brief      Compute the derivative and output.
   *             		\dot{x}(t) = Ax(t) + Bu(t)
   *             		y(t) = Cx(t) + Du(t)
   *             The derivative (\dot{x}) is saved in the
   *             state space while the ouput returned.
   *
   * @return     The output y(t).
   */
  Eigen::VectorXd linear_derivative_and_output();

  /**
   * @brief      Zero-Order Hold method to discretize the system
   */
  void discretize();

  Eigen::MatrixXd get_A() const
  {
    return A;
  };
  Eigen::MatrixXd get_B() const
  {
    return B;
  };
  Eigen::MatrixXd get_C() const
  {
    return C;
  };
  Eigen::MatrixXd get_D() const
  {
    return D;
  };

  virtual void update_configuration() override
  {
    plant->update_configuration();
  }

  virtual void compute_derivative() override
  {
    linear_derivative();
  }

protected:
  std::shared_ptr<plant_t> plant;

  Eigen::VectorXd x;
  Eigen::VectorXd u;

  Eigen::VectorXd xd;

  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
};

}  // namespace prx