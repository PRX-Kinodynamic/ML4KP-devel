#pragma once

#include "prx/simulation/system.hpp"

#include "prx/utilities/geometry/movable_object.hpp"

#include <bitset>

#include "prx/simulation/integrators/integrator.hpp"
#include "prx/simulation/integrators/euler.hpp"
#include "prx/simulation/integrators/dopri5.hpp"
#include "prx/simulation/integrators/runge_kutta4.hpp"

namespace prx
{
class system_factory_t;

class plant_t : public system_t, public movable_object_t
{
public:
  plant_t(const plant_t& _plant) = default;
  plant_t(const std::string& path);
  virtual ~plant_t();

  virtual void add_system(system_ptr_t&) override final;

  virtual void propagate(const double simulation_step) override;

  virtual void compute_stopping_maneuver(space_point_t, std::vector<double>&, std::vector<double>&) override;

  virtual void compute_control() override;

  virtual void update_configuration() = 0;

  const std::vector<std::pair<unsigned, unsigned>>& get_collision_list()
  {
    return collision_list;
  }

  void set_integrator(integrator_t::integrators integrator);

  virtual void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) override;

  virtual inline space_t* get_derivative_space()
  {
    return derivative_space;
  }

  virtual bool linearize(Eigen::MatrixXd& A, Eigen::MatrixXd& B)
  {
    const prx::math::S s{ 5 };
    const prx::math::I_min i_min{ -2 };
    using Model = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using Derivative = prx::math::first_order_derivative_t<Model, Eigen::VectorXd, s, i_min>;

    const prx::space_t* ss = get_state_space();
    const prx::space_t* cs = get_control_space();
    const prx::space_t* ds = get_derivative_space();

    const std::size_t ss_dim{ ss->get_dimension() };
    const std::size_t cs_dim{ cs->get_dimension() };
    const std::size_t ds_dim{ ds->get_dimension() };
    const double h{ prx::simulation_step * prx::simulation_step };

    Model ss_model = [&](const Eigen::VectorXd& v_in) {
      get_state_space()->copy_from(v_in);
      // propagate(h);
      compute_derivative();

      Eigen::VectorXd out(Eigen::VectorXd::Zero(ds_dim));
      get_derivative_space()->copy_to(out);
      return out;
    };
    Model cs_model = [&](const Eigen::VectorXd& v_in) {
      get_control_space()->copy_from(v_in);
      // propagate(h);
      compute_derivative();

      Eigen::VectorXd out(Eigen::VectorXd::Zero(ds_dim));
      get_derivative_space()->copy_to(out);
      return out;
    };
    Derivative ss_derivative(ss_model, h, ss_dim, ss_dim);
    Derivative cs_derivative(cs_model, h, cs_dim, ss_dim);

    Eigen::VectorXd ss_in{ Eigen::VectorXd::Zero(ss_dim) };
    Eigen::VectorXd cs_in{ Eigen::VectorXd::Zero(cs_dim) };

    ss->copy_to(ss_in);
    cs->copy_to(cs_in);

    A = ss_derivative(ss_in);
    B = cs_derivative(cs_in);
    return true;
  }

  virtual bool linearize(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D,
                         space_point_t xt, space_point_t ut)
  {
    return linearize(A, B);
  };

  space_t* derivative_space;

  static int registred_plants;

  virtual void compute_derivative() = 0;

protected:
  plant_t(const system_ptr_t other) : system_t(other), movable_object_t(other->get_pathname())
  {
    auto _plant = std::dynamic_pointer_cast<plant_t>(other);
    prx_assert(_plant != nullptr, "Problem casting to a plant_t");
    derivative_space = _plant->derivative_space;
    // derivative_memory = other -> derivative_memory;
    integrator = _plant->integrator;
    derivative_state = _plant->derivative_state;

    for (int i = 0; i < derivative_space->get_dimension(); ++i)
    {
      derivative_memory.push_back(_plant->derivative_memory[i]);
    }
  }

  std::vector<double*> derivative_memory;

  // bodies that we want to check collisions for
  std::vector<std::pair<unsigned, unsigned>> collision_list;

  std::shared_ptr<integrator_t> integrator;

private:
  space_point_t derivative_state;
  friend system_factory_t;
};

}  // namespace prx
