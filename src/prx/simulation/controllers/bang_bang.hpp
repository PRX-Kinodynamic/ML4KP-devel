#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
class bang_bang_t : public controller_t
{
public:
  bang_bang_t(system_ptr_t _plant, std::vector<std::vector<double>> _dimensions_set_points,
              std::string _name = "bang-bang_ctrl")
    : controller_t(_plant, _name), dimensions_set_points(_dimensions_set_points)
  {
    auto cs = _plant->get_control_space();

    auto cs_dim = cs->get_dimension();
    prx_assert(dimensions_set_points.size() == cs_dim,
               "[bang_bang_t]: number of set bounds must equal plant's control dimension");
    aux_ctrl_pt = cs->make_point();
    generate_controls(0);
    ctrl_to_use = 0;
  }

  virtual ~bang_bang_t()
  {
  }

  virtual void compute_controls() override
  {
    _plant->get_control_space()->copy_from(ctrls[ctrl_to_use]);
    _plant->get_control_space()->enforce_bounds();
  }

  void set_control(unsigned int i)
  {
    prx_assert(i < ctrls.size(), "Control number must be less than " << ctrls.size());
    ctrl_to_use = i;
  }

  space_point_t get_control_at(unsigned int i)
  {
    prx_assert(i < ctrls.size(), "Requested control " << i << "but only got " << ctrls.size() << " controls");
    return ctrls[i];
  }

  unsigned int get_num_ctrls()
  {
    return ctrls.size();
  }

protected:
  void generate_controls(int c_i)
  {
    auto cs = _plant->get_control_space();
    if (c_i == cs->get_dimension())
    {
      auto new_ctrl = cs->make_point();
      cs->copy_point(new_ctrl, aux_ctrl_pt);
      ctrls.push_back(new_ctrl);
      PRX_DEBUG_ITERABLE("CTRL", *new_ctrl);
      return;
    }

    for (int i = 0; i < dimensions_set_points[c_i].size(); ++i)
    {
      (*aux_ctrl_pt)[c_i] = dimensions_set_points[c_i][i];
      PRX_DEBUG_ITERABLE("aux_ctrl_pt", *aux_ctrl_pt);
      generate_controls(c_i + 1);
    }
  };
  space_point_t aux_ctrl_pt;
  unsigned int ctrl_to_use;
  std::vector<space_point_t> ctrls;
  std::vector<std::vector<double>> dimensions_set_points;
};
}  // namespace prx