#include "prx/factor_graphs/defs.hpp"

namespace prx
{
namespace fg
{
template <typename ParamID_fn typename Skip_fn>
static gtsam::NonlinearFactorGraph trajectory_to_graph(ParamID_fn param_id, Skip_fn skip)
{
  for (unsigned xi = 0; xi < traj_real.size(); xi += increment)
  {
    if (xi < traj_real.size() - increment - 1)
    {
      const double x{ traj_real[xi]->vector()[0] };
      const double y{ traj_real[xi]->vector()[1] };
      const double x1{ traj_real[xi + increment]->vector()[0] };
      const double y1{ traj_real[xi + increment]->vector()[1] };

      const prx_symbol_t state_symbol{ symbol_factory_t::create_symbol("state_symbol", i, xi) };
      const prx_symbol_t next_state_symbol{ symbol_factory_t::create_symbol("state_symbol", i, xi + increment) };
      const prx_symbol_t control_symbol{ symbol_factory_t::create_symbol("control_symbol", i, xi) };
      const prx_symbol_t time_symbol{ symbol_factory_t::create_symbol("time_symbol", i, xi) };
      const prx_symbol_t param_symbol{ symbol_factory_t::create_symbol("param_symbol", param_id(x, y)) };
      if (skip(x, y) != skip(x1, y1))
      {
        graph_trajs.addPrior(state_symbol, traj_real[xi]->to_vector(), x_sigma);
        init_vals.insert(state_symbol, traj_real[xi]->to_vector());
        continue;
      }

      Eigen::VectorXd t_vec{ (Eigen::VectorXd(1) << plan[xi].duration * increment).finished() };
      graph_trajs.addPrior(state_symbol, traj_real[xi]->to_vector(), x_sigma);
      graph_trajs.addPrior(control_symbol, plan[xi].control->to_vector(), cs_dm);
      graph_trajs.addPrior(time_symbol, t_vec, t_dm);

      init_vals.insert(state_symbol, traj_real[xi]->to_vector());
      init_vals.insert(control_symbol, plan[xi].control->to_vector());
      init_vals.insert(time_symbol, t_vec);
      if (xi + increment >= traj_real.size() - increment - 1)
      {
        graph_trajs.addPrior(next_state_symbol, traj_real[xi + increment]->to_vector(), x_sigma);
        init_vals.insert(next_state_symbol, traj_real[xi + increment]->to_vector());
      }
      graph_trajs.add(propagation_factor_5_t<3, 4, 1>(state_symbol, next_state_symbol, control_symbol, time_symbol,
                                                      param_symbol, dm, sg));
    }
  }
}
}  // namespace fg
}  // namespace prx