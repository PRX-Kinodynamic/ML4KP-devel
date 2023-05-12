#pragma once
#include "prx/utilities/defs.hpp"

using WorldFunction = std::function<void()>;
using namespace prx;

namespace prx
{
namespace friction_map
{
struct world_functions_t
{
  std::shared_ptr<system_group_t> _sg;
  space_t* ss;
  space_t* ps;
  world_functions_t(std::shared_ptr<system_group_t> sg) : _sg(sg)
  {
    ss = _sg->get_state_space();
    ps = _sg->get_parameter_space();
  }
  template <typename FrictionAt, typename... FrictionArgs>
  WorldFunction get_real_world(FrictionAt& friction_at, FrictionArgs... fargs)
  {
    std::function<void()> real_world = [&]()  // no-lint
    {
      const double x{ ss->at(0) };
      const double y{ ss->at(1) };
      auto friction_params = friction_at(x, y, fargs...);
      ps->copy_from(friction_params);
    };
    return real_world;
  }
};

namespace friction_maps
{
using FrictionVector = Eigen::Vector<double, 1>;

FrictionVector friction_map1_at(const double x, const double y)
{
  double friction = 1;
  const double max_friction{ 2 };
  if (y < 1.5)
  {
    friction = max_friction * y / 1.5;
  }
  else if (y < 1.7)
  {
    friction = max_friction;
  }
  else
  {
    friction = max_friction * (3 - y) / 1.3;
  }
  return FrictionVector(friction);
}

FrictionVector friction_map2_at(const double x, const double y)
{
  double friction = 1;
  const double max_friction{ 2 };
  if (y < 1.5)
  {
    friction = max_friction - max_friction * y / 1.5;
  }
  else if (y < 1.7)
  {
    friction = max_friction - max_friction;
  }
  else
  {
    friction = max_friction - max_friction * (3 - y) / 1.3;
  }
  return FrictionVector(friction);
}
FrictionVector friction_map3_at(const double x, const double y)
{
  return FrictionVector(1);
}

struct friction_maps_t
{
  // Asuming lower limit of all is (0,0)
  static inline std::unordered_map<int, double> x_max_map{ // no-lint
                                                           { 1, 10.0 },
                                                           { 2, 10.0 }
  };
  static inline std::unordered_map<int, double> y_max_map{ // no-lint
                                                           { 1, 10.0 },
                                                           { 2, 10.0 }
  };
  // static inline std::unordered_map<int, WorldFunction> world_functions_map{
  //   { 1, friction_maps::friction_map1_at },
  //   { 2, friction_maps::friction_map2_at }
  //   // no-lint
  // };
};

template <typename F, typename... Fargs>
void friction_map_to_file(const int map_id, const double stepping, F& f, Fargs... fargs)
{
  const std::string fm_out_dir = prx::out_path + "friction_maps/";
  logger_t logger(fm_out_dir + "friction_map_" + std::to_string(map_id) + ".txt", ' ');
  const double x_max{ friction_maps_t::x_max_map[map_id] };
  const double y_max{ friction_maps_t::y_max_map[map_id] };
  for (double x = 0; x < x_max; x += stepping)
  {
    for (double y = 0; y < y_max; y += stepping)
    {
      const FrictionVector friction_params{ f(x, y, fargs...) };
      logger.log(x, y, friction_params.transpose());
    }
  }
}
}  // namespace friction_maps

template <typename Plant>
void mecanum_omnibot_follow_path(Plant& plant, std::shared_ptr<system_group_t> sg,
                                 const trajectory_t& desired_trajectory, trajectory_t& resulting_trajectory,
                                 plan_t& resulting_plan)
{
  const double l_a = .11;
  const double l_b = .10;
  const double l_ab = l_a + l_b;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  Eigen::Matrix<double, 4, 3> inverse;
  inverse << -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0,  // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,         // no-lint
      -1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), -1.0 / 4.0,        // no-lint
      1.0 / (4.0 * l_ab), 1.0 / (4.0 * l_ab), 1.0 / 4.0;          // no-lint
  double curr_freq{ 0.0 };
  Eigen::Vector3d current_state_vec;
  std::shared_ptr<custom_controller_t> omnibot_controller =
      std::make_shared<custom_controller_t>(plant, "omnibot_controller");

  double goal_region_radius{ 0.01 };
  Eigen::Vector4d U;
  bool controller_reached_goal{ true };
  omnibot_controller->custom_control_function = [&](const space_point_t& goal, const space_point_t& control) {
    ss->copy_to(current_state_vec);
    const Eigen::Vector3d xd{ goal->vector() - current_state_vec };
    // const Eigen::Vector3d xd{ current_state_vec - goal->vector() };
    // PRX_DEBUG_VAR_1(current_state_vec.transpose());
    // PRX_DEBUG_VAR_1(xd.transpose());
    // PRX_DEBUG_VAR_2(curr_freq, xd.norm());
    if (xd.norm() < goal_region_radius)
    {
      U = Eigen::Vector4d::Zero();
      curr_freq = 0.0;
      controller_reached_goal = true;
    }
    else if (curr_freq <= 0.0)
    {
      U = (inverse * xd).normalized() * 128;
      curr_freq = 0.12;
    }
    cs->copy(control, U);
    curr_freq -= simulation_step;
  };
  space_point_t goal = ss->make_point();
  condition_check_t checker("time", 1);

  // auto cc = create_default_goal_check(ss, goal, goal_region_radius);
  custom_check_t goal_reached = [&]() { return controller_reached_goal; };
  condition_check_t check_goal_reached(goal_reached);
  checker.add_condition(&check_goal_reached);

  space_point_t start_state = ss->make_point();
  space_point_t end_state = ss->make_point();
  trajectory_t local_traj(ss);

  start_state = desired_trajectory[0];
  for (int i = 0; i < desired_trajectory.size() - 1; ++i)
  {
    goal = desired_trajectory[i + 1];
    omnibot_controller->set_goal(goal);
    checker.reset();
    sg->propagate(start_state, omnibot_controller, checker, local_traj);
    resulting_trajectory += local_traj;
    resulting_plan += *(omnibot_controller->get_plan());
    omnibot_controller->get_plan()->clear();
    controller_reached_goal = false;
    start_state = resulting_trajectory.back();
  }
}

void init_logmap(std::unordered_map<std::string, logger_t>& logs, const std::string& prefix = "")
{
  const std::string fm_out_dir = prx::out_path + "friction_maps/" + prefix;

  const std::vector<std::pair<std::string, std::string>> filenames = {
    { "traj_real_file", "fmbasis_trajs_real.txt" },
    { "plan_real_file", "fmbasis_plans_real.txt" },
    { "traj_fg_file", "fmbasis_trajs_fg.txt" },
    { "fg_graph_file", "fmbasis_factor_graph.dot" },
    { "thx_file", "fmbasis_factor_graph_thx.txt" },
    { "idd_friction_map", "fmbasis_idd_friction_map.txt" },
    { "goals", "fmbasis_goals.txt" },
    { "gt_friction_map", "fmbasis_gt_friction_map.txt" },
    { "fg_log", "fmbasis_fg.log" },
    { "error_grid", "fmbasis_error_grid.log" },
    { "thetas_error_log", "thetas_error.log" },
    { "interpolated_error_log", "interpolated_error.log" },
    { "visited", "visited_cells.log" }
  };
  for (auto& str_pair : filenames)
  {
    const std::string path{ fm_out_dir + str_pair.second };
    std::remove(path.c_str());
    logs.emplace(std::make_pair(str_pair.first, path));
  }
}

// Given a grid (nxm), put it into a vector (1 x (n*m))
template <typename BasisVector, typename ThetasGrid>
BasisVector basis_vector_from_grid(ThetasGrid& thetas_grid)
{
  BasisVector bv{ BasisVector::Zero() };
  std::size_t i = 0;
  for (auto pair_ : thetas_grid)
  {
    bv[i] = pair_.second[0];
    i++;
    // th[i] = values.at<theta_t>(keys_[i])[0];  // Assuming THETA_DIM==1 for now.
  }
  return bv;
}

template <typename BasisVector, typename ThetasGrid>
void basis_vector_to_grid(const BasisVector vector, ThetasGrid& thetas_grid)
{
  std::size_t i{ 0 };
  for (auto pair_ : thetas_grid)
  {
    pair_.second[0] = vector[i];
    i++;
  }
}

template <typename ThetaPose, typename BasisPose>
double compute_weight(const ThetaPose& theta_t_pos, const BasisPose& theta_i_pos, const double& length)
{
  const double Bx{ theta_i_pos[0] };
  const double By{ theta_i_pos[1] };

  const double Xx{ theta_t_pos[0] };
  const double Xy{ theta_t_pos[1] };

  const double delta_x{ std::fabs(Bx - Xx) };
  const double delta_y{ std::fabs(By - Xy) };

  const double D{ std::sqrt(length * length + length * length) };
  // PRX_DEBUG_VAR_2(delta_x, delta_y);
  // PRX_DEBUG_VAR_2(length, D);
  return std::max(1.0 - ((delta_x + delta_y) / D), 0.0);
}

template <typename BasisVector, typename ThetaPosGrid, typename StateVector>
BasisVector weights_vector_given_state(ThetaPosGrid& pos_grid, const StateVector& theta_i_pos)
{
  BasisVector weights{ BasisVector::Zero() };
  const double cell_length{ pos_grid.get_cell_length(0) };
  std::size_t i = 0;
  for (auto pair_ : pos_grid)
  {
    Eigen::Vector2d position{ pos_grid.template unmap_key<Eigen::Vector2d>(pair_.first) };
    // const StateVector pos_theta{};
    weights[i] = compute_weight(position, theta_i_pos, cell_length);
    i++;
  }
  // PRX_DEBUG_VAR_1(weights.transpose());
  weights = weights / weights.sum();
  // PRX_DEBUG_VAR_1(weights.transpose());
  return weights;
}

// Useful to iterate over plans and trajectories:
// Iter is (x_i, u_i, x_{i+1})
struct plan_trajectory_t
{
  // ToDo: Think of a better name for "step"...
  using Step = std::tuple<prx::space_point_t, prx::plan_step_t, prx::space_point_t>;

  plan_trajectory_t(plan_t* plan, trajectory_t* traj) : _plan(plan), _trajectory(traj)
  {
    // PRX_DEBUG_VAR_2(plan->size(), traj->size());
    prx_assert(_plan->size() + 1 == _trajectory->size(), "Mismatch in sizes");
    // prx_assert(_plan->size() >= _trajectory->size(), "Mismatch in sizes");
  }

  class iterator
  {
    using iterator_category = std::output_iterator_tag;
    using value_type = Step;  // crap
    using difference_type = Step;
    using pointer = const Step*;
    using reference = Step;

    prx::trajectory_t::iterator _xi;    // x_i
    prx::plan_t::iterator _ui;          // u_i
    prx::trajectory_t::iterator _xip1;  // x_{i+1}

  public:
    explicit iterator(prx::trajectory_t::iterator xi, prx::plan_t::iterator ui, prx::trajectory_t::iterator xip1)
      : _xi(xi), _ui(ui), _xip1(xip1)
    {
    }
    explicit iterator(plan_t* plan, trajectory_t* traj)
      : _xi((*traj).begin()), _ui((*plan).begin()), _xip1((*traj).begin() + 1)
    {
    }

    iterator& operator++()
    {
      _xi = _xip1;
      _ui++;
      _xip1++;
      return *this;
    }
    iterator operator++(int)
    {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const
    {
      return _xip1 == other._xip1;
    }
    bool operator!=(iterator other) const
    {
      return !(*this == other);
    }
    reference operator*() const
    {
      prx::space_point_t xi_pt = *_xi;
      prx::plan_step_t ui_pt = *_ui;
      prx::space_point_t xip1_pt = *_xip1;
      return std::make_tuple(xi_pt, ui_pt, xip1_pt);
    }
  };

  iterator begin()
  {
    return iterator(_plan, _trajectory);
  }
  iterator end()
  {
    return iterator((*_trajectory).end() - 1, (*_plan).end(), (*_trajectory).end());
  }

  plan_t* _plan;
  trajectory_t* _trajectory;
};

// template <typename BasisVector, typename State, typename ThetaPosGrid>
// friction_vector_t friction_at(const State& state, const basis_vector_t& thetas, ThetaPosGrid& pos_grid)
// {
//   const double x{ state[0] };
//   const double y{ state[1] };
//   const BasisVector weights{ weights_vector_given_state(pos_grid, state) };

//   // const basis_vector_t weights{ compute_weights_vector(pos_grid, state_t(x, y, 0)) };
//   const friction_vector_t friction_vector{ weights.adjoint() * thetas };

//   return friction_vector;
// }

template <typename BasisVector, typename ThetasGrid>
void compute_friction_map(ThetasGrid& grid, logger_t& logger, const std::vector<double>& x_linspace,
                          const std::vector<double>& y_linspace)
{
  for (auto x : x_linspace)
  {
    for (auto y : y_linspace)
    {
      const BasisVector weights = weights_vector_given_state<BasisVector>(grid, Eigen::Vector2d(x, y));
      const BasisVector basis = basis_vector_from_grid<BasisVector>(grid);

      auto friction_vector{ weights.adjoint() * basis };
      logger.log(x, y, friction_vector.transpose());
    }
  }
}

template <typename BasisVector, typename ThetasGrid, typename GuardGrid>
void compute_friction_map(ThetasGrid& grid, GuardGrid& guard_grid, logger_t& logger,
                          const std::vector<double>& x_linspace, const std::vector<double>& y_linspace)
{
  const BasisVector basis = basis_vector_from_grid<BasisVector>(grid);
  const BasisVector guard = basis_vector_from_grid<BasisVector>(guard_grid);
  // const BasisVector guarded{ basis.array() * guard.array() };
  for (auto x : x_linspace)
  {
    for (auto y : y_linspace)
    {
      const BasisVector weights = weights_vector_given_state<BasisVector>(grid, Eigen::Vector2d(x, y));
      // auto friction_vector{ weights.adjoint() * guarded };
      const BasisVector guarded_weight{ weights.array() * guard.array() };
      auto friction_vector{ guarded_weight.dot(basis) };
      logger.log(x, y, friction_vector);
    }
  }
}

template <typename ThetaFrictionGrid, typename BasisVector>
void update_friction_grid(ThetaFrictionGrid& tf_grid, BasisVector new_frictions)
{
  std::size_t i{ 0 };
  for (auto pair_ : tf_grid)
  {
    tf_grid[pair_.first][0] = new_frictions[i];
    i++;
  }
}
}  // namespace friction_map
}  // namespace prx