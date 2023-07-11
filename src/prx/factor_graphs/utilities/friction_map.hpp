#pragma once
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>

#include "prx/simulation/playback/utils.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/factors/friction_fusion_factor.hpp"
#include "prx/factor_graphs/factors/mj_friction_propagation.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

namespace prx
{
namespace fg
{

template <Eigen::Index TH_DIM, Eigen::Index BASIS_DIM>
class mj_friction_map_t
{
  using Friction = Eigen::Vector<double, TH_DIM>;
  using Position = Eigen::Vector<double, 2>;
  using BasisPosition = Eigen::Matrix<double, 4, 2>;
  using Weights = Eigen::Vector<double, 4>;
  using LocalBasis = Eigen::Vector<double, 4>;

  using PropagationMj = prx::fg::mj_friction_estimation_t<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic>;
  using State = typename PropagationMj::State;
  using Control = typename PropagationMj::Control;
  using Theta = typename PropagationMj::Theta;
  using MjFunction = typename PropagationMj::MjFunction;
  using PositiveVectorFactor = prx::fg::positive_vector_factor_t<TH_DIM>;
  using BasisFrictionFactor = prx::fg::friction_local_fusion_factor_t<TH_DIM, State, BasisPosition>;

  using SystemGroupPtr = std::shared_ptr<prx::system_group_t>;
  using GraphValuesPair = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>;
  using LocalFusionFactor = prx::fg::friction_local_fusion_factor_t<TH_DIM, State, BasisPosition>;

public:
  template <typename Bounds>
  mj_friction_map_t(SystemGroupPtr sg, const Bounds bounds, const std::size_t grid_divisions,
                    const Friction initial_friction)
    : _sg(sg)
    , _ss_dim(_sg->get_state_space()->get_dimension())
    , _cs_dim(_sg->get_control_space()->get_dimension())
    , _ps_dim(_sg->get_parameter_space()->get_dimension())
    , _frictions_grid(bounds, grid_divisions)
    , _visited_grid(bounds, grid_divisions)
    , _basis_grid(bounds, grid_divisions)
    , _length_0(_frictions_grid.get_cell_length(0))
    , _length_1(_frictions_grid.get_cell_length(1))
    , _initial_friction(initial_friction)
  {
    std::size_t basis_idx{ 0 };
    std::function<prx::prx_symbol_t(const std::vector<double>&)> basis_grid_initializer =
        [&](const std::vector<double>&) {
          const prx::prx_symbol_t basis_symbol{ symbol_factory_t::create_hashed_symbol("basis", basis_idx) };
          basis_idx++;
          return basis_symbol;
        };
    _basis_grid.populate_grid(basis_grid_initializer);
    _frictions_grid.populate_grid(_initial_friction);
    _visited_grid.populate_grid(Friction::Zero());
  }

  gtsam::Values create_factor_graph(prx::trajectory_t& traj, prx::plan_t& plan, const std::size_t idx,
                                    gtsam::NonlinearFactorGraph& graph, gtsam::Values& previous_values)
  {
    gtsam::Values values;
    // gtsam::NonlinearFactorGraph graph;

    std::unordered_map<prx::prx_symbol_t, Position> basis_used;

    std::size_t step_i{ 0 };
    prx::simulation::plan_trajectory_stepper_t plan_trajectory(&plan, &traj);
    for (auto state_ctrl_tuple : plan_trajectory)
    {
      const prx::space_point_t xi_pt = std::get<0>(state_ctrl_tuple);
      const prx::plan_step_t ui_pt = std::get<1>(state_ctrl_tuple);
      const prx::space_point_t xip1_pt = std::get<2>(state_ctrl_tuple);

      const State xi_v{ xi_pt->vector() };
      const Control ui_v{ ui_pt.control->vector() };
      const Eigen::VectorXd ti_v{ (Eigen::VectorXd(1) << ui_pt.duration).finished() };
      const State xip1_v{ xip1_pt->vector() };

      prx_symbol_t state_symbol{ symbol_factory_t::create_hashed_symbol("state_symbol", idx, step_i) };
      prx_symbol_t next_state_symbol{ symbol_factory_t::create_hashed_symbol("state_symbol", idx, step_i + 1) };
      prx_symbol_t control_symbol{ symbol_factory_t::create_hashed_symbol("control_symbol", idx, step_i) };
      prx_symbol_t param_symbol{ symbol_factory_t::create_hashed_symbol("param_symbol", idx, step_i) };
      // prx_symbol_t weights_symbol{ symbol_factory_t::create_hashed_symbol("weight_symbol", idx, step_i) };

      _symbol_positions[param_symbol] = xi_v.head(2);

      _visited_grid(xi_v[0], xi_v[1])[0] = 1;

      graph.add(PositiveVectorFactor(param_symbol, _noise_models["positive_friction"]));
      graph.add(PropagationMj(param_symbol, _noise_models["parameter_space"], _sg, _fg_mjfn, _ss_dim, _cs_dim, _ps_dim,
                              xi_pt, xip1_pt, ui_pt.control, prx::simulation_step));

      Position position_0{ xi_v.head(2) };
      Position position_1{ xi_v.head(2) + Position(0, _length_1) };
      Position position_2{ xi_v.head(2) + Position(_length_0, 0) };
      Position position_3{ xi_v.head(2) + Position(_length_0, _length_1) };

      // PRX_DEBUG_VAR_2(param_symbol, prx::symbol_factory_t::formatter(param_symbol));
      // PRX_DEBUG_VAR_1(position_0.transpose());
      // PRX_DEBUG_VAR_1(position_1.transpose());
      // PRX_DEBUG_VAR_1(position_2.transpose());
      // PRX_DEBUG_VAR_1(position_3.transpose());

      const prx::prx_symbol_t basis_symbol_0{ _basis_grid(position_0[0], position_0[1]) };
      const prx::prx_symbol_t basis_symbol_1{ _basis_grid(position_1[0], position_1[1]) };
      const prx::prx_symbol_t basis_symbol_2{ _basis_grid(position_2[0], position_2[1]) };
      const prx::prx_symbol_t basis_symbol_3{ _basis_grid(position_3[0], position_3[1]) };
      BasisPosition basis_positions;
      basis_positions.row(0) = _frictions_grid.template unmap<Position>(position_0[0], position_0[1]);
      basis_positions.row(1) = _frictions_grid.template unmap<Position>(position_1[0], position_1[1]);
      basis_positions.row(2) = _frictions_grid.template unmap<Position>(position_2[0], position_2[1]);
      basis_positions.row(3) = _frictions_grid.template unmap<Position>(position_3[0], position_3[1]);
      _symbol_positions[basis_symbol_0] = basis_positions.row(0);
      _symbol_positions[basis_symbol_1] = basis_positions.row(1);
      _symbol_positions[basis_symbol_2] = basis_positions.row(2);
      _symbol_positions[basis_symbol_3] = basis_positions.row(3);

      Weights init_weight{ LocalFusionFactor::compute_weight(position_0, _length_0, basis_positions) };
      LocalBasis local_basis;
      std::vector<Position> positions = { position_0, position_1, position_2, position_3 };
      for (int i = 0; i < 4; ++i)
      {
        if (previous_values.exists(_basis_grid(positions[i][0], positions[i][1])))
        {
          local_basis[i] = previous_values.at<Friction>(_basis_grid(positions[i][0], positions[i][1]))[0];
        }
        else
        {
          local_basis[i] = _initial_friction[0];
        }
      }

      auto x = init_weight.dot(local_basis);
      values.insert(param_symbol, (Eigen::VectorXd(1) << x).finished());
      graph.add(BasisFrictionFactor(_noise_models["small_basis"], param_symbol, basis_symbol_0, basis_symbol_1,
                                    basis_symbol_2, basis_symbol_3, xi_v, basis_positions, _length_0, TH_DIM, 1));
      values.insert_or_assign(basis_symbol_0, (Eigen::VectorXd(1) << local_basis[0]).finished());
      values.insert_or_assign(basis_symbol_1, (Eigen::VectorXd(1) << local_basis[1]).finished());
      values.insert_or_assign(basis_symbol_2, (Eigen::VectorXd(1) << local_basis[2]).finished());
      values.insert_or_assign(basis_symbol_3, (Eigen::VectorXd(1) << local_basis[3]).finished());
      basis_used[basis_symbol_0] = basis_positions.row(0);
      basis_used[basis_symbol_1] = basis_positions.row(1);
      basis_used[basis_symbol_2] = basis_positions.row(2);
      basis_used[basis_symbol_3] = basis_positions.row(3);

      step_i++;
    }

    for (auto basis_symbol : basis_used)
    {
      // if (basis_covariances.count(basis_symbol.first) != 0)
      // {
      //   Position p{ basis_symbol.second };
      //   graph.addPrior(basis_symbol.first, resulting_values.at<Friction>(basis_symbol.first),
      //                  basis_covariances[basis_symbol.first]);
      // }
      graph.add(PositiveVectorFactor(basis_symbol.first, _noise_models["positive_friction"]));
    }

    return values;
  }

  void to_files(gtsam::Values& values, const std::string prefix)
  {
    const std::string fm_out_dir = prx::out_path + "friction_maps/" + prefix + "_";
    logger_t log_frictionmap(fm_out_dir + "fmbasis_idd_friction_map.txt");

    _basis_grid.to_file(fm_out_dir + "frictions_grid_mj.txt", [&](const prx::prx_symbol_t& s) {
      if (values.exists(s))
        return values.at<Friction>(s)[0];
      else
        return -1.0;
    });
    _visited_grid.to_file(fm_out_dir + "visited_grid_mj.txt", [&](const Friction& v) { return v.transpose(); });

    auto x_bounds = _visited_grid.bounds(0);
    auto y_bounds = _visited_grid.bounds(1);

    auto X = prx::linspace<double>(x_bounds.first, x_bounds.second, 100);
    auto Y = prx::linspace<double>(y_bounds.first, y_bounds.second, 100);
    for (auto x : X)
    {
      for (auto y : Y)
      {
        if (_visited_grid(x, y)[0] > 0)
        {
          Position position_0(x, y);
          Position position_1(x, y + _length_1);
          Position position_2(x + _length_0, y);
          Position position_3(x + _length_0, y + _length_1);
          BasisPosition basis_positions;
          basis_positions.row(0) = _frictions_grid.template unmap<Eigen::Vector2d>(position_0[0], position_0[1]);
          basis_positions.row(1) = _frictions_grid.template unmap<Eigen::Vector2d>(position_1[0], position_1[1]);
          basis_positions.row(2) = _frictions_grid.template unmap<Eigen::Vector2d>(position_2[0], position_2[1]);
          basis_positions.row(3) = _frictions_grid.template unmap<Eigen::Vector2d>(position_3[0], position_3[1]);
          auto weight = LocalFusionFactor::compute_weight(position_0, _length_0, basis_positions);

          auto basis_0 = values.at<Friction>(_basis_grid(position_0[0], position_0[1]));
          auto basis_1 = values.at<Friction>(_basis_grid(position_1[0], position_1[1]));
          auto basis_2 = values.at<Friction>(_basis_grid(position_2[0], position_2[1]));
          auto basis_3 = values.at<Friction>(_basis_grid(position_3[0], position_3[1]));
          Eigen::Vector4d basis(basis_0[0], basis_1[0], basis_2[0], basis_3[0]);

          auto friction_at_xy = weight.dot(basis);
          log_frictionmap(x, y, friction_at_xy);
        }
        else
        {
          log_frictionmap(x, y, 1.0);
        }
      }
    }
  }
  std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> _noise_models;

  MjFunction _fg_mjfn;

  prx::regular_grid_t<Friction, 2> _frictions_grid;
  prx::regular_grid_t<Friction, 2> _visited_grid;
  prx::regular_grid_t<prx::prx_symbol_t, 2> _basis_grid;

  std::unordered_map<prx::prx_symbol_t, Position> _symbol_positions;

private:
  std::shared_ptr<prx::system_group_t> _sg;
  const std::size_t _ss_dim;
  const std::size_t _cs_dim;
  const std::size_t _ps_dim;
  const double _length_0;
  const double _length_1;
  const Friction _initial_friction;
};
}  // namespace fg
}  // namespace prx