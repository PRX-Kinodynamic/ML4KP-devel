#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/simulation/controllers/custom_controller.hpp"
#include "prx/simulation/general/condition_check.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/plants/types/noisy_plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

#include "prx/visualization/three_js_group.hpp"

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

#include "prx/mujoco/mj_simulator.hpp"
#include "prx/mujoco/plants/mj_friction_plant.hpp"
#include <gtsam/nonlinear/Marginals.h>

#include "friction_maps.hpp"

using namespace prx;

const Eigen::Index TH_DIM{ 1 };

using friction_vector_t = Eigen::Vector<double, TH_DIM>;

int main(int argc, char** argv)
{
  const std::string params_file{ "executables/factor_graphs/mj_collect_friction_gt.yaml" };
  param_loader params(params_file, argc, argv);

  std::string model_filename{ params["model_file"].as<std::string>() };
  const std::vector<double> min_bounds{ params["min_bounds"].as<std::vector<double>>() };
  const std::vector<double> max_bounds{ params["max_bounds"].as<std::vector<double>>() };
  const std::size_t grid_divisions{ params["grid_divisions"].as<std::size_t>() };
  const bool visualize{ params["visualize"].as<bool>() };
  const std::string output_filename{ params["output_filename"].as<>() };

  const std::vector<std::pair<double, double>> env_bounds{ std::make_pair(min_bounds[0], max_bounds[0]),
                                                           std::make_pair(min_bounds[1], max_bounds[1]) };

  prx::regular_grid_t<friction_vector_t, 2> frictions_grid{ env_bounds, grid_divisions };

  std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(model_filename, visualize);
  std::shared_ptr<prx::mujoco_plant_t> mj_plant = std::make_shared<prx::mujoco_plant_t>("mujoco_plant");
  mj_plant->initialize(sim);
  // prx::mujoco::mj_friction_plant_t::add_friction_to_plant(sim, mj_plant);

  sim->init_simulator(mj_plant);

  auto context = sim->get_context("mujoco");
  const auto sg = context.first;
  const auto ss = sg->get_state_space();
  const auto cs = sg->get_control_space();
  const auto ps = sg->get_parameter_space();
  prx_assert(ss != nullptr, "State space is null!!!");
  prx_assert(cs != nullptr, "Control space is null!!!");
  prx_assert(ps != nullptr, "Parameter space is null!!!");
  const std::size_t ss_dim{ ss->get_dimension() };
  const std::size_t cs_dim{ cs->get_dimension() };
  const std::size_t ps_dim{ ps->get_dimension() };

  prx::space_point_t state = ss->make_point();
  prx::space_point_t dummy = ss->make_point();
  ss->copy_to(state);
  friction_vector_t param_vector{ friction_vector_t::Zero() };

  // state->at(0) = 1;
  // state->at(1) = 1;
  // sg->propagate(state, Eigen::Vector3d::Zero(), 0.5, dummy);
  // for (int i = 0; i < sim->_mj_data->ncon; ++i)
  // {
  //   int id_geom1 = sim->_mj_data->contact[i].geom1;
  //   int id_geom2 = sim->_mj_data->contact[i].geom2;
  //   mjtNum* friction = sim->_mj_data->contact[i].friction;
  //   std::string b1 = mj_id2name(sim->_mj_model, mjOBJ_GEOM, id_geom1);
  //   std::string b2 = mj_id2name(sim->_mj_model, mjOBJ_GEOM, id_geom2);
  //   PRX_DEBUG_VAR_2(id_geom1, b1);
  //   PRX_DEBUG_VAR_2(id_geom2, b2);
  //   PRX_DEBUG_VAR_2(friction[0], friction[1]);
  //   PRX_DEBUG_VAR_1(friction[2]);
  //   PRX_DEBUG_VAR_2(friction[3], friction[4]);
  // }
  using Container2D = std::vector<double>;
  std::function<friction_vector_t(const Container2D&)> grid_initializer = [&](const Container2D& xy) {
    state->at(0) = xy[0];
    state->at(1) = xy[1];
    ss->copy_from(state);

    sg->propagate(state, Eigen::Vector3d::Zero(), 0.1, dummy);
    for (int i = 0; i < sim->_mj_data->ncon; ++i)
    {
      int id_geom1 = sim->_mj_data->contact[i].geom1;
      int id_geom2 = sim->_mj_data->contact[i].geom2;
      std::string b1 = mj_id2name(sim->_mj_model, mjOBJ_GEOM, id_geom1);
      std::string b2 = mj_id2name(sim->_mj_model, mjOBJ_GEOM, id_geom2);
      if (b1 == "ball_geom" || b2 == "ball_geom")
      {
        param_vector[0] = sim->_mj_data->contact[i].friction[3];
      }
    }
    return param_vector;
  };

  frictions_grid.populate_grid(grid_initializer);
  frictions_grid.to_file(prx::out_path + "friction_maps/" + output_filename,
                         [&](const friction_vector_t& v) { return v.transpose(); });
  return 0;
}