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
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/range.hpp"
#include "prx/utilities/geometry/regular_grid.hpp"

#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/planning/initialization_trajs_fg.hpp"
#include "prx/factor_graphs/factors/parameter_fusion_factor.hpp"

#include "prx/factor_graphs/factors/euclidian_distance_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/state_prior.hpp"
#include "prx/factor_graphs/graphs/ilqr.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"

#include "prx/mujoco/mj_simulator.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using namespace prx;
using namespace prx::fg;
using namespace prx::utilities;

using graph_values_t = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>;
using Distance = Eigen::Vector<double, 1>;
using Position = Eigen::Vector<double, 3>;

const double rod_length{ 28.0 };
const double center_position{ 6.02 };
const double initial_end_position{ 19.7989898732233 };
// const gtsam::SharedNoiseModel all_constraint_3d = gtsam::noiseModel::Constrained::All(1);
const gtsam::SharedNoiseModel all_constraint_3d = gtsam::noiseModel::Isotropic::Sigma(1, 1);
logger_t rods_logger(prx::out_path + "tensegrity/fg_out_rods.txt");

graph_values_t create_tensegrity_graph()
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const Eigen::Vector3d zero{ Eigen::Vector3d::Ones() };
  // XZ plane
  for (int i = 1; i <= 4; ++i)
  {
    const std::string i_str{ "0" + std::to_string(i) };
    const prx_symbol_t start_rod_pos_i{ symbol_factory_t::create_hashed_symbol("start_rod_pos", i_str) };
    const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i_str) };
    graph.add(euclidian_distance_factor_t<3>(rod_length, start_rod_pos_i, end_rod_pos_i, values));
    // The rods are numbered clockwise, with the frame in the center on the tensegrity
    // The least significant bit is z.
    const int x_sign{ (i > 2) ? -1 : 1 };
    const int z_sign{ i & 0x02 ? -1 : 1 };

    const Position start_position{ x_sign * center_position, 0, z_sign * center_position };
    const Position end_position{ x_sign * initial_end_position, 0, z_sign * initial_end_position };
    graph.addPrior(start_rod_pos_i, start_position);
    values.insert(start_rod_pos_i, start_position);
    values.insert(end_rod_pos_i, end_position);
    // values.insert(end_rod_pos_i, end_position);
    // PRX_DEBUG_VAR_2(start_position.transpose(), i_str);
    PRX_DEBUG_VAR_2(end_position.transpose(), i_str);
    // PRX_DEBUG_VAR_1((start_position).norm());
    // PRX_DEBUG_VAR_1((end_position).norm());
    // PRX_DEBUG_VAR_1((start_position - end_position).norm());
  }
  for (int i = 1; i < 4; i += 2)
  {
    const std::string i_str{ "0" + std::to_string(i) };
    const std::string ip1_str{ "0" + std::to_string(i + 1) };
    const prx_symbol_t cable_length_symbol{ symbol_factory_t::create_hashed_symbol("sensor", i_str, ip1_str) };
    const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i_str) };
    const prx_symbol_t end_rod_pos_ip1{ symbol_factory_t::create_hashed_symbol("end_rod_pos", ip1_str) };
    graph.add(euclidian_distance_factor_t<3>(cable_length_symbol, end_rod_pos_i, end_rod_pos_ip1, all_constraint_3d));
  }

  // YZ plane
  for (int i = 1; i <= 4; ++i)
  {
    const std::string i_str{ "1" + std::to_string(i) };
    const prx_symbol_t start_rod_pos_i{ symbol_factory_t::create_hashed_symbol("start_rod_pos", i_str) };
    const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i_str) };
    graph.add(euclidian_distance_factor_t<3>(rod_length, start_rod_pos_i, end_rod_pos_i, values));
    // The rods are numbered clockwise, with the frame in the center on the tensegrity, which makes a gray code
    // The least significant bit is z.
    const int y_sign{ (i > 2) ? -1 : 1 };
    const int z_sign{ i & 0x02 ? -1 : 1 };

    const Position start_position{ 0, y_sign * center_position, z_sign * center_position };
    const Position end_position{ 0, y_sign * initial_end_position, z_sign * initial_end_position };
    graph.addPrior(start_rod_pos_i, start_position);
    values.insert(start_rod_pos_i, start_position);
    values.insert(end_rod_pos_i, end_position);
    // PRX_DEBUG_VAR_2(start_position.transpose(), i_str);
    PRX_DEBUG_VAR_2(end_position.transpose(), i_str);
  }
  for (int i = 1; i < 4; i += 2)
  {
    const std::string i_str{ "1" + std::to_string(i) };
    const std::string ip1_str{ "1" + std::to_string(i + 1) };
    const prx_symbol_t cable_length_symbol{ symbol_factory_t::create_hashed_symbol("sensor", i_str, ip1_str) };
    const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i_str) };
    const prx_symbol_t end_rod_pos_ip1{ symbol_factory_t::create_hashed_symbol("end_rod_pos", ip1_str) };
    graph.add(euclidian_distance_factor_t<3>(cable_length_symbol, end_rod_pos_i, end_rod_pos_ip1, all_constraint_3d));
  }

  // Z+ plane
  for (const std::string i : { "01", "04" })
  {
    for (const std::string j : { "11", "14" })
    {
      const prx_symbol_t cable_length_symbol{ symbol_factory_t::create_hashed_symbol("sensor", i, j) };
      const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i) };
      const prx_symbol_t end_rod_pos_j{ symbol_factory_t::create_hashed_symbol("end_rod_pos", j) };
      graph.add(euclidian_distance_factor_t<3>(cable_length_symbol, end_rod_pos_i, end_rod_pos_j, all_constraint_3d));
    }
  }

  // Z- plane
  for (const std::string i : { "02", "03" })
  {
    for (const std::string j : { "12", "13" })
    {
      const prx_symbol_t cable_length_symbol{ symbol_factory_t::create_hashed_symbol("sensor", i, j) };
      const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i) };
      const prx_symbol_t end_rod_pos_j{ symbol_factory_t::create_hashed_symbol("end_rod_pos", j) };
      graph.add(euclidian_distance_factor_t<3>(cable_length_symbol, end_rod_pos_i, end_rod_pos_j, all_constraint_3d));
    }
  }
  return { graph, values };
}

void rods_to_file(gtsam::Values& values)
{
  for (const std::string i : { "0", "1" })
  {
    for (const std::string j : { "1", "2", "3", "4" })
    {
      const prx_symbol_t start_rod_pos_i{ symbol_factory_t::create_hashed_symbol("start_rod_pos", i + j) };
      const prx_symbol_t end_rod_pos_i{ symbol_factory_t::create_hashed_symbol("end_rod_pos", i + j) };
      const Position start_position{ values.at<Position>(start_rod_pos_i) };
      const Position end_position{ values.at<Position>(end_rod_pos_i) };
      rods_logger.log(start_position.transpose(), end_position.transpose());
    }
  }
}

void add_sensor_value(gtsam::Values& values, std::string name, double value)
{
  // Find two digits: Expecting "name" to be sensor_dd_dd
  std::regex word_regex("(\\d\\d)");
  std::vector<std::string> matches;

  std::copy(std::sregex_token_iterator(name.begin(), name.end(), word_regex), std::sregex_token_iterator(),
            std::back_inserter(matches));
  prx_assert(matches.size() == 2,
             "Expected matches in sensor name '" << name << "' to be exactly 2. Got " << matches.size());
  const prx_symbol_t cable_length_symbol{ symbol_factory_t::create_hashed_symbol("sensor", matches[0], matches[1]) };
  auto key_str = prx::symbol_factory_t::formatter(cable_length_symbol);
  PRX_DEBUG_VAR_3(name, key_str, value);
  values.insert_or_assign(cable_length_symbol, Distance(value));
}

int main(int argc, char* argv[])
{
  auto params = param_loader("executables/tensegrity/fix_test.yaml", argc, argv);
  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string data_file{ prx::out_path + params["tensegrity_data"].as<>() };
  std::string graph_file{ prx::out_path + "/tensegrity/fix_fg.dot" };
  std::cout << data_file << std::endl;
  prx::utilities::csv_reader_t reader(data_file);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::tie(graph, values) = create_tensegrity_graph();

  while (reader.has_next_line())
  {
    auto block = reader.next_block();
    for (auto line : block)
    {
      const std::string sensor_name{ line[0] };
      const double value{ std::stod(line[1]) };
      add_sensor_value(values, sensor_name, value);
      /* code */
    }
    // auto line = reader.next_line();
    gtsam::LevenbergMarquardtParams lm_params{ prx::fg::utilities::default_levenberg_marquardt_parameters() };
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    values = optimizer.optimize();
    rods_to_file(values);
    // break;
    // std::cout << sensor_name << " " << value << std::endl;
  }
  symbol_factory_t::symbols_to_file();

  // values.print("values", prx::symbol_factory_t::formatter);
  // fg::utilities::print_values<Eigen::Vector3d, Distance>(values, std::cout);

  // fg::utilities::values_to_file<Eigen::Vector3d>(values, prx::out_path + "tensegrity/fg_out_rods.txt");
  // fg::utilities::values_to_file<Distance>(values, prx::out_path + "tensegrity/fg_out_sensors.txt");
  prx::fg::formatter_t graph_formatter;
  graph.saveGraph(graph_file, values, prx::symbol_factory_t::formatter, graph_formatter);
}