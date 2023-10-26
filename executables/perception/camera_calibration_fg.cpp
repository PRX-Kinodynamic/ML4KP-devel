#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/logger.hpp"

#include <chrono>
#include <thread>

#include <gtsam/nonlinear/Marginals.h>

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/factors/aruco_marker.hpp"
#include "prx/factor_graphs/factors/camera_calibration_factor.hpp"
#include "prx/factor_graphs/factors/euclidian_distance_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
using prx::param_loader;
using prx::prx_symbol_t;
using prx::symbol_factory_t;
using prx::fg::camera_feature_factor_t;
using prx::fg::factor_graph_logger_t;
using prx::fg::positive_vector_factor_t;
using prx::utilities::csv_reader_t;

using position_distance_factor = prx::fg::euclidian_distance_factor_t<3>;
using Pixel = prx::fg::camera_feature_factor_t::Pixel;
using Rotation = prx::fg::camera_feature_factor_t::Rotation;
using Position = prx::fg::camera_feature_factor_t::Position;
using Translation = prx::fg::camera_feature_factor_t::Translation;
using CameraWithDistortion = prx::fg::camera_feature_factor_t::CameraWithDistortion;

using Line = std::vector<std::string>;
constexpr Eigen::Index CameraWithDistortionDim{ CameraWithDistortion::RowsAtCompileTime };

std::unordered_map<std::string, int> cameras_ids;
int total_cameras{ 0 };
double marker_length;
double marker_diagonal;
std::size_t distances{ 0 };
CameraWithDistortion camera_initial_value{ CameraWithDistortion::Ones() };
Rotation rotation_initial_value{ Rotation::Zero() };
Position position_initial_value{ Position::Ones() };
Translation translation_initial_value{ Translation::Ones() };

std::unordered_map<std::string, gtsam::SharedNoiseModel> noise_models;
std::unordered_map<int, prx_symbol_t> marker_ids;
std::unordered_map<prx_symbol_t, CameraWithDistortion> camera_values;
std::unordered_map<prx_symbol_t, Rotation> rotation_values;
std::unordered_map<prx_symbol_t, Position> position_values;
std::unordered_map<prx_symbol_t, Translation> translation_values;

std::unordered_map<prx_symbol_t, std::vector<prx_symbol_t>> camera_marker_map;
void init()
{
  noise_models["distance_between_markers"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-1);
  noise_models["feature"] = gtsam::noiseModel::Isotropic::Sigma(2, 1e0);
  noise_models["markers_pixels"] = gtsam::noiseModel::Isotropic::Sigma(2, 1e-2);
  noise_models["camera_positive"] =
      gtsam::noiseModel::Constrained::MixedSigmas(CameraWithDistortion(0, 0, 0, 0, 0, 1, 1, 1, 1));
  noise_models["marker_translation"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
}

template <typename Key, typename T>
bool check_map_for_init(std::unordered_map<Key, T>& map, const Key& key, const T& init_value)
{
  bool res{ false };
  if (map.count(key) == 0)
  {
    map[key] = init_value;
    res = true;
  }
  return res;
}

void create_marker_fg(const Line& line, gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
                      bool add_aruco_markers)
{
  const std::string camera_name{ line[0] };
  const int marker_id{ std::stoi(line[7]) };
  bool new_maker = false;

  total_cameras += check_map_for_init(cameras_ids, camera_name, total_cameras);

  std::vector<camera_feature_factor_t::Pixel> features;
  std::vector<prx_symbol_t> feature_keys;
  std::vector<prx_symbol_t> rotation_keys;
  std::vector<prx_symbol_t> position_keys;
  std::vector<prx_symbol_t> translation_keys;

  const prx_symbol_t camera_key{ symbol_factory_t::create_symbol("camera", cameras_ids[camera_name]) };
  const prx_symbol_t s_key{ symbol_factory_t::create_symbol("scale", cameras_ids[camera_name]) };

  check_map_for_init(camera_values, camera_key, camera_initial_value);
  if (!values.exists(camera_key))
  {
    values.insert(camera_key, camera_values[camera_key]);
    values.insert(s_key, Eigen::Vector<double, 1>(1));
    const positive_vector_factor_t<CameraWithDistortionDim> positive_camera_vec(camera_key, noise_models["camera_"
                                                                                                         "positive"]);
    graph.add(positive_camera_vec);
  }

  for (int i = 0; i < 4; ++i)
  {
    const double u{ std::stod(line[8 + i * 2]) };
    const double v{ std::stod(line[8 + i * 2 + 1]) };
    const Pixel pixel(u, v);

    feature_keys.push_back(symbol_factory_t::create_symbol("feature", marker_id, i));
    rotation_keys.push_back(symbol_factory_t::create_symbol("rotation", marker_id, i));
    position_keys.push_back(symbol_factory_t::create_symbol("position", marker_id, i));
    translation_keys.push_back(symbol_factory_t::create_symbol("translation", marker_id, i));

    if (!values.exists(feature_keys.back()))
    {
      new_maker = true;
      const camera_feature_factor_t camera_feature_factor(feature_keys.back(), camera_key, rotation_keys.back(),
                                                          position_keys.back(), translation_keys.back(), s_key,
                                                          noise_models["feature"]);
      graph.add(camera_feature_factor);
      graph.addPrior(feature_keys.back(), pixel, noise_models["markers_pixels"]);
      values.insert(rotation_keys.back(), rotation_initial_value);
      values.insert(position_keys.back(), position_initial_value);
      values.insert(translation_keys.back(), translation_initial_value);
    }
    values.insert_or_assign(feature_keys.back(), pixel);
  }

  if (new_maker)
  {
    const prx_symbol_t marker_length_key{ symbol_factory_t::create_symbol("distance", 0) };
    const prx_symbol_t marker_diagonal_key{ symbol_factory_t::create_symbol("distance", 1) };
    for (int i = 0; i < 4; ++i)
    {
      const position_distance_factor distance_factor(marker_length_key, position_keys[i], position_keys[(i + 1) % 4],
                                                     noise_models["distance_between_markers"]);
      graph.add(distance_factor);
    }

    for (int i = 0; i < 2; ++i)
    {
      const position_distance_factor distance_factor(marker_diagonal_key, position_keys[i], position_keys[i + 2],
                                                     noise_models["distance_between_markers"]);
      graph.add(distance_factor);
    }
  }
  if (add_aruco_markers)
  {
    prx::prx_symbol_t aruco_marker_translation =
        symbol_factory_t::create_symbol("aruco_marker_tra", marker_id, cameras_ids[camera_name]);
    prx::prx_symbol_t aruco_marker_rotation =
        symbol_factory_t::create_symbol("aruco_marker_rot", marker_id, cameras_ids[camera_name]);

    prx::fg::aruco_marker_translation_factor_t marker_translation_factor(
        aruco_marker_translation, feature_keys[0], feature_keys[1], feature_keys[2], feature_keys[3], camera_key,
        marker_length, noise_models["marker_translation"]);
    prx::fg::aruco_marker_rotation_factor_t marker_rotation_factor(
        aruco_marker_rotation, feature_keys[0], feature_keys[1], feature_keys[2], feature_keys[3], camera_key,
        marker_length, noise_models["marker_translation"]);

    if (!values.exists(aruco_marker_translation))
    {
      graph.add(marker_translation_factor);
      graph.add(marker_rotation_factor);
      values.insert(aruco_marker_translation, translation_initial_value);
      values.insert(aruco_marker_rotation, rotation_initial_value);

      for (auto key : camera_marker_map[camera_key])
      {
        const prx_symbol_t distance_key{ symbol_factory_t::create_hashed_symbol("distance_between_tags", marker_id, key,
                                                                                cameras_ids[camera_name]) };
        const position_distance_factor distance_factor(distance_key, aruco_marker_translation, key,
                                                       noise_models["distance_between_markers"]);
        graph.add(distance_factor);
        values.insert(distance_key, Eigen::Vector<double, 1>(1));
        distances++;
      }
      camera_marker_map[camera_key].push_back(aruco_marker_translation);
    }
  }
}

void create_markers_length_fg(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values)
{
  const prx_symbol_t marker_length_key{ symbol_factory_t::create_symbol("distance", 0) };
  const prx_symbol_t marker_diagonal_key{ symbol_factory_t::create_symbol("distance", 1) };

  values.insert(marker_length_key, Eigen::Vector<double, 1>(marker_length));
  values.insert(marker_diagonal_key, Eigen::Vector<double, 1>(marker_diagonal));

  graph.addPrior(marker_length_key, Eigen::Vector<double, 1>(marker_length), noise_models["distance_between_markers"]);
  graph.addPrior(marker_diagonal_key, Eigen::Vector<double, 1>(marker_diagonal),
                 noise_models["distance_between_markers"]);
}

void update_values()
{
}

int main(int argc, char** argv)
{
  init();
  auto params = prx::param_loader("executables/perception/camera_calibration.yaml", argc, argv);
  std::string markers_file = params["markers_file"].as<>();
  csv_reader_t reader(markers_file, ' ');

  std::size_t l_idx{ 0 };

  camera_initial_value << 1000, 0, params["image_width"].as<int>() / 2,  // no-lint
      0, 1000, params["image_height"].as<int>() / 2,                     // no-lint
      0, 0, 1;
  const std::string factor_graph_file{ prx::out_path + params["factor_graph_file"].as<>() };
  marker_length = params["marker_length"].as<double>();
  marker_diagonal = std::sqrt(std::pow(marker_length, 2) + std::pow(marker_length, 2));

  prx::fg::formatter_t graph_formatter;
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::utilities::default_levenberg_marquardt_parameters() };
  factor_graph_logger_t friction_map_logger(prx::out_path + params["factor_graph_logger"].as<>(), ' ', "-");

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  create_markers_length_fg(graph, values);

  bool add_aruco_markers{ false };
  bool compute_marginals{ false };
  while (reader.has_next_line())
  {
    // auto line = reader.next_line();
    auto line = reader.next_line<std::string>("/camera_logitech/pracsys/markers", 0);
    // PRX_DEBUG_ITERABLE(line);
    if (line.size() == 0 || std::stoi(line[7]) == 0)
      continue;

    create_marker_fg(line, graph, values, add_aruco_markers);

    graph.saveGraph(factor_graph_file, values, prx::key_formatter, graph_formatter);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
    values = prx::fg::utilities::optimize_and_log(optimizer, lm_params, friction_map_logger, 0);

    values.print("values", prx::key_formatter);
    const double error{ graph.error(values) };
    PRX_DEBUG_VAR_1(error);
    add_aruco_markers = error < 1e-3;

    if (compute_marginals)
    {
      gtsam::Marginals marginals{ graph, values };
      auto f = symbol_factory_t::create_symbol("feature", 50, 0);
      std::cout << prx::key_formatter(f) << " " << marginals.marginalCovariance(f) << std::endl;
    }
    compute_marginals = add_aruco_markers;
  }
  return 0;
}