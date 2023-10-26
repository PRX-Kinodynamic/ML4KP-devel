#include <algorithm>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unordered_map>
#include <unordered_set>

#include "prx/utilities/defs.hpp"
#include "prx/external/aruco_nano.h"
#include "prx/utilities/general/logger.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/type_convertions.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/perception/camera.hpp"
#include "prx/factor_graphs/factors/aruco_marker.hpp"
#include "prx/factor_graphs/factors/camera_calibration_factor.hpp"
#include "prx/factor_graphs/factors/coplanar_factors.hpp"
#include "prx/factor_graphs/factors/euclidian_distance_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/factors/transform_factors.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"
#include "prx/factor_graphs/utilities/fg_to_csv.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <thread>

#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;
using namespace prx;

using prx::utilities::csv_reader_t;

using sf = symbol_factory_t;
using partial_positive_3d_vec = typename prx::fg::partial_positive_vector_factor_t<3>;
using distance_3d_factor = typename prx::fg::euclidean_distance_factor_t<3>;
using ctw_factor = typename prx::fg::camera_to_world_factor_t;
using projection_factor = typename prx::fg::camera_projection_factor_t;
using aruco_projection_factor = typename prx::fg::aruco_camera_projection_factor_t;
using coplanar_factor = typename prx::fg::coplanar_4_point3D_factor_t;
using corner_angle_factor = typename prx::fg::angle_between_3_3d_points_factor_t;
using corner_offset_factor = typename prx::fg::point_offset_in_local_frame_factor_t;
using rotation_vec_mat_factor = typename prx::fg::rotation_vector_matrix_factor_t;
using global_offset_factor = typename prx::fg::point_offset_in_global_frame_factor_t;
using camera_phi_factor = typename prx::fg::camera_phi_factor_t;
using csv_reader_t = prx::utilities::csv_reader_t;

using Pixel = Eigen::Vector2d;

const Eigen::Vector3d Zero_3{ Eigen::Vector3d::Zero() };
const Eigen::Vector3d Ones_3{ Eigen::Vector3d::Ones() };
const Eigen::Vector4d Ones_4{ Eigen::Vector4d::Ones() };

const Eigen::Vector4d quat_init{ 0.5, 0.5, 0.5, 0.5 };

// Create symbol of marker's corner from markers id & corner id
auto sy_w_corner = [](std::size_t mid, std::size_t cid) {
  return sf::create_hashed_symbol("W_AR_P", mid, "CORNER", cid);
};

auto sy_p_center = [](std::size_t cam_id, std::size_t mid) {
  return sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", mid);
};

auto sy_marker_pixel = [](std::size_t cam_id, std::size_t mid) {
  return sf::create_hashed_symbol("M^{", cam_id, "}_{", mid, "}");
};
auto sy_marker_position = [](std::size_t mid) { return sf::create_hashed_symbol("M^{POS}_{", mid, "}"); };
auto dyn_marker_position = [](std::size_t mid, std::size_t t) {
  return sf::create_hashed_symbol("M^{", t, "}_{", mid, "}");
};

auto sy_p_corner = [](std::size_t cam_id, std::size_t mid, std::size_t corner_id) {
  return sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", mid, "CORNER", corner_id);
};
auto sy_projection = [](std::size_t cam_id) { return sf::create_hashed_symbol("C^{PROJECTION}_", cam_id); };
// Create symbol of marker's center from markers id
// auto sy_w_center = [](std::size_t mid) { return sf::create_hashed_symbol("W_AR_P", mid); };
auto sy_rot_vec = [](std::size_t cam_id) { return sf::create_hashed_symbol("R", cam_id); };
auto sy_tra_vec = [](std::size_t cam_id) { return sf::create_hashed_symbol("t", cam_id); };
auto sy_scale = [](std::size_t cam_id) { return sf::create_hashed_symbol("s_", cam_id); };

std::vector<std::string> get_filenames(const std::string path)
{
  std::vector<std::string> files;
  if (std::filesystem::is_directory(path))
  {
    for (auto const& dir_entry : std::filesystem::directory_iterator{ path })
    {
      files.push_back(dir_entry.path());
    }
  }

  return files;
}

template <typename ValueType, typename Ids, typename SymbolFunction, typename SymbolPositions>
void update_symbol_positions(const Ids& ids, const SymbolFunction& symbol_function, SymbolPositions& symbol_positions,
                             gtsam::Values& values)
{
  for (auto m : ids)
  {
    const prx::prx_symbol_t symbol{ symbol_function(m) };
    symbol_positions[symbol] = values.at<ValueType>(symbol).head(2);
  }
}
int main(int argc, char** argv)
{
  prx::param_loader params{ "executables/perception/camera_to_world.yaml", argc, argv };
  const bool use_image_dir{ params["use_image_dir"].as<bool>() };
  std::vector<std::string> images_paths;
  if (use_image_dir)
  {
    const std::string image_dir{ params["image_dir"].as<std::string>() };
    images_paths = get_filenames(image_dir);
  }
  else
  {
    images_paths = params["images"].as<std::vector<std::string>>();
  }
  std::string markers_file{ params["markers_file"].as<std::string>() };
  std::unordered_map<prx::prx_symbol_t, Pixel> symbol_positions;

  Eigen::Matrix3d camera_matrix;
  Eigen::Vector<double, 5> distortion_coeff;
  const double marker_length{ params["marker_length"].as<double>() };
  const double marker_to_center{ marker_length / std::sqrt(2.0) };
  // init_from_container(camera_matrix, params["camera_matrix"].as<std::vector<double>>());
  // init_from_container(distortion_coeff, params["distortion_coeff"].as<std::vector<double>>());
  const std::vector<Eigen::Vector3d> corners{ { -marker_length, -marker_length, 0 },
                                              { marker_length, marker_length, 0 },
                                              { marker_length, -marker_length, 0 },
                                              { -marker_length, marker_length, 0 } };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> noise_models;

  const Eigen::Vector3d pixels_sigmas{ 0.1, 0.1, 1e-5 };
  noise_models["pixels"] = gtsam::noiseModel::Isotropic::Sigma(2, 1e-2);
  noise_models["markers_distance"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["coplanar_corners"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["coplanar_markers"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["in_markers_distance"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["markers_offset"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["normalize_quat"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["zero_frame"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["positive_quat"] = gtsam::noiseModel::Constrained::All(4);
  noise_models["positive_vec"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["pixel_value"] = gtsam::noiseModel::Isotropic::Sigma(2, 1e-5);
  noise_models["cam_params"] = gtsam::noiseModel::Isotropic::Sigma(5, 1e0);
  noise_models["projection"] = gtsam::noiseModel::Isotropic::Sigma(2, 1e0);
  noise_models["corners_dist"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["pos_projection"] = gtsam::noiseModel::Isotropic::Sigma(12, 1e0);
  noise_models["norm_projection"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["corner_offset"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["rotation_vec"] = gtsam::noiseModel::Isotropic::Sigma(9, 1e-1);
  noise_models["proj_trans"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["proj_rotvec"] = gtsam::noiseModel::Isotropic::Sigma(9, 1e0);
  noise_models["markers_prior"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
  noise_models["camera_phi"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["scale"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-5);

  std::size_t cam_id{ 0 };
  std::unordered_set<std::size_t> marker_world_ids{};

  std::vector<std::vector<aruconano::Marker>> markers_set;
  // Iterate over images
  for (const auto& img_str : images_paths)
  {
    cv::Mat img{ cv::imread(img_str, cv::IMREAD_COLOR) };

    // uvs.emplace_back(img.rows / 2.0, img.cols / 2.0);
    prx_assert(!img.empty(), "Image cannot be read");
    std::vector<aruconano::Marker> markers{ aruconano::MarkerDetector::detect(img) };
    markers_set.push_back(markers);
    for (const auto& m : markers)
    {
      m.draw(img);
    }
    cv::imwrite(prx::out_path + "perception/" + fs::path(img_str).filename().string(), img);
  }
  // Iterate over Markers
  for (const auto& markers : markers_set)
  {
    for (const auto& m : markers)
    {
      if (m.id == 0)
        continue;
      std::size_t corner_id{ 0 };
      Eigen::Vector2d center{ Eigen::Vector2d::Zero() };

      const prx_symbol_t ci_pixel_ar_j{ sy_p_center(cam_id, m.id) };
      // const prx_symbol_t w_ar_p_i{ sf::create_hashed_symbol("W_AR_P", m.id) };
      marker_world_ids.insert(m.id);

      // Iterate over corners
      for (auto pt : m)
      {
        const Eigen::Vector2d pixels_corner{ pt.x, pt.y };
        const Eigen::Vector3d corner_vec{ corners[corner_id] };

        const prx_symbol_t w_ar_p_i_corner_j{ sy_w_corner(m.id, corner_id) };
        corner_id++;
        // const prx_symbol_t w_ar_p_i_corner_jp1{ sy_w_corner(m.id, (corner_id % 4)) };

        center += pixels_corner;
      }
      center = center / 4.0;
      graph.add(projection_factor(sy_marker_position(m.id), sy_projection(cam_id), center, noise_models["projection"]));
      // values.insert_or_assign(w_ar_p_i_corner_j, corner_vec);
    }
  }

  std::unordered_set<std::size_t> markers_ids{};
  csv_reader_t reader(markers_file, ' ');
  gtsam::Values values_markers;
  gtsam::NonlinearFactorGraph graph_markers;
  for (auto line : reader)
  {
    const std::size_t marker_id{ prx::utilities::convert_to<std::size_t>(line[0]) };
    const std::size_t x_val{ prx::utilities::convert_to<std::size_t>(line[1]) };
    const std::size_t y_val{ prx::utilities::convert_to<std::size_t>(line[2]) };
    const std::size_t z_val{ prx::utilities::convert_to<std::size_t>(line[3]) };
    graph_markers.addPrior(sy_marker_position(marker_id), Eigen::Vector3d(x_val, y_val, z_val),
                           noise_models["markers_prior"]);
    values_markers.insert(sy_marker_position(marker_id), Eigen::Vector3d(x_val, y_val, z_val));
    symbol_positions[sy_marker_position(marker_id)] = Eigen::Vector2d(x_val, y_val);
    markers_ids.insert(marker_id);
  }

  values.insert(values_markers);
  graph.add(graph_markers);

  const projection_factor::Projection camera_projection_init{ projection_factor::Projection::Ones() };
  values.insert(sy_projection(cam_id), camera_projection_init);
  graph.add(prx::fg::positive_vector_factor_t<12>(sy_projection(cam_id), noise_models["pos_projection"]));
  graph.add(prx::fg::camera_projection_norm_factor_t(sy_projection(cam_id), noise_models["norm_projection"]));
  // graph.add(prx::fg::normalize_factor_t<12>(sy_projection(cam_id), noise_models["norm_projection"]));
  cam_id++;

  std::function<std::tuple<bool, Pixel>(const gtsam::Values&, const gtsam::Key&)> variables_positions =
      [&](const gtsam::Values& values, const gtsam::Key& key)  // no-lint
  {
    bool bool_res{ false };
    Pixel pos_res{ Pixel::Zero() };
    if (symbol_positions.count(key) > 0)
    {
      bool_res = true;
      pos_res = symbol_positions[key];
    }

    return std::make_tuple(bool_res, pos_res);
  };

  logger_t fg_log(prx::out_path + "ctw.log");
  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  // lm_params.setUseFixedLambdaFactor(false);
  // lm_params.setMaxIterations(1000000);
  lm_params.setMaxIterations(1000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  // lm_params.setVerbosityLM("TERMINATION");

  prx::fg::fg_to_csv<2>(graph, values, prx::out_path + "/mj_ctw_in.txt", variables_positions);

  symbol_factory_t::symbols_to_file();
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  auto results = fg::utilities::optimize_and_log(optimizer, lm_params, fg_log, 0);

  update_symbol_positions<Eigen::Vector3d>(markers_ids, sy_marker_position, symbol_positions, results);
  prx::fg::fg_to_csv<2>(graph, results, prx::out_path + "/mj_ctw_out_1.txt", variables_positions);
  // graph.printErrors(results, "graph", prx::symbol_factory_t::formatter);
  // results.print("ctw", prx::symbol_factory_t::formatter);

  gtsam::NonlinearFactorGraph graph_2;
  gtsam::Values values_2;
  cam_id = 0;

  projection_factor::Projection P0_vec{ results.at<projection_factor::Projection>(sy_projection(cam_id)) };
  PRX_DEBUG_VAR_1(P0_vec);
  std::size_t t_marker{ 0 };
  logger_t logger(prx::out_path + "/ctw_traj.txt", ' ');
  // values_2.insert(sy_projection(cam_id), results.at<projection_factor::Projection>(sy_projection(cam_id)));

  for (const auto& markers : markers_set)
  {
    for (const auto& m : markers)
    {
      Eigen::Vector2d center{ Eigen::Vector2d::Zero() };

      for (auto pt : m)
      {
        const Eigen::Vector2d pixel{ pt.x, pt.y };
        center += pixel;
      }
      center = center / 4.0;
      logger("UV", m.id, center.transpose());
      if (m.id == 0)
      {
        graph_2.add(camera_phi_factor(dyn_marker_position(m.id, t_marker), sy_projection(cam_id), sy_scale(cam_id),
                                      center, noise_models["camera_phi"]));

        const Eigen::Vector3d init_pos{ Eigen::Vector3d::Zero() };
        results.insert(dyn_marker_position(m.id, t_marker), init_pos);
        t_marker++;
      }
      else
      {
        graph_2.add(camera_phi_factor(sy_marker_position(m.id), sy_projection(cam_id), sy_scale(cam_id), center,
                                      noise_models["camera_phi"]));
      }
    }
  }
  results.insert(sy_scale(cam_id), Eigen::Vector<double, 1>(1.0));
  graph_2.add(graph_markers);
  graph_2.add(prx::fg::positive_vector_factor_t<1>(sy_scale(cam_id), noise_models["scale"]));
  graph_2.addPrior(sy_scale(cam_id), Eigen::Vector<double, 1>(1.0), noise_models["scale"]);
  graph_2.add(prx::fg::positive_vector_factor_t<12>(sy_projection(cam_id), noise_models["pos_projection"]));
  symbol_factory_t::symbols_to_file();
  gtsam::LevenbergMarquardtOptimizer optimizer_2(graph_2, results, lm_params);
  auto results_2 = fg::utilities::optimize_and_log(optimizer_2, lm_params, fg_log, 0);
  graph_2.printErrors(results_2, "graph", prx::symbol_factory_t::formatter);
  // results_2.print("ctw", prx::symbol_factory_t::formatter);

  update_symbol_positions<Eigen::Vector3d>(markers_ids, sy_marker_position, symbol_positions, results_2);

  prx::fg::fg_to_csv<2>(graph_2, results_2, prx::out_path + "/mj_ctw_out_2.txt", variables_positions);

  for (std::size_t i = 0; i < t_marker; ++i)
  {
    camera_phi_factor::Position position{ results_2.at<camera_phi_factor::Position>(dyn_marker_position(0, i)) };
    logger("R", i, position.transpose());
  }
  for (std::size_t idx : { 62, 50, 51, 54, 64, 55 })
  {
    camera_phi_factor::Position position{ results_2.at<camera_phi_factor::Position>(sy_marker_position(idx)) };
    logger("M", idx, position.transpose());
  }

  return 0;
}