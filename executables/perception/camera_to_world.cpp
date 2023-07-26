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
#include "prx/factor_graphs/utilities/perception/camera.hpp"

#include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/factors/aruco_marker.hpp"
#include "prx/factor_graphs/factors/camera_calibration_factor.hpp"
#include "prx/factor_graphs/factors/euclidian_distance_factor.hpp"
#include "prx/factor_graphs/factors/positive_vector_factor.hpp"
#include "prx/factor_graphs/utilities/constants.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/formatter.hpp"

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

using namespace prx;
using sf = symbol_factory_t;
using prx::utilities::csv_reader_t;
using partial_positive_3d_vec = typename prx::fg::partial_positive_vector_factor_t<3>;
using distance_3d_factor = typename prx::fg::euclidean_distance_factor_t<3>;
using ctw_factor = typename prx::fg::camera_to_world_factor_t;

namespace fs = std::filesystem;
const Eigen::Vector3d Zero_3{ Eigen::Vector3d::Zero() };
const Eigen::Vector3d Ones_3{ Eigen::Vector3d::Ones() };
const Eigen::Vector4d Ones_4{ Eigen::Vector4d::Ones() };

const Eigen::Vector4d quat_init{ 0.5, 0.5, 0.5, 0.5 };

// Create symbol of marker's corner from markers id & corner id
auto sy_w_corner = [](std::size_t mid, std::size_t cid) {
  return sf::create_hashed_symbol("W_AR_P", mid, "CORNER", cid);
};

// Create symbol of marker's center from markers id
auto sy_w_center = [](std::size_t mid) { return sf::create_hashed_symbol("W_AR_P", mid); };

int main(int argc, char** argv)
{
  auto params = prx::param_loader("executables/perception/camera_to_world.yaml", argc, argv);
  std::vector<std::string> images_paths = params["images"].as<std::vector<std::string>>();

  Eigen::Matrix3d camera_matrix;
  Eigen::Vector<double, 5> distortion_coeff;
  const double marker_length{ params["marker_length"].as<double>() };
  const double marker_to_center{ marker_length / std::sqrt(2.0) };
  // init_from_container(camera_matrix, params["camera_matrix"].as<std::vector<double>>());
  // init_from_container(distortion_coeff, params["distortion_coeff"].as<std::vector<double>>());
  const std::vector<Eigen::Vector3d> corners{ { marker_length, marker_length, 0 },
                                              { marker_length, -marker_length, 0 },
                                              { -marker_length, marker_length, 0 },
                                              { -marker_length, -marker_length, 0 } };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::unordered_map<std::string, gtsam::noiseModel::Base::shared_ptr> noise_models;

  const Eigen::Vector3d pixels_sigmas{ 0.1, 0.1, 1e-5 };
  noise_models["pixels"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e0);
  noise_models["markers_distance"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e0);
  noise_models["in_markers_distance"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-1);
  noise_models["normalize_quat"] = gtsam::noiseModel::Isotropic::Sigma(1, 1e-1);
  noise_models["zero_frame"] = gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
  noise_models["positive_quat"] = gtsam::noiseModel::Constrained::All(4);
  noise_models["positive_vec"] = gtsam::noiseModel::Constrained::All(3);
  noise_models["pixel_value"] = gtsam::noiseModel::Diagonal::Sigmas(pixels_sigmas);
  noise_models["cam_params"] = gtsam::noiseModel::Isotropic::Sigma(5, 1e-5);

  std::size_t cam_id{ 0 };
  std::unordered_set<std::size_t> marker_world_ids{};

  // Iterate over images
  for (const auto& img_str : images_paths)
  {
    cv::Mat img{ cv::imread(img_str, cv::IMREAD_COLOR) };
    // uvs.emplace_back(img.rows / 2.0, img.cols / 2.0);
    prx_assert(!img.empty(), "Image cannot be read");
    auto markers = aruconano::MarkerDetector::detect(img);

    const prx_symbol_t c_param_i{ symbol_factory_t::create_hashed_symbol("C_PARAM", cam_id) };
    const prx_symbol_t c_r_i{ sf::create_hashed_symbol("C_R", cam_id) };
    const prx_symbol_t c_t_i{ sf::create_hashed_symbol("C_T", cam_id) };

    // Iterate over Markers
    for (const auto& m : markers)
    {
      // if (m.id == 1 || m.id > 3)
      if (m.id != 2)
        continue;
      m.draw(img);
      std::size_t corner_id{ 0 };
      Eigen::Vector3d center{ Eigen::Vector3d::Zero() };
      const prx_symbol_t ci_pixel_ar_j{ sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", m.id) };
      const prx_symbol_t w_ar_p_i{ sf::create_hashed_symbol("W_AR_P", m.id) };
      marker_world_ids.insert(m.id);

      // Iterate over corners
      for (auto pt : m)
      {
        const Eigen::Vector3d pixels_corner{ pt.x, pt.y, 1.0 };
        const Eigen::Vector3d rand_vec{ corners[corner_id] };
        const prx_symbol_t w_ar_p_i_corner_j{ sy_w_corner(m.id, corner_id) };
        const prx_symbol_t ci_pixel_ar_j_corner_k{ sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", m.id, "CORNER",
                                                                            corner_id) };
        corner_id++;
        const prx_symbol_t w_ar_p_i_corner_jp1{ sy_w_corner(m.id, (corner_id % 4)) };
        const prx_symbol_t ci_pixel_ar_j_corner_kp1{ sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", m.id, "CORNER",
                                                                              (corner_id % 4)) };

        center += pixels_corner;

        graph.add(distance_3d_factor(marker_length, w_ar_p_i_corner_j, w_ar_p_i_corner_jp1,
                                     noise_models["in_markers_distance"]));
        graph.add(
            distance_3d_factor(marker_to_center, w_ar_p_i_corner_j, w_ar_p_i, noise_models["in_markers_distance"]));

        // graph.add(
        //     ctw_factor(ci_pixel_ar_j_corner_k, c_t_i, c_r_i, c_param_i, w_ar_p_i_corner_j, noise_models["pixels"]));
        graph.addPrior(ci_pixel_ar_j_corner_k, pixels_corner, noise_models["pixel_value"]);
        values.insert(ci_pixel_ar_j_corner_k, pixels_corner);
        values.insert(w_ar_p_i_corner_j, rand_vec);
      }
      center = center / 4.0;
      graph.addPrior(ci_pixel_ar_j, center, noise_models["pixel_value"]);
      graph.add(partial_positive_3d_vec(Eigen::Vector3d{ 0, 0, 1 }, w_ar_p_i, noise_models["positive_vec"]));
      // graph.add(partial_positive_3d_vec(Eigen::Vector3d{ 0, 0, 1 }, w_ar_p_i, noise_models["positive_vec"]));
      graph.add(ctw_factor(ci_pixel_ar_j, c_t_i, c_r_i, c_param_i, w_ar_p_i, noise_models["pixels"]));

      values.insert(ci_pixel_ar_j, center);
    }
    graph.add(prx::fg::positive_vector_factor_t<5>(c_param_i, noise_models["cam_params"]));
    graph.add(prx::fg::normalize_factor_t<4>(c_r_i, noise_models["normalize_quat"]));
    graph.add(prx::fg::positive_vector_factor_t<4>(c_r_i, noise_models["positive_quat"]));
    graph.add(partial_positive_3d_vec(Eigen::Vector3d{ 0, 0, 1 }, c_t_i, noise_models["positive_vec"]));

    // const Eigen::Vector<double, 5> cam_init_i{ 0.5, 0.5, img.rows / 2.0, img.cols / 2.0, 0 };
    const Eigen::Vector<double, 5> cam_init_i{ 0.5, 0.5, img.cols / 2.0, img.rows / 2.0, 0.1 };

    values.insert(c_t_i, Ones_3);
    values.insert(c_r_i, quat_init);
    values.insert(c_param_i, cam_init_i);
    cv::imwrite(prx::out_path + "perception/" + fs::path(img_str).filename().string(), img);
    cam_id++;
  }
  // graph.add(distance_3d_factor(1.219, sf::create_hashed_symbol("W_AR_P", 2), sf::create_hashed_symbol("W_AR_P", 3),
  //                              noise_models["markers_distance"]));
  for (auto mid : marker_world_ids)
  {
    const prx_symbol_t w_ar_p_i{ sy_w_center(mid) };
    const Eigen::Vector3d rand_vec{ Zero_3 + Eigen::Vector3d(uniform_random(), uniform_random(), 0) };
    values.insert(w_ar_p_i, rand_vec);
  }
  // graph.addPrior(sf::create_hashed_symbol("W_AR_P", 2), Zero_3, noise_models["zero_frame"]);
  values.insert_or_assign(sf::create_hashed_symbol("W_AR_P", 2), Zero_3);
  // Eigen::Vector3d init_3 = Zero_3 + Eigen::Vector3d(0, 1, 0);
  // values.insert_or_assign(sf::create_hashed_symbol("W_AR_P", 3), init_3);

  logger_t fg_log(prx::out_path + "ctw.log");
  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(10000);
  lm_params.setRelativeErrorTol(1e-10);
  lm_params.setAbsoluteErrorTol(1e-10);
  lm_params.setlambdaUpperBound(1e64);
  // lm_params.setVerbosityLM("TERMINATION");

  symbol_factory_t::symbols_to_file();
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, lm_params);
  auto results = fg::utilities::optimize_and_log(optimizer, lm_params, fg_log, 0);
  graph.printErrors(results, "graph", prx::symbol_factory_t::formatter);
  results.print("ctw", prx::symbol_factory_t::formatter);

  logger_t results_logger(out_path + "camera_to_world_results.txt");
  // for (int i = 0; i < marker_world_locations.size(); ++i)
  for (auto mid : marker_world_ids)
  {
    const prx_symbol_t sy_w{ sy_w_center(mid) };
    const auto res_w_ar_i{ results.at<Eigen::Vector3d>(sy_w) };
    results_logger("Wi", sf::formatter(sy_w), res_w_ar_i.transpose());
    for (int j = 0; j < 4; ++j)
    {
      const prx_symbol_t sy_w_c{ sy_w_corner(mid, j) };
      const auto res_w_c{ results.at<Eigen::Vector3d>(sy_w_c) };
      results_logger("Wci", sf::formatter(sy_w_c), res_w_c.transpose());
    }
  }

  std::function<std::tuple<bool, Eigen::Vector<double, 2>>(const gtsam::Values&, const gtsam::Key&)>
      variables_positions = [&](const gtsam::Values& values, const gtsam::Key& key)  // no-lint
  {
    bool bool_res{ false };
    Eigen::Vector<double, 2> pos_res{ Eigen::Vector<double, 2>::Zero() };
    if (marker_world_ids.count(key) > 0)
    {
      const auto res_w_ar_i{ results.at<Eigen::Vector3d>(key) };
      bool_res = true;
      pos_res = 1000 * res_w_ar_i.head(2);
    }

    return std::make_tuple(bool_res, pos_res);
  };
  prx::fg::create_gml_file(graph, results, prx::out_path + "perception/ctw.gml", variables_positions);

  return 0;
}