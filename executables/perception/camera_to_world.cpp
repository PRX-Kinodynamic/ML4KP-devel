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

auto sy_p_corner = [](std::size_t cam_id, std::size_t mid, std::size_t corner_id) {
  return sf::create_hashed_symbol("C", cam_id, "PIXEL_AR", mid, "CORNER", corner_id);
};
// Create symbol of marker's center from markers id
// auto sy_w_center = [](std::size_t mid) { return sf::create_hashed_symbol("W_AR_P", mid); };
auto sy_rot_vec = [](std::size_t cam_id) { return sf::create_hashed_symbol("R", cam_id); };
auto sy_tra_vec = [](std::size_t cam_id) { return sf::create_hashed_symbol("t", cam_id); };

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

  std::size_t cam_id{ 0 };
  std::unordered_set<std::size_t> marker_world_ids{};

  const Eigen::Vector<double, 9> rot_vec_init{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  const Eigen::Vector<double, 3> translation_init{ Eigen::Vector<double, 3>::Ones() };
  // Iterate over images
  for (const auto& img_str : images_paths)
  {
    cv::Mat img{ cv::imread(img_str, cv::IMREAD_COLOR) };
    Eigen::Matrix3d A_init{ Eigen::Matrix3d::Zero() };
    A_init << 1000, 10, img.cols / 2.0,  // no-lint
        0, 1000, img.rows / 2.0,         // no-lint
        0, 0, 1;
    Eigen::Matrix<double, 3, 4> Rt_init{ Eigen::Matrix<double, 3, 4>::Zero() };
    Rt_init.block<3, 3>(0, 0) = rot_vec_init.reshaped(3, 3);
    Rt_init.block<3, 1>(0, 3) = translation_init;
    PRX_DEBUG_VAR_1(A_init * Rt_init);
    const projection_factor::Projection camera_projection_init{ (A_init * Rt_init).reshaped(12, 1) };
    // uvs.emplace_back(img.rows / 2.0, img.cols / 2.0);
    prx_assert(!img.empty(), "Image cannot be read");
    auto markers = aruconano::MarkerDetector::detect(img);

    const prx_symbol_t key_ci_proj{ symbol_factory_t::create_hashed_symbol("C", cam_id, "PROJECTION") };
    const prx_symbol_t key_ci_Rv{ sy_rot_vec(cam_id) };
    const prx_symbol_t key_ci_t{ sy_tra_vec(cam_id) };

    // Iterate over Markers
    for (const auto& m : markers)
    {
      // if (m.id != 2 ||)
      // if (m.id == 1 || m.id > 3)
      if (m.id == 1)
        continue;
      m.draw(img);
      std::size_t corner_id{ 0 };
      Eigen::Vector2d center{ Eigen::Vector2d::Zero() };

      const prx_symbol_t ci_pixel_ar_j{ sy_p_center(cam_id, m.id) };
      // const prx_symbol_t w_ar_p_i{ sf::create_hashed_symbol("W_AR_P", m.id) };
      marker_world_ids.insert(m.id);

      // Iterate over corners
      for (auto pt : m)
      {
        const Eigen::Vector2d pixels_corner{ pt.x, pt.y };
        const Eigen::Vector3d rand_vec{ corners[corner_id] };

        const prx_symbol_t w_ar_p_i_corner_j{ sy_w_corner(m.id, corner_id) };
        const prx_symbol_t ci_pixel_ar_j_corner_k{ sy_p_corner(cam_id, m.id, corner_id) };
        corner_id++;
        const prx_symbol_t w_ar_p_i_corner_jp1{ sy_w_corner(m.id, (corner_id % 4)) };
        const prx_symbol_t ci_pixel_ar_j_corner_kp1{ sy_p_corner(cam_id, m.id, (corner_id % 4)) };

        center += pixels_corner;

        graph.add(corner_offset_factor(corners[corner_id], key_ci_Rv, key_ci_t, w_ar_p_i_corner_j,
                                       noise_models["corner_offset"]));

        // graph.add(distance_3d_factor(marker_length, w_ar_p_i_corner_j, w_ar_p_i_corner_jp1,
        //                              noise_models["in_markers_distance"]));
        // graph.add(
        // distance_3d_factor(marker_to_center, w_ar_p_i_corner_j, w_ar_p_i, noise_models["in_markers_distance"]));

        graph.addPrior(ci_pixel_ar_j_corner_k, pixels_corner, noise_models["pixel_value"]);
        graph.add(
            projection_factor(ci_pixel_ar_j_corner_k, w_ar_p_i_corner_j, key_ci_proj, noise_models["projection"]));

        values.insert(ci_pixel_ar_j_corner_k, pixels_corner);
        values.insert_or_assign(w_ar_p_i_corner_j, rand_vec);
      }
      // graph.add(distance_3d_factor(2 * marker_to_center, sy_w_corner(m.id, 0), sy_w_corner(m.id, 2),
      //                              noise_models["corners_dist"]));
      // graph.add(distance_3d_factor(2 * marker_to_center, sy_w_corner(m.id, 1), sy_w_corner(m.id, 3),
      //                              noise_models["corners_dist"]));
      graph.add(coplanar_factor(sy_w_corner(m.id, 0), sy_w_corner(m.id, 1), sy_w_corner(m.id, 2), sy_w_corner(m.id, 3),
                                noise_models["coplanar_corners"]));
      // graph.add(coplanar_factor(sy_w_center(m.id), sy_w_corner(m.id, 1), sy_w_corner(m.id, 2), sy_w_corner(m.id, 3),
      //                           noise_models["coplanar_corners"]));
      graph.add(corner_angle_factor(PRX_PI / 2.0, sy_w_corner(m.id, 0), sy_w_corner(m.id, 1), sy_w_corner(m.id, 3),
                                    noise_models["coplanar_corners"]));
      graph.add(corner_angle_factor(PRX_PI / 2.0, sy_w_corner(m.id, 1), sy_w_corner(m.id, 2), sy_w_corner(m.id, 0),
                                    noise_models["coplanar_corners"]));
      graph.add(corner_angle_factor(PRX_PI / 2.0, sy_w_corner(m.id, 2), sy_w_corner(m.id, 3), sy_w_corner(m.id, 1),
                                    noise_models["coplanar_corners"]));
      graph.add(corner_angle_factor(PRX_PI / 2.0, sy_w_corner(m.id, 3), sy_w_corner(m.id, 0), sy_w_corner(m.id, 2),
                                    noise_models["coplanar_corners"]));

      center = center / 4.0;
      graph.addPrior(ci_pixel_ar_j, center, noise_models["pixel_value"]);
      // graph.add(partial_positive_3d_vec(Eigen::Vector3d{ 0, 0, 1 }, w_ar_p_i, noise_models["positive_vec"]));
      // graph.add(projection_factor(ci_pixel_ar_j, w_ar_p_i, key_ci_proj, noise_models["projection"]));
      graph.add(aruco_projection_factor(ci_pixel_ar_j, sy_w_corner(m.id, 0), sy_w_corner(m.id, 1), sy_w_corner(m.id, 2),
                                        sy_w_corner(m.id, 3), key_ci_proj, noise_models["projection"]));

      values.insert(ci_pixel_ar_j, center);
    }
    values.insert(key_ci_proj, camera_projection_init);
    values.insert(key_ci_Rv, rot_vec_init);
    values.insert(key_ci_t, translation_init);

    graph.add(prx::fg::projection_to_rotation_factor_t(key_ci_Rv, key_ci_proj, noise_models["proj_rotvec"]));
    graph.add(prx::fg::projection_to_translation_factor_t(key_ci_t, key_ci_proj, noise_models["proj_trans"]));
    graph.add(rotation_vec_mat_factor(key_ci_Rv, noise_models["rotation_vec"]));
    // graph.add(prx::fg::positive_vector_factor_t<12>(key_ci_proj, noise_models["pos_projection"]));
    graph.add(prx::fg::normalize_factor_t<12>(key_ci_proj, noise_models["norm_projection"]));

    cv::imwrite(prx::out_path + "perception/" + fs::path(img_str).filename().string(), img);
    cam_id++;
  }
  // graph.add(coplanar_factor(sy_w_center(2), sy_w_center(3), sy_w_center(4), sy_w_center(5),
  //                           noise_models["coplanar_markers"]));

  for (int i = 0; i < 4; ++i)
  {
    // graph.add(distance_3d_factor(1.220, sy_w_corner(2, i), sy_w_corner(3, i), noise_models["in_markers_distance"]));
    // graph.add(distance_3d_factor(2.550, sy_w_corner(5, i), sy_w_corner(3, i), noise_models["in_markers_distance"]));
    // graph.add(distance_3d_factor(1.240, sy_w_corner(5, i), sy_w_corner(4, i), noise_models["in_markers_distance"]));
    // graph.add(distance_3d_factor(2.500, sy_w_corner(2, i), sy_w_corner(4, i), noise_models["in_markers_distance"]));
    // graph.add(distance_3d_factor(2.650, sy_w_corner(2, i), sy_w_corner(5, i), noise_models["in_markers_distance"]));
    // graph.add(distance_3d_factor(2.610, sy_w_corner(3, i), sy_w_corner(4, i), noise_models["in_markers_distance"]));
    const Eigen::Vector3d offset_zero{ Eigen::Vector3d::Zero() };
    const Eigen::Vector3d offset_x{ Eigen::Vector3d{ 2.5, 0, 0 } };
    const Eigen::Vector3d offset_y{ Eigen::Vector3d{ 0, 1.220, 0 } };
    const Eigen::Vector3d offset_xy{ offset_x + offset_y };
    const Eigen::Vector3d init_val_m2{ corners[i] };
    const Eigen::Vector3d init_val_m3{ corners[i] + offset_y };
    const Eigen::Vector3d init_val_m4{ corners[i] + offset_x };
    const Eigen::Vector3d init_val_m5{ corners[i] + offset_xy };
    graph.add(global_offset_factor(offset_y, sy_w_corner(2, i), sy_w_corner(3, i), noise_models["markers_offset"]));
    graph.add(global_offset_factor(offset_x, sy_w_corner(5, i), sy_w_corner(3, i), noise_models["markers_offset"]));
    graph.add(global_offset_factor(offset_y, sy_w_corner(5, i), sy_w_corner(4, i), noise_models["markers_offset"]));
    graph.add(global_offset_factor(offset_x, sy_w_corner(2, i), sy_w_corner(4, i), noise_models["markers_offset"]));
    graph.add(global_offset_factor(offset_xy, sy_w_corner(2, i), sy_w_corner(5, i), noise_models["markers_offset"]));
    graph.add(global_offset_factor(offset_xy, sy_w_corner(3, i), sy_w_corner(4, i), noise_models["markers_offset"]));

    values.insert_or_assign(sy_w_corner(2, i), init_val_m2);
    values.insert_or_assign(sy_w_corner(3, i), init_val_m3);
    values.insert_or_assign(sy_w_corner(4, i), init_val_m4);
    values.insert_or_assign(sy_w_corner(5, i), init_val_m5);
  }
  // for (auto mid : marker_world_ids)
  // {
  //   const prx_symbol_t w_ar_p_i{ sy_w_center(mid) };
  //   const Eigen::Vector3d rand_vec{ Zero_3 + Eigen::Vector3d(uniform_random(), uniform_random(), 0) };
  //   values.insert(w_ar_p_i, rand_vec);
  // }
  // graph.addPrior(sf::create_hashed_symbol("W_AR_P", 2), Zero_3, noise_models["zero_frame"]);
  // values.insert_or_assign(sf::create_hashed_symbol("W_AR_P", 2), Zero_3);
  // Eigen::Vector3d init_3 = Zero_3 + Eigen::Vector3d(0, 1, 0);
  // values.insert_or_assign(sf::create_hashed_symbol("W_AR_P", 3), init_3);

  logger_t fg_log(prx::out_path + "ctw.log");
  gtsam::LevenbergMarquardtParams lm_params{ fg::utilities::default_levenberg_marquardt_parameters() };
  lm_params.setUseFixedLambdaFactor(true);
  // lm_params.setUseFixedLambdaFactor(false);
  lm_params.setMaxIterations(1000000);
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
    // sy_p_center
    // sy_p_corner
    for (int cid = 0; cid < cam_id; ++cid)
    {
      const prx_symbol_t sy_p{ sy_p_center(cid, mid) };
      const auto res_pixel{ results.at<Pixel>(sy_p) };
      results_logger("Pi", sf::formatter(sy_p), res_pixel.transpose(), mid);
    }
    Eigen::Vector3d w_center{ Eigen::Vector3d::Zero() };
    for (int j = 0; j < 4; ++j)
    {
      const prx_symbol_t sy_w_c{ sy_w_corner(mid, j) };
      const auto res_w_c{ results.at<Eigen::Vector3d>(sy_w_c) };
      w_center += res_w_c;
      results_logger("Wci", sf::formatter(sy_w_c), res_w_c.transpose(), j);
      for (int cid = 0; cid < cam_id; ++cid)
      {
        const prx_symbol_t sy_p_c{ sy_p_corner(cid, mid, j) };
        const auto res_p_c{ results.at<Pixel>(sy_p_c) };
        results_logger("Pci", sf::formatter(sy_p_c), res_p_c.transpose(), j);
      }
    }
    w_center = w_center / 4.0;
    results_logger("Wi", mid, w_center.transpose());
    for (auto nid : marker_world_ids)
    {
      if (mid != nid)
      {
        Eigen::Vector3d w_center_j{ Eigen::Vector3d::Zero() };
        for (int j = 0; j < 4; ++j)
        {
          const prx_symbol_t sy_w_c{ sy_w_corner(nid, j) };
          const auto res_w_c{ results.at<Eigen::Vector3d>(sy_w_c) };
          w_center_j += res_w_c;
        }
        w_center_j = w_center_j / 4;
        results_logger("distance", w_center.transpose(), w_center_j.transpose(), (w_center - w_center_j).norm());
      }
    }
  }

  for (int i = 0; i < cam_id; ++i)
  {
    const prx_symbol_t key_ci_proj{ symbol_factory_t::create_hashed_symbol("C", i, "PROJECTION") };
    const projection_factor::Projection res_ci_proj{ results.at<projection_factor::Projection>(key_ci_proj) };
    const Eigen::Matrix<double, 3, 4> P{ res_ci_proj.reshaped(3, 4) };
    const Eigen::Matrix3d B{ P.block<3, 3>(0, 0) };
    const Eigen::Vector3d b{ P.block<3, 1>(0, 3) };
    Eigen::Matrix3d K{ B * B.transpose() };
    K = K / K(2, 2);
    const double u0{ K(0, 2) };
    const double v0{ K(1, 2) };
    const double ku{ K(0, 0) };
    const double kc{ K(0, 1) };
    const double kv{ K(1, 1) };

    const double beta{ std::sqrt(ku - v0 * v0) };
    const double gamma{ (kc - u0 * v0) / beta };
    const double alpha{ std::sqrt(ku - u0 * u0 - gamma * gamma) };
    Eigen::Matrix3d A{ Eigen::Matrix3d::Zero() };
    A << alpha, gamma, u0,  // no-lint
        0, beta, v0,        // no-lint
        0, 0, 1;
    Eigen::Matrix3d AAt{ A * A.transpose() };
    const Eigen::Matrix3d R{ A.inverse() * B };
    const Eigen::Vector3d t{ A.inverse() * b };
    PRX_DEBUG_VAR_1(P);
    PRX_DEBUG_VAR_1(K);
    PRX_DEBUG_VAR_1(A);
    PRX_DEBUG_VAR_1(AAt);
    PRX_DEBUG_VAR_1(R);
    PRX_DEBUG_VAR_1(R * R.transpose());
    PRX_DEBUG_VAR_1(t);
    // results_logger("distance", res_w_ar_i.transpose(), res_w_ar_j.transpose(), (res_w_ar_i - res_w_ar_j).norm());
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