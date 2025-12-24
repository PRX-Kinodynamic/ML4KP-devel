#include <iostream>
#include <map>

#ifndef JSON
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/factors/se3_observation.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/simulation/plants/first_order_free_body.hpp"
#include "prx/factor_graphs/factors/obstacle_factor.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/factors/screw_smoothing.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>

#include <nlohmann/json.hpp>

using SF = prx::fg::symbol_factory_t;
using Translation = Eigen::Vector3d;
using SE3ObsFactor = prx::fg::SE3_observation_factor_t;
using ScrewAxis = prx::fg::screw_axis_t;
using SE3 = prx::fg::se3_t;
using Integrator = prx::fg::lie_integration_factor_t<SE3, ScrewAxis>;
// using ScrewSmothing = prx::fg::screw_smoothing_factor_t;
using Bars = std::tuple<SE3, SE3, SE3>;
using BarEndCaps = std::pair<Translation, Translation>;
using csv_reader_t = prx::utilities::csv_reader_t;

gtsam::Key rod_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("X^{", rod, "}_{", t, "}");
}

gtsam::Key vel_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("V^{", rod, "}_{", t, "}");
}

gtsam::Key accel_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("\\dot{V}^{", rod, "}_{", t, "}");
}
gtsam::Key wrench_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("F^{", rod, "}_{", t, "}");
}

void read_endcaps_file(const std::string filename, std::vector<Eigen::Vector3d>& endcaps)
{
  using prx::utilities::convert_to;

  prx_assert(std::filesystem::exists(filename), "Filename [" << filename << "] does not exists.");
  csv_reader_t reader(filename, ' ');
  while (reader.has_next_line())
  {
    auto line = reader.next_line<std::string>();
    if (line.size() == 0)
      continue;
    const double x{ convert_to<double>(line[0]) };
    const double y{ convert_to<double>(line[1]) };
    const double z{ convert_to<double>(line[2]) };
    endcaps.emplace_back(x, y, z);
  }
}
template <typename Container>
std::size_t min_container_size(const Container& container)
{
  return container.size();
}

template <typename Container, typename... Containers>
std::size_t min_container_size(const Container& container, const Containers... containers)
{
  return std::min(min_container_size(container), min_container_size(containers...));
}

void read_timestamps(const std::string dir, std::vector<double>& timestamps)
{
  std::map<std::size_t, double> seqs_map;
  for (auto const& dir_entry : std::filesystem::directory_iterator{ dir })
  {
    std::ifstream f(dir_entry.path());
    nlohmann::json json = nlohmann::json::parse(f);
    // for (const nlohmann::json& element : json)
    // {
    // PRX_DBG_VARS(element);
    auto header = json["header"];
    const std::size_t seq{ header["seq"].template get<std::size_t>() };
    const double secs{ header["secs"].template get<double>() };

    seqs_map[seq] = secs;
    // }
  }

  const double t0{ seqs_map.begin()->second };
  for (auto seq_secs : seqs_map)
  {
    timestamps.emplace_back(seq_secs.second - t0);
  }
  // PRX_DBG_VARS(timestamps);
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["dir"].set("None");
  params["noise"].set("0.1");
  params["time_dir"].set("None");
  params["iterations"].set(100);

  params.add_opts(argc, argv);

  const std::string timestamps_directory{ params["time_dir"].as<>() };
  const std::string directory{ params["dir"].as<>() };

  std::vector<double> timestamps;
  read_timestamps(timestamps_directory, timestamps);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  int t{ 0 };
  const Translation offset(0, 0.0, 3.25 / 2.0);
  const double noise_limit{ params["noise"].as<double>() };
  const double noise_sigma{ 1.0 };
  // const double noise_sigma{ noise_limit == 0 ? 1.0 : noise_limit };
  // const double noise_sigma{ noise_limit };
  gtsam::noiseModel::Base::shared_ptr se3_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };
  gtsam::noiseModel::Base::shared_ptr se3_prior{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-2) };
  gtsam::noiseModel::Base::shared_ptr z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, noise_sigma) };

  std::ofstream ofs_gt(prx::out_path + "/tensegrity_gt.txt");
  std::ofstream ofs_encaps(prx::out_path + "/tensegrity_gt_caps.txt");

  std::vector<std::vector<BarEndCaps>> rods;
  std::vector<Eigen::Vector3d> ec0, ec1, ec2, ec3, ec4, ec5;
  read_endcaps_file(directory + "/0.txt", ec0);
  read_endcaps_file(directory + "/1.txt", ec1);
  read_endcaps_file(directory + "/2.txt", ec2);
  read_endcaps_file(directory + "/3.txt", ec3);
  read_endcaps_file(directory + "/4.txt", ec4);
  read_endcaps_file(directory + "/5.txt", ec5);

  const std::size_t min_data{ min_container_size(ec0, ec1, ec2, ec3, ec4, ec5) };
  for (int i = 0; i < min_data; ++i)
  {
    // Order: rod01_endpt0, rod01_endpt1, rod23_endpt0, rod23_endpt1, rod45_endpt0, rod45_endpt1
    const Translation r01pt0{ ec0[i] };
    const Translation r01pt1{ ec1[i] };
    const Translation r23pt0{ ec2[i] };
    const Translation r23pt1{ ec3[i] };
    const Translation r45pt0{ ec4[i] };
    const Translation r45pt1{ ec5[i] };

    rods.push_back({ std::make_pair(r01pt0, r01pt1),  // no-lint
                     std::make_pair(r23pt0, r23pt1),  // no-lint
                     std::make_pair(r45pt0, r45pt1) });

    ofs_encaps << r01pt0.transpose() << " ";
    ofs_encaps << r01pt1.transpose() << " ";
    ofs_encaps << r23pt0.transpose() << " ";
    ofs_encaps << r23pt1.transpose() << " ";
    ofs_encaps << r45pt0.transpose() << " ";
    ofs_encaps << r45pt1.transpose() << "\n";
  }
  ofs_gt.close();
  ofs_encaps.close();

  // exit(0);
  // const std::size_t N{ params["N"].as<std::size_t>() };
  // const double div_size{ params["size"].as<double>() };  // t_{half}

  const double t0{ timestamps[0] };
  const double tF{ timestamps.back() };
  // const double divisions{ 2.0 };

  // PRX_DBG_VARS(N, div_size, t0, tF);
  // std::size_t div_num{ 0 };
  // double a{ 0 };
  // double b{ 0 };

  // const Translation offset_axis{ offset.normalized() };
  // for (int i = 0; i < timestamps.size(); ++i)
  // {
  //   const double ti{ timestamps[i] };
  //   const int div_num{ determine_limits(a, b, ti, t0, tF, div_size) };
  //   const Bars& bars{ poses[i] };
  //   const std::vector<SE3> rods_i{ { std::get<0>(bars), std::get<1>(bars), std::get<2>(bars) } };

  //   // PRX_DBG_VARS(ti, div_num, a, b);
  //   for (int r = 0; r < 3; ++r)
  //   {
  //     const gtsam::Key kx{ rod_symbol(r, i) };
  //     const gtsam::Key kp{ poly_symbol(r, div_num) };
  //     const std::string skX{ SF::formatter(kx) };
  //     const std::string skP{ SF::formatter(kp) };

  //     const Translation rand0{ prx::gaussian_random<Translation>(-noise_limit, noise_limit) };
  //     const Translation rand1{ prx::gaussian_random<Translation>(-noise_limit, noise_limit) };

  //     const BarEndCaps rod_caps{ rods[i][r] };
  //     const Translation r1{ rod_caps.first + rand0 };
  //     const Translation r2{ rod_caps.second + rand1 };

  //     // graph.emplace_shared<ManifoldChebyshev>(0, poses[i], se3_noise, N, timestamps[i], a, b);
  //     graph.emplace_shared<manifold_evaluation_t<gtsam::Chebyshev2, SE3>>(kp, kx, se3_noise, N, ti, a, b);

  //     graph.emplace_shared<SE3ObsFactor>(kx, -offset, r1, z_noise);
  //     graph.emplace_shared<SE3ObsFactor>(kx, offset, r2, z_noise);

  //     // g_T_r1.linear() = Eigen::Ma;
  //     // R_T_r1;
  //     // graph.addPrior(kx0, poses[i], se3_prior);
  //     // initial_values.insert(kx0, poses[i]);
  //     const Eigen::Quaterniond init_quat{ Eigen::Quaterniond::FromTwoVectors(offset_axis, r2 - r1) };
  //     const SE3 midpt_init{ init_quat, (r1 + r2) / 2.0 };
  //     const SE3 midpt{ rods_i[r] };

  //     // const Eigen::Quaterniond init_quat{ Eigen::Quaterniond::FromTwoVectors(r1, r2) };

  //     // PRX_DBG_VARS(r1.transpose(), r2.transpose());
  //     PRX_DBG_VARS(midpt);
  //     PRX_DBG_VARS(midpt_init);

  //     // initial_values.insert(kx, midpt);
  //     initial_values.insert(kx, midpt_init);
  //     initial_values.insert_or_assign(kp, gtsam::ParameterMatrix<6>(N));
  //   }
  // }

  // gtsam::ParameterMatrix<6> init_params(N);

  // SF::symbols_to_file();
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  // lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(params["iterations"].as<int>());

  PRX_MSG("Starting optimizer");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };
  // gtsam::Values result{ initial_values };
  // result.print();
  const std::function<bool(const gtsam::Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>&
      printCondition = [&](const gtsam::Factor*, double err, size_t) { return err > 2.0; };

  // graph.printErrors(result, "Error: ", SF::formatter, printCondition);

  std::ofstream ofs_fg_se3(prx::out_path + "/tensegrity_fg.txt");
  std::ofstream ofs_fg_endcaps(prx::out_path + "/tensegrity_fg_caps.txt");
  ofs_fg_se3 << "# Qw0 Qx0 Qy0 Qz0 x0 y0 z0 Qw1 Qx1 Qy1 Qz1 x1 y1 z1 Qw2 Qx2 Qy2 Qz2 x2 y2 z2\n";

  // for (int i = 0; i < timestamps.size(); ++i)
  // {
  //   const double ti{ timestamps[i] };
  //   const int div_num{ determine_limits(a, b, ti, t0, tF, div_size) };

  //   const gtsam::Chebyshev2::ManifoldEvaluationFunctor<SE3> f(N, ti, a, b);
  //   for (int r = 0; r < 3; ++r)
  //   {
  //     const gtsam::Key kp{ poly_symbol(r, div_num) };
  //     gtsam::ParameterMatrix<6> params{ result.at<gtsam::ParameterMatrix<6>>(kp) };
  //     const SE3 se3{ f(params) };
  //     const Translation cap0{ SE3ObsFactor::predict(se3, offset) };
  //     const Translation cap1{ SE3ObsFactor::predict(se3, -offset) };
  //     ofs_fg_se3 << se3 << " ";
  //     ofs_fg_endcaps << cap0.transpose() << " " << cap1.transpose() << " ";
  //   }
  //   ofs_fg_se3 << "\n";
  //   ofs_fg_endcaps << "\n";
  // }
  ofs_fg_se3.close();
  ofs_fg_endcaps.close();

  return 0;
}
#else
int main(int argc, char* argv[])
{
  std::cout << "JSON not built" << std::endl;
  return -1;
}
#endif
