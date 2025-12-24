#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <functional>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/plants/pusher_slider.hpp"
#include "prx/factor_graphs/utilities/values_utilities.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/factor_graphs/factors/constraint_factor.hpp"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>

using State = prx::fg::SE2_t;
using CsvReader = prx::utilities::csv_reader_t;
using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
using Point = Eigen::Vector2d;
using prx::utilities::convert_to;
using PositiveDoubleFactor = prx::fg::constraint_factor_t<double, prx::fg::DoubleCmp<std::less<>>>;

class circle_fit_t : public prx::fg::noise_model_2factor_t<Point, double>
{
  using Base = prx::fg::noise_model_2factor_t<Point, double>;

public:
  circle_fit_t(const gtsam::Key k_center, const gtsam::Key& k_rad, const Point pt,
               const NoiseModel& cost_model = nullptr)
    : Base(k_center, k_rad, cost_model, 0.01), _pt(pt)
  {
  }

  virtual Point predict(const double& x1) const override
  {
    return Point::Zero();
  }

  static Point compute_point(const Point& center, const double& radius, const double& angle)
  {
    const Point sc{ std::cos(angle), std::sin(angle) };
    const Point pt_p{ center + radius * sc };
    return pt_p;
  }

  static double compute_angle(const Point& center, const Point& pt)
  {
    const double angle{ std::atan2(pt[1] - center[1], pt[0] - center[0]) };
    return angle;
  }

  virtual Error compute_error(const Point& center, const double& radius) const override
  {
    const double angle{ compute_angle(center, _pt) };
    // const double angle{ std::atan2(_pt[1] - center[1], _pt[0] - center[0]) };
    // const Point sc{ std::cos(angle), std::sin(angle) };
    const Point pt_p{ compute_point(center, radius, angle) };
    const Point error{ pt_p - _pt };
    // PRX_DBG_VARS(_pt.transpose(), pt_p.transpose());
    return error;
  }

  const Point _pt;
};

void read_tf(const std::string filename, std::vector<Point>& trajectory)
{
  prx_assert(std::filesystem::exists(filename), "Filename [" << filename << "] does not exists.");
  CsvReader reader(filename, ' ');

  double t_prev{ -1 };
  std::vector<State> states;
  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;

    // const double t{ convert_to<double>(line[0]) };
    // const double t{ convert_to<double>(line[0]) };

    const double x{ convert_to<double>(line[1]) };
    const double y{ convert_to<double>(line[2]) };
    // const double z{ convert_to<double>(line[3]) };

    // 8 => "t x y z qw qx qy qz"
    if (line.size() == 8)
    {
      const double qw{ convert_to<double>(line[4]) };
      const double qx{ convert_to<double>(line[5]) };
      const double qy{ convert_to<double>(line[6]) };
      const double qz{ convert_to<double>(line[7]) };
      const Eigen::Quaterniond q{ Eigen::Quaterniond(qw, qx, qy, qz) };

      const double angle{ prx::quaternion_to_euler(q)[2] };
      states.emplace_back(x, y, angle);
    }
    // 7 => "t x y th xDto yDot thDot"; 4 => "t x y th"
    else if (line.size() == 7 or line.size() == 4)
    {
      const double angle{ convert_to<double>(line[3]) };
      states.emplace_back(x, y, angle);
    }

    // State x0_inv{ 0, 0, 0 };
    // start_states.emplace_back(x0_inv * x0);
  }
  const State x0_inv{ states[0].inverse() };

  for (auto state : states)
  {
    const State xi{ x0_inv * state };
    trajectory.emplace_back(xi[0], xi[1]);
  }
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["file"].set("None");

  params.add_opts(argc, argv);

  const std::string filename{ params["file"].as<>() };
  std::vector<Point> trajectory;
  read_tf(filename, trajectory);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const gtsam::Key k_rad{ gtsam::Symbol('R', 0) };
  const gtsam::Key k_center{ gtsam::Symbol('C', 0) };

  for (auto state : trajectory)
  {
    graph.emplace_shared<circle_fit_t>(k_center, k_rad, state);
  }
  PRX_DBG_VARS(graph.size());
  // gtsam::noiseModel::Base::shared_ptr rad_noise{ gtsam::noiseModel::Isotropic::Sigma(1, 1e-4) };
  // graph.emplace_shared<PositiveDoubleFactor>(k_rad, 0.0, rad_noise);

  const Point mid{ trajectory.back() - trajectory[0] };
  const double radius_initial{ mid.norm() / 2.0 };
  const Point center_initial{ mid / 2.0 };
  initial_values.insert(k_rad, radius_initial);
  initial_values.insert(k_center, center_initial);
  PRX_DBG_VARS(radius_initial);
  PRX_DBG_VARS(center_initial.transpose());

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  PRX_MSG("Starting optimizer");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  const double error{ graph.error(result) };

  const double radius_result{ result.at<double>(k_rad) };
  const Point center_result{ result.at<Point>(k_center) };
  PRX_DBG_VARS(radius_result);
  PRX_DBG_VARS(center_result.transpose());

  const double L{ 0.31 };
  const double beta{ std::asin(L / (2.0 * radius_result)) };
  const double delta{ std::atan(2.0 * std::tan(beta)) };

  std::ofstream ofs_out(params["out"].as<>());
  ofs_out << "# radius center(x,y) error\n";
  ofs_out << radius_result << " ";
  ofs_out << center_result.transpose() << " ";
  ofs_out << error << " ";
  ofs_out << beta << " ";
  ofs_out << delta << " ";
  ofs_out << "\n";
  // std::ofstream ofs(prx::out_path + "/circle_fit.txt");
  // std::ofstream ofs_circle(prx::out_path + "/circle_fit_estimated.txt");

  // const double th0{ circle_fit_t::compute_angle(center_result, trajectory[0]) };
  // const double thT{ circle_fit_t::compute_angle(center_result, trajectory.back()) };
  // const double angle_diff(thT - th0);
  // const double arc_length{ angle_diff * radius_result };
  // PRX_DBG_VARS(arc_length);

  // for (auto pt : trajectory)
  // {
  //   ofs << pt.transpose() << "\n";
  // }
  // for (double i = 0; i < 2 * prx::constants::pi; i += 0.1)
  // {
  //   const Point circle_pt{ circle_fit_t::compute_point(center_result, radius_result, i) };
  //   ofs_circle << circle_pt.transpose() << "\n";
  // }
  // ofs.close();
  // ofs_circle.close();

  return 0;
}