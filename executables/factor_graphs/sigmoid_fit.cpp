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

class sigmoid_fit_t : public gtsam::NoiseModelFactor1<double, double, double>
{
  using Base = gtsam::NoiseModelFactor1<double, double, double>;
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

public:
  sigmoid_fit_t(const gtsam::Key kL, const gtsam::Key kx0, const gtsam::Key kK,  // no-lint
                const double x, const double y, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, kL, kx0, kK), _x(x), _y(y)
  {
  }

  static double logistics(const double& x, const double& L, const double& x0, const double& k)
  {
    const double f{ L / (1.0 + std::exp(-k * (x - x0))) };
    return f;
  }

  virtual Eigen::VectorXd evaluateError(const double& L, const double& x0, const double& k,  // no-lint
                                        OptionalMatrix HL = boost::none,                     // no-lint
                                        OptionalMatrix Hx0 = boost::none,                    // no-lint
                                        OptionalMatrix Hk = boost::none) const override
  {
    const double error{ logistics(_x, L, x0, k) - _y };
    // [,  (L*exp(-k*(x - x0))*(x - x0))/(exp(-k*(x - x0)) + 1)^2]
    if (HL)
    {
      *HL = Eigen::Matrix<double, 1, 1>::Zero();
      (*HL)(0, 0) = logistics(_x, 1.0, x0, k);  // 1.0 / (std::exp(-k * (x - x0)) + 1.0);
    }
    if (Hx0)
    {
      *Hx0 = Eigen::Matrix<double, 1, 1>::Zero();
      (*Hx0)(0, 0) = -(L * k * std::exp(-k * (_x - x0))) / std::pow(std::exp(-k * (_x - x0)) + 1, 2);
    }
    if (Hk)
    {
      *Hk = Eigen::Matrix<double, 1, 1>::Zero();
      (*Hk)(0, 0) = (L * std::exp(-k * (_x - x0)) * (_x - x0)) / std::pow(std::exp(-k * (_x - x0)) + 1, 2);
    }
    return Eigen::Vector<double, 1>(error);
  }

  const double _x;
  const double _y;
};

class power_fit_t : public gtsam::NoiseModelFactorN<double, double>
{
  using Base = gtsam::NoiseModelFactorN<double, double>;
  using OptionalMatrix = boost::optional<Eigen::MatrixXd&>;

public:
  power_fit_t(const gtsam::Key ka, const gtsam::Key kb,  // no-lint
              const double x, const double y, const NoiseModel& cost_model = nullptr)
    : Base(cost_model, ka, kb), _x(x), _y(y)
  {
  }

  static double fit(const double& x, const double& a, const double& b,  // no-lint
                    OptionalMatrix Ha = boost::none, OptionalMatrix Hb = boost::none)
  {
    const double f{ a * std::pow(x, b) };

    if (Ha)
    {
      // [x^b, a*x^b*log(x)];
      *Ha = Eigen::Matrix<double, 1, 1>::Identity() * std::pow(x, b);
    }
    if (Hb)
    {
      *Hb = Eigen::Matrix<double, 1, 1>::Identity() * a * std::pow(x, b) * std::log(x);
    }
    return f;
  }

  virtual Eigen::VectorXd evaluateError(const double& a, const double& b,  // no-lint
                                        OptionalMatrix Ha = boost::none,   // no-lint
                                        OptionalMatrix Hb = boost::none) const override
  {
    const Eigen::Vector<double, 1> error{ fit(_x, a, b, Ha, Hb) - _y };

    return error;
  }

  const double _x;
  const double _y;
};

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["file"].set("None");

  params.add_opts(argc, argv);

  const std::string filename{ params["file"].as<>() };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const gtsam::Key k_L{ gtsam::Symbol('L', 0) };
  const gtsam::Key k_x{ gtsam::Symbol('x', 0) };
  const gtsam::Key k_K{ gtsam::Symbol('K', 0) };
  const gtsam::Key k_a{ gtsam::Symbol('a', 0) };
  const gtsam::Key k_b{ gtsam::Symbol('b', 0) };

  prx_assert(std::filesystem::exists(filename), "Filename [" << filename << "] does not exists.");
  CsvReader reader(filename, ' ');

  while (reader.has_next_line())
  {
    auto line = reader.next_line();

    if (line.size() == 0)
      continue;

    const double xi{ convert_to<double>(line[3]) };
    const double yi{ convert_to<double>(line[1]) };
    std::cout << xi << " " << yi << std::endl;
    // graph.emplace_shared<sigmoid_fit_t>(k_L, k_x, k_K, xi, yi);
    graph.emplace_shared<power_fit_t>(k_a, k_b, xi, yi);
  }
  PRX_DBG_VARS(graph.size());

  initial_values.insert(k_L, 1.0);
  initial_values.insert(k_x, 0.0);
  initial_values.insert(k_K, 1.0);
  initial_values.insert(k_a, 1.0);
  initial_values.insert(k_b, 1.0);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(100);

  PRX_MSG("Starting optimizer");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  // const double L{ result.at<double>(k_L) };
  // const double x0{ result.at<double>(k_x) };
  // const double K{ result.at<double>(k_K) };
  // PRX_DBG_VARS(L, x0, K);
  //
  const double a{ result.at<double>(k_a) };
  const double b{ result.at<double>(k_b) };
  PRX_DBG_VARS(a, b);

  return 0;
}