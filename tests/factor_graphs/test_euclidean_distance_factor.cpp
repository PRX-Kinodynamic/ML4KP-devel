#define BOOST_AUTO_TEST_MAIN symbols_factory_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/factor_graphs/factors/euclidean_distance_factor.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"
#include "prx/utilities/defs.hpp"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
using prx::fg::euclidean_distance_factor_t;

BOOST_AUTO_TEST_CASE(evaluate_error)
{
  const Eigen::Index Dimension{ 3 };
  using Point = Eigen::Vector<double, Dimension>;
  using Error = Eigen::Vector<double, 1>;

  const gtsam::Key kp0{ 0 };
  const gtsam::Key kp1{ 1 };
  const double distance{ std::sqrt(5) };

  euclidean_distance_factor_t<Dimension> factor(kp0, kp1, distance, gtsam::noiseModel::Isotropic::Sigma(1, 1));

  const Point p0(0, 0, 0);
  const Point p1(0, 1, 2);
  const Error p00_dist(-5);  // 0 - 5
  const Error p01_dist(0);   // (1^2+2^2) - 5

  const Error e00{ factor.evaluateError(p0, p0) };
  const Error e01{ factor.evaluateError(p0, p1) };
  BOOST_CHECK_MESSAGE(std::abs(p01_dist[0] - e01[0]) < 1e-4, EXPECTED_GOT(p01_dist, e01));
  BOOST_CHECK_MESSAGE(std::abs(p00_dist[0] - e00[0]) < 1e-4, EXPECTED_GOT(p00_dist, e00));
}

BOOST_AUTO_TEST_CASE(min_distance)
{
  const Eigen::Index Dimension{ 3 };
  using Point = Eigen::Vector<double, Dimension>;
  using Error = Eigen::Vector<double, 1>;

  const gtsam::Key kp0{ 0 };
  const gtsam::Key kp1{ 1 };
  const double distance{ std::sqrt(5) };
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  const Point p0(0, 0, 0);
  const Point p1(0, 0, 0);

  // graph.addPrior(kp0, p0);
  graph.emplace_shared<euclidean_distance_factor_t<Dimension>>(kp0, kp1, distance,
                                                               gtsam::noiseModel::Isotropic::Sigma(1, 1e0));
  values.insert(kp0, p0);
  values.insert(kp1, p1);

  // printf("------------\n");
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  // gtsam::Values results = prx::fg::optimize_and_log(optimizer, lm_params);
  gtsam::Values results = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();

  Point r_p0{ results.at<Point>(kp0) };
  Point r_p1{ results.at<Point>(kp1) };
  // std::cout << r_p0 << "\n";
  // std::cout << r_p1 << "\n";
  BOOST_CHECK_MESSAGE(std::abs((r_p0 - r_p1).norm() - distance) < 1e-2, EXPECTED_GOT(distance, (r_p0 - r_p1).norm()));
}
