#include <iostream>

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
using ScrewSmothing = prx::fg::screw_smoothing_factor_t;

gtsam::Key rod_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("X^{", rod, "}_{", t, "}");
}

gtsam::Key poly_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("p^{", rod, "}_{", t, "}");
}

gtsam::Key rodvel_symbol(const int rod, const int t)
{
  return SF::create_hashed_symbol("\\dot{X}^{", rod, "}_{", t, "}");
}

template <class Basis, typename Type>
class manifold_evaluation_t
  : public gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<gtsam::traits<Type>::dimension>, Type>
{
  static constexpr Eigen::Index Dim{ gtsam::traits<Type>::dimension };

  using Base = gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<Dim>, Type>;
  using Derived = manifold_evaluation_t<Basis, Type>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using JacobianXX = Eigen::Matrix<double, Dim, Dim>;
  using JacobianXP = Eigen::Matrix<double, Dim, -1>;

public:
  manifold_evaluation_t(const gtsam::Key keyPolyParams, const gtsam::Key keyX, const NoiseModel& cost_model,
                        const size_t N, double x, double a, double b)
    : Base(cost_model, keyPolyParams, keyX), _evaluation_function(N, x, a, b)
  {
  }

  virtual Eigen::VectorXd evaluateError(const gtsam::ParameterMatrix<Dim>& P, const Type& x,  // no-lint
                                        OptDeriv Hp = boost::none, OptDeriv Hx = boost::none) const override
  {
    const bool compute_derivs{ (Hx or Hp) };
    // BASIS::template ManifoldEvaluationFunctor<T>(N, x);
    const Type predicted{ _evaluation_function(P, compute_derivs ? &dxp_H_P : nullptr) };

    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const Type between{ x.between(predicted,                          // no-lint
                                  compute_derivs ? &b_H_x : nullptr,  // no-lint
                                  compute_derivs ? &b_H_xp : nullptr) };
    const Eigen::VectorXd error{ Type::Logmap(between, compute_derivs ? &err_H_b : nullptr) };

    if (Hp)
    {
      *Hp = err_H_b * b_H_xp * dxp_H_P;
    }
    if (Hx)
    {
      *Hx = err_H_b * b_H_x;
    }

    return error;
  }

private:
  const typename Basis::template ManifoldEvaluationFunctor<Type> _evaluation_function;

  mutable JacobianXX err_H_b;  // Deriv error wrt between

  mutable JacobianXX b_H_x;   // Deriv between wrt x
  mutable JacobianXX b_H_xp;  // Deriv between wrt x_{predicted}

  mutable Eigen::MatrixXd dxp_H_P;  // Deriv \dot{predicted} wrt Params
};

int determine_limits(double& a, double& b, const double& ti, const double& t0, const double& tF, const double& div_size)
{
  const double ti_pos{ ti / (tF - t0) };
  const double total_divisions{ (tF - t0) / div_size };
  const double offset{ std::floor(ti_pos * total_divisions) };
  a = t0 + offset * div_size;
  b = a + div_size;
  return offset;
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["N"].set(1000);
  params["size"].set(2);
  params["iterations"].set(5);
  params["noise"].set(0.0);

  params.add_opts(argc, argv);

  std::ifstream f(prx::lib_path + "/data/tensegrity/data.json");
  nlohmann::json json{ nlohmann::json::parse(f) };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  int t{ 0 };
  const Translation offset(0, 0, 3.25 / 2.0);
  const double noise_limit{ params["noise"].as<double>() };

  gtsam::noiseModel::Base::shared_ptr se3_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };
  gtsam::noiseModel::Base::shared_ptr se3_prior{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-2) };
  gtsam::noiseModel::Base::shared_ptr z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1.0 / noise_limit) };

  std::ofstream ofs_gt(prx::out_path + "/tensegrity_gt.txt");
  ofs_gt << "# Qw Qx Qy Qz x y z\n";

  std::vector<SE3> poses;
  std::vector<double> timestamps;
  std::vector<Translation> rod_end1;
  std::vector<Translation> rod_end2;
  for (auto& element : json)
  {
    // const Translation rand0{ prx::uniform_random<Translation>(0.0, 0.10) };
    // const Translation rand1{ prx::uniform_random<Translation>(0.0, 0.10) };

    const double ti{ element["time"].template get<double>() };
    const std::vector<double> vec{ element["pos"].template get<std::vector<double>>() };
    const Translation r01pt1{ element["r01_end_pt1"].template get<std::vector<double>>().data() };
    const Translation r01pt2{ element["r01_end_pt2"].template get<std::vector<double>>().data() };

    const Eigen::Vector3d position(vec[0], vec[1], vec[2]);
    const Eigen::Quaterniond quat(vec[3], vec[4], vec[5], vec[6]);
    // SE3 se3(quat, position);

    timestamps.emplace_back(ti);
    poses.emplace_back(quat, position);
    rod_end1.emplace_back(r01pt1);
    rod_end2.emplace_back(r01pt2);
    // std::cout << ti << '\n';
    ofs_gt << poses.back() << "\n";
  }
  const std::size_t N{ params["N"].as<std::size_t>() };
  const double div_size{ params["size"].as<double>() };  // t_{half}

  const double t0{ timestamps[0] };
  const double tF{ timestamps.back() };
  // const double divisions{ 2.0 };

  PRX_DBG_VARS(N, div_size);
  // std::size_t div_num{ 0 };
  double a{ 0 };
  double b{ 0 };

  for (int i = 0; i < timestamps.size(); ++i)
  {
    const double ti{ timestamps[i] };
    const int div_num{ determine_limits(a, b, ti, t0, tF, div_size) };

    // PRX_DBG_VARS(ti, div_num, a, b);

    const gtsam::Key kx0{ rod_symbol(0, i) };
    const gtsam::Key kp{ poly_symbol(0, div_num) };
    const std::string skX{ SF::formatter(kx0) };
    const std::string skP{ SF::formatter(kp) };

    const Translation rand0{ prx::uniform_random<Translation>(-noise_limit, noise_limit) };
    const Translation rand1{ prx::uniform_random<Translation>(-noise_limit, noise_limit) };

    const Translation r01pt1{ rod_end1[i] + rand0 };
    const Translation r01pt2{ rod_end2[i] + rand1 };

    // graph.emplace_shared<ManifoldChebyshev>(0, poses[i], se3_noise, N, timestamps[i], a, b);
    graph.emplace_shared<manifold_evaluation_t<gtsam::Chebyshev2, SE3>>(kp, kx0, se3_noise, N, ti, a, b);

    graph.emplace_shared<SE3ObsFactor>(kx0, -offset, r01pt1, z_noise);
    graph.emplace_shared<SE3ObsFactor>(kx0, offset, r01pt2, z_noise);

    // graph.addPrior(kx0, poses[i], se3_prior);
    // initial_values.insert(kx0, poses[i]);
    //
    const Eigen::Quaterniond init_quat{ Eigen::Quaterniond::FromTwoVectors(Translation(0, 0, 1), r01pt2 - r01pt1) };
    const SE3 midpt01{ init_quat, (r01pt1 + r01pt2) / 2.0 };

    initial_values.insert(kx0, midpt01);
    initial_values.insert_or_assign(kp, gtsam::ParameterMatrix<6>(N));
  }

  // gtsam::ParameterMatrix<6> init_params(N);

  // SF::symbols_to_file();
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  // lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(params["iterations"].as<int>());

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  // gtsam::GaussianFactorGraph::shared_ptr gfg = graph.linearize(initial_values);
  // gtsam::VectorValues result = gfg->optimize();
  // parameters_ = solution.at(0);
  gtsam::Values result{ optimizer.optimize() };
  // result.print();

  std::ofstream ofs_fg(prx::out_path + "/tensegrity_fg.txt");

  for (int i = 0; i < timestamps.size(); ++i)
  {
    const double ti{ timestamps[i] };
    const int div_num{ determine_limits(a, b, ti, t0, tF, div_size) };

    const gtsam::Key kp{ poly_symbol(0, div_num) };
    gtsam::ParameterMatrix<6> params{ result.at<gtsam::ParameterMatrix<6>>(kp) };
    const gtsam::Chebyshev2::ManifoldEvaluationFunctor<SE3> f(N, ti, a, b);
    ofs_fg << f(params) << "\n";
  }
  ofs_fg.close();
  // gtsam::Chebyshev2::Parameters parameters{ result.at<gtsam::Chebyshev2::Parameters>(0) };

  // PRX_DBG_VARS(parameters);
  // for (double xi = -1.0; xi < 1.0; xi += 0.01)
  // {
  //   std::cout << xi << " " << f(xi) << " " << p(params, xi, N) << "\n";
  // }
  // result.print("Result", SF::formatter);

  // std::ofstream ofs_init(prx::lib_path + "/out/tensegrity_initial.txt");
  // std::ofstream ofs_res(prx::lib_path + "/out/tensegrity_result.txt");

  // ofs_init << "# Qw Qx Qy Qz x y z\n";
  // ofs_res << "# Qw Qx Qy Qz x y z\n";
  // for (int i = 0; i < t; ++i)
  // {
  //   ofs_init << initial_values.at<SE3>(rod_symbol(1, i)) << "\n";
  //   ofs_res << result.at<SE3>(rod_symbol(1, i)) << "\n";
  // }

  // prx::fg::values_by_type_to_file<ScrewAxis>(initial_values, prx::out_path + "/tensegrity_initial_screw.txt");
  // prx::fg::values_by_type_to_file<ScrewAxis>(result, prx::out_path + "/tensegrity_result_screw.txt");

  // prx::fg::values_by_type_to_file<SE3>(initial_values, prx::out_path + "/tensegrity_initial_se3.txt");
  // prx::fg::values_by_type_to_file<SE3>(result, prx::out_path + "/tensegrity_result_se3.txt");
  // initial_values.print("NonlinearFactorGraph", SF::formatter);
  // graph.printErrors(result, "NonlinearFactorGraph: ", SF::formatter);

  return 0;
}
#else
int main(int argc, char* argv[])
{
  std::cout << "JSON not built" << std::endl;
  return -1;
}
#endif