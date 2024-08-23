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
using Bars = std::tuple<SE3, SE3, SE3>;
using BarEndCaps = std::pair<Translation, Translation>;

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
  /**
  each timestep has
  pos: 3 * [x,y,z,qw,qx,qy,qz]
  gt_sites: Ground truth 3 rods * [3 * 10 xyz locations of cable attachment points],
  pred_sites: best initial estimate of 3 rods * [3 * 10 xyz locations of cable attachment points],
  end_pts: Ground truth 3 rods * [3 * 2 xyz locations of end cap centers],

  the end caps are enumerated as 0 to 5 e.g. first rod is rod01, 2nd is rod23, 3rd is rod45
  also the motors are also enumerated b0 to b5
  the sites naming convention is s_{end_cap/motor its on}_{end_cap/motor its connected to}
  **/
  std::ifstream f(prx::lib_path + "/data/tensegrity/6d_estimation.json");
  nlohmann::json json{ nlohmann::json::parse(f) };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  int t{ 0 };
  const Translation offset(0, 0, 3.25 / 2.0);
  const double noise_limit{ params["noise"].as<double>() };

  gtsam::noiseModel::Base::shared_ptr se3_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };
  gtsam::noiseModel::Base::shared_ptr se3_prior{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-2) };
  gtsam::noiseModel::Base::shared_ptr z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1.0) };

  std::ofstream ofs_gt(prx::out_path + "/tensegrity_gt.txt");
  std::ofstream ofs_encaps(prx::out_path + "/tensegrity_gt_caps.txt");
  ofs_gt << "# Qw0 Qx0 Qy0 Qz0 x0 y0 z0 Qw1 Qx1 Qy1 Qz1 x1 y1 z1 Qw2 Qx2 Qy2 Qz2 x2 y2 z2\n";
  ofs_encaps << "# x01_0 y01_0 z01_0 x01_1 y01_1 z01_1 x23_0 y23_0 z23_0 x23_1 y23_1 z23_1 x45_0 y45_0 z45_0 x45_1 "
                "y45_1 z45_1\n";

  std::vector<Bars> poses;
  std::vector<double> timestamps;
  std::vector<std::vector<BarEndCaps>> rods;
  for (auto& element : json)
  {
    const double ti{ element["time"].template get<double>() };
    const std::vector<double> vec{ element["pos"].template get<std::vector<double>>() };

    const nlohmann::json js_rod01{ element["end_pts"]["rod_01"] };
    const nlohmann::json js_rod23{ element["end_pts"]["rod_23"] };
    const nlohmann::json js_rod45{ element["end_pts"]["rod_45"] };

    const Translation r01pt0{ js_rod01["rod_01_end_pt0"].template get<std::vector<double>>().data() };
    const Translation r01pt1{ js_rod01["rod_01_end_pt1"].template get<std::vector<double>>().data() };
    const Translation r23pt0{ js_rod23["rod_23_end_pt0"].template get<std::vector<double>>().data() };
    const Translation r23pt1{ js_rod23["rod_23_end_pt1"].template get<std::vector<double>>().data() };
    const Translation r45pt0{ js_rod45["rod_45_end_pt0"].template get<std::vector<double>>().data() };
    const Translation r45pt1{ js_rod45["rod_45_end_pt1"].template get<std::vector<double>>().data() };

    const Eigen::Vector3d position0(vec[0], vec[1], vec[2]);
    const Eigen::Quaterniond quat0(vec[3], vec[4], vec[5], vec[6]);
    const Eigen::Vector3d position1(vec[7 + 0], vec[7 + 1], vec[7 + 2]);
    const Eigen::Quaterniond quat1(vec[7 + 3], vec[7 + 4], vec[7 + 5], vec[7 + 6]);
    const Eigen::Vector3d position2(vec[14 + 0], vec[14 + 1], vec[14 + 2]);
    const Eigen::Quaterniond quat2(vec[14 + 3], vec[14 + 4], vec[14 + 5], vec[14 + 6]);

    timestamps.emplace_back(ti);
    const SE3 b0{ quat0, position0 };
    const SE3 b1{ quat1, position1 };
    const SE3 b2{ quat2, position2 };
    const Bars bars{ std::make_tuple(b0, b1, b2) };
    poses.push_back(bars);

    rods.push_back({ std::make_pair(r01pt0, r01pt1),  // no-lint
                     std::make_pair(r23pt0, r23pt1),  // no-lint
                     std::make_pair(r45pt0, r45pt1) });

    ofs_gt << b0 << " ";
    ofs_gt << b1 << " ";
    ofs_gt << b2 << "\n";

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
  const std::size_t N{ params["N"].as<std::size_t>() };
  const double div_size{ params["size"].as<double>() };  // t_{half}

  const double t0{ timestamps[0] };
  const double tF{ timestamps.back() };
  // const double divisions{ 2.0 };

  PRX_DBG_VARS(N, div_size, t0, tF);
  // std::size_t div_num{ 0 };
  double a{ 0 };
  double b{ 0 };

  for (int i = 0; i < timestamps.size(); ++i)
  {
    const double ti{ timestamps[i] };
    const int div_num{ determine_limits(a, b, ti, t0, tF, div_size) };

    // PRX_DBG_VARS(ti, div_num, a, b);
    for (int r = 0; r < 3; ++r)
    {
      const gtsam::Key kx{ rod_symbol(r, i) };
      const gtsam::Key kp{ poly_symbol(r, div_num) };
      const std::string skX{ SF::formatter(kx) };
      const std::string skP{ SF::formatter(kp) };

      const Translation rand0{ prx::uniform_random<Translation>(-noise_limit, noise_limit) };
      const Translation rand1{ prx::uniform_random<Translation>(-noise_limit, noise_limit) };

      const BarEndCaps rod_caps{ rods[i][r] };
      const Translation r1{ rod_caps.first + rand0 };
      const Translation r2{ rod_caps.second + rand1 };

      // graph.emplace_shared<ManifoldChebyshev>(0, poses[i], se3_noise, N, timestamps[i], a, b);
      graph.emplace_shared<manifold_evaluation_t<gtsam::Chebyshev2, SE3>>(kp, kx, se3_noise, N, ti, a, b);

      graph.emplace_shared<SE3ObsFactor>(kx, -offset, r1, z_noise);
      graph.emplace_shared<SE3ObsFactor>(kx, offset, r2, z_noise);
      // graph.addPrior(kx0, poses[i], se3_prior);
      // initial_values.insert(kx0, poses[i]);
      //
      const Eigen::Quaterniond init_quat{ Eigen::Quaterniond::FromTwoVectors(Translation(0, 0, 1), r2 - r1) };
      const SE3 midpt{ init_quat, (r1 + r2) / 2.0 };

      initial_values.insert(kx, midpt);
      initial_values.insert_or_assign(kp, gtsam::ParameterMatrix<6>(N));
    }
  }

  // gtsam::ParameterMatrix<6> init_params(N);

  // SF::symbols_to_file();
  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  // lm_params.setUseFixedLambdaFactor(true);
  lm_params.setMaxIterations(params["iterations"].as<int>());

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };
  // result.print();

  std::ofstream ofs_fg(prx::out_path + "/tensegrity_fg.txt");

  for (int i = 0; i < timestamps.size(); ++i)
  {
    const double ti{ timestamps[i] };
    const int div_num{ determine_limits(a, b, ti, t0, tF, div_size) };

    const gtsam::Chebyshev2::ManifoldEvaluationFunctor<SE3> f(N, ti, a, b);
    for (int r = 0; r < 3; ++r)
    {
      const gtsam::Key kp{ poly_symbol(r, div_num) };
      gtsam::ParameterMatrix<6> params{ result.at<gtsam::ParameterMatrix<6>>(kp) };
      ofs_fg << f(params) << " ";
    }
    ofs_fg << "\n";
  }
  ofs_fg.close();

  return 0;
}
#else
int main(int argc, char* argv[])
{
  std::cout << "JSON not built" << std::endl;
  return -1;
}
#endif