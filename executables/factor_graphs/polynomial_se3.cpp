#include <iostream>

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

using prx::utilities::convert_to;
using SF = prx::fg::symbol_factory_t;
using Translation = Eigen::Vector3d;
using SE3ObsFactor = prx::fg::SE3_observation_factor_t;
using ScrewAxis = prx::fg::screw_axis_t;
using SE3 = prx::fg::se3_t;
using Integrator = prx::fg::lie_integration_factor_t<SE3, ScrewAxis>;
using ScrewSmothing = prx::fg::screw_smoothing_factor_t;
using Graph = gtsam::NonlinearFactorGraph;

gtsam::Key x_symbol(const int idx)
{
  return gtsam::Symbol('X', idx);
}

gtsam::Key xdot_symbol(const int idx)
{
  return gtsam::Symbol('V', idx);
}

gtsam::Key poly_symbol(const int idx)
{
  return gtsam::Symbol('P', idx);
}

void read_data(const std::string& filename, std::vector<gtsam::Pose3>& data)
{
  std::ifstream file(filename.c_str());

  // Assuming input is (Position, Quat) = (x, y, th, qw, qx, qy, qz)
  // while (reader.has_next_line())
  // {
  //   auto line = reader.next_line();
  //   if (line.size() < 0)
  //     continue;

  //   data.emplace_back();
  //   std::vector<double>& curr{ data.back() };

  //   prx_assert((line.size() - 4) % 3, "Wrong number of inputs [" << line.size() << "]");
  //   for (int i = 3; i < line.size(); ++i)
  //   {
  //     curr.emplace_back(convert_to<double>(line[i]));
  //   }
  // }
}

std::vector<Translation> get_observations(const std::vector<double>& row)
{
  std::vector<Translation> observations;
  for (int i = 3; i < row.size(); i += 3)
  {
    observations.emplace_back();
    Translation& xi{ observations.back() };
    xi[0] = row[i];
    xi[1] = row[i + 1];
    xi[2] = row[i + 2];
    if (std::isnan(xi.template maxCoeff<Eigen::PropagateNaN>()))
    {
      observations.pop_back();
    }
  }
  const double idx{ row[1] };
  PRX_DBG_VARS(idx);
  return observations;
}

struct rod_t
{
  rod_t()
    : initialized(false), e0_valid(false), e1_valid(false), offset(0, 0, 3.25 / 2.0), offset_axis(offset.normalized())
  {
    ofs_init.open(prx::out_path + "/initial_values.txt");
  }

  void update_SE3()
  {
    if (e0_valid and e1_valid)
    {
      const Eigen::Quaterniond init_quat{ Eigen::Quaterniond::FromTwoVectors(offset_axis, endcap1 - endcap0) };
      const Translation position{ (endcap1 + endcap0) / 2.0 };
      current.position() = position;
      current.quaternion() = init_quat;

      ofs_init << current << " " << endcap1.transpose() << " " << endcap0.transpose() << "\n";
    }
  }

  void update_rods(const std::vector<Translation>& observations)
  {
    if (not initialized)
    {
      const Translation& z0{ observations[0] };
      endcap0 = z0;
      initialized = true;
      e0_valid = true;

      e1_valid = false;
      endcap1 = z0;  // Same value to force initialization of SE3
      if (observations.size() > 1)
      {
        const Translation& z1{ observations[1] };
        endcap1 = z1;
      }
      e1_valid = true;
    }
    else if (observations.size() >= 2)
    {
      const Translation z0{ observations[0] };
      const Translation z1{ observations[1] };

      const double dist_e0z0{ (endcap0 - z0).norm() };
      const double dist_e0z1{ (endcap0 - z1).norm() };

      const double dist_e1z0{ (endcap1 - z0).norm() };
      const double dist_e1z1{ (endcap1 - z1).norm() };

      const std::vector<double> distances{ { dist_e0z0, dist_e0z1, dist_e1z0, dist_e1z1 } };

      auto result_iter = std::min_element(distances.begin(), distances.end());
      const double min_element{ *result_iter };
      auto min_idx = std::distance(distances.begin(), result_iter);

      if (min_idx < 2)  // E0 to some z is the closest
      {
        endcap0 = dist_e0z0 < dist_e0z1 ? z0 : z1;
        endcap1 = dist_e0z0 < dist_e0z1 ? z1 : z0;
      }
      else
      {
        endcap1 = dist_e1z0 < dist_e1z1 ? z0 : z1;
        endcap0 = dist_e1z0 < dist_e1z1 ? z1 : z0;
      }
      e0_valid = true;
      e1_valid = true;
    }
    else if (observations.size() == 1)
    {
      const Translation z{ observations[0] };

      const double dist_e0z{ (endcap0 - z).norm() };
      const double dist_e1z{ (endcap1 - z).norm() };

      if (dist_e0z < dist_e1z)
      {
        endcap0 = z;
        e0_valid = true;
        e1_valid = false;
      }
      else
      {
        endcap1 = z;
        e0_valid = false;
        e1_valid = true;
      }
    }
    update_SE3();
    PRX_DBG_VARS(e0_valid, endcap0.transpose());
    PRX_DBG_VARS(e1_valid, endcap1.transpose());
    PRX_DBG_VARS(current);
  }

  std::ofstream ofs_init;

  const Translation offset;
  const Translation offset_axis;

  SE3 current;
  int id;

  bool e0_valid;
  bool e1_valid;
  Translation endcap0;
  Translation endcap1;

  bool initialized;
};

void add_observations(const gtsam::Key& kx, const std::vector<double>& row, rod_t& rod, const double noise_sigma,
                      Graph& graph)
{
  const Translation offset(0, 0, 0.325 / 2.0);

  const gtsam::noiseModel::Base::shared_ptr z_noise{ gtsam::noiseModel::Isotropic::Sigma(3, noise_sigma) };
  std::vector<Translation> observations{ get_observations(row) };
  PRINT_KEYS(kx);
  rod.update_rods(observations);

  if (rod.e0_valid)
  {
    graph.emplace_shared<SE3ObsFactor>(kx, -offset, rod.endcap0, z_noise);
  }
  if (rod.e1_valid)
  {
    graph.emplace_shared<SE3ObsFactor>(kx, offset, rod.endcap1, z_noise);
  }
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

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["print_matrix"].set(false);

  params.add_opts(argc, argv);

  const std::string filename_red{ params["red_file"].as<>() };

  Graph graph;
  gtsam::Values initial_values;

  std::vector<gtsam::Pose3> data_red;
  read_data(filename_red, data_red);

  const double a{ 0.0 };
  const double b{ 1.0 };
  const std::size_t N{ params["N"].as<std::size_t>() };
  gtsam::noiseModel::Base::shared_ptr se3_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };

  rod_t rod_red;
  const double noise_sigma{ 1.0 };

  const gtsam::Key kp{ poly_symbol(4) };
  // for (auto row : data_red)
  // {
  //   const double ti{ row[0] };
  //   const double idx{ row[1] };
  //   const double id{ row[2] };
  //   const gtsam::Key kx{ rod_symbol(id, idx) };

  //   // std::vector<Translation> observations{ get_observations(row) };

  //   add_observations(kx, row, rod_red, noise_sigma, graph);
  //   graph.emplace_shared<manifold_evaluation_t<gtsam::Chebyshev2, SE3>>(kp, kx, se3_noise, N, ti, a, b);

  //   // graph.emplace_shared<SE3ObsFactor>(kx, -offset, r1, z_noise);
  //   // graph.emplace_shared<SE3ObsFactor>(kx, offset, r2, z_noise);

  //   // initial_values.insert(kx, rod_red.current);
  //   initial_values.insert(kx, SE3::identity());
  //   initial_values.insert_or_assign(kp, gtsam::ParameterMatrix<6>(N));

  //   // ofs_init << idx << " " << initial_values.at<SE3>(kx) << "\n";
  // }
  // ofs_init.close();
  // initial_values.print("initial_values", SF::formatter);

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(params["iterations"].as<int>());

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  // graph.printErrors(result, "Graph", SF::formatter);
  // result.print("Result", SF::formatter);
  gtsam::ParameterMatrix<6> result_param_mat{ result.at<gtsam::ParameterMatrix<6>>(kp) };

  std::ofstream ofs_out(prx::out_path + "/traj_estimation.txt");

  PRX_DBG_VARS(a, b);

  if (params["print_matrix"].as<bool>())
  {
    std::cout << "matrix_values: [ ";
    const Eigen::Matrix<double, 6, -1>& mat_aux{ result_param_mat.matrix() };
    for (int i = 0; i < 6; ++i)
    {
      std::cout << "\t\t";
      for (int j = 0; j < N; ++j)
      {
        std::cout << mat_aux(i, j);
        if (j < N - 1)
        {
          std::cout << ", ";
        }
      }
      if (i < N - 1)
      {
        std::cout << ",\n";
      }
    }
    std::cout << "\t\t]";
  }
  // PRX_DBG_VARS(result_param_mat);
  // for (double ti = a; ti < b; ti += 0.1)
  for (auto row : data_red)
  {
    // const double ti{ row[0] };
    // const double idx{ row[1] };
    // const gtsam::Chebyshev2::ManifoldEvaluationFunctor<SE3> f(N, ti, a, b);
    // const SE3 se3{ f(result_param_mat) };

    // // PRX_DBG_VARS(se3);
    // // const Translation cap0{ SE3ObsFactor::predict(se3, offset) };
    // ofs_out << ti << " " << idx << " " << se3 << "\n";
  }
  ofs_out.close();

  return 0;
}