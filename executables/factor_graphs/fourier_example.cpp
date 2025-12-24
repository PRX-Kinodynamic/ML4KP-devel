#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

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
#include "prx/factor_graphs/plants/pusher_slider.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/default_parameters.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/Fourier.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using Sequence = std::map<double, double>;
using Sample = std::pair<double, double>;
using Weights = Eigen::Matrix<double, 1, -1>; /* 1xN vector */
using CsvReader = prx::utilities::csv_reader_t;

std::vector<Eigen::Vector3d> read_data(const std::string filename)
{
  using prx::utilities::convert_to;
  std::vector<Eigen::Vector3d> data;
  CsvReader reader(filename, ' ');
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line[0] == "#")
      continue;
    const double xi{ convert_to<double>(line[1]) };
    const double yi{ convert_to<double>(line[2]) };
    const double zi{ convert_to<double>(line[3]) };
    data.emplace_back(xi, yi, zi);
  }
  return data;
}

double f(const double x)
{
  const double sign{ std::signbit(x) ? -1.0 : 1.0 };
  // return sign - x / 2.0;
  return std::sin(6.0 * x) + std::sin(60.0 * std::exp(x));
}

double p(const Eigen::VectorXd& ck, const double& x, const std::size_t& N)
{
  const Weights w{ gtsam::Chebyshev2::CalculateWeights(N, x) };
  return ck.dot(w);
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
    using TypeTraits = gtsam::traits<Type>;
    const bool compute_derivs{ (Hx or Hp) };
    // BASIS::template ManifoldEvaluationFunctor<T>(N, x);
    const Type predicted{ _evaluation_function(P, compute_derivs ? &dxp_H_P : nullptr) };

    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const Type between{ TypeTraits::Between(x, predicted,                       // no-lint
                                            compute_derivs ? &b_H_x : nullptr,  // no-lint
                                            compute_derivs ? &b_H_xp : nullptr) };
    const Eigen::VectorXd error{ TypeTraits::Logmap(between, compute_derivs ? &err_H_b : nullptr) };

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
  const typename Basis::template VectorEvaluationFunctor<Dim> _evaluation_function;

  mutable JacobianXX err_H_b;  // Deriv error wrt between

  mutable JacobianXX b_H_x;   // Deriv between wrt x
  mutable JacobianXX b_H_xp;  // Deriv between wrt x_{predicted}

  mutable Eigen::MatrixXd dxp_H_P;  // Deriv \dot{predicted} wrt Params
};

int main(int argc, char* argv[])
{
  using R3Manifold = manifold_evaluation_t<gtsam::FourierBasis, Eigen::Vector3d>;

  prx::param_loader params{};
  params["N"].set(5);
  params["iterations"].set(10);
  params.add_opts(argc, argv);

  const std::vector<Eigen::Vector3d> data{ read_data("/Users/Gary/pracsys/mmmpy_lite/test/walking_test_01.txt") };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  gtsam::noiseModel::Base::shared_ptr R3_noise{ gtsam::noiseModel::Isotropic::Sigma(3, 1) };

  const gtsam::Key kp{ gtsam::Symbol('P', 0) };

  const double dt{ 0.01 };
  const double a{ 0.0 };
  const double b{ data.size() * dt };
  const std::size_t N{ params["N"].as<std::size_t>() };

  PRX_DBG_VARS(N);
  double ti{ 0.0 };

  for (int i = 0; i < data.size(); ++i)
  {
    const gtsam::Key kxi{ gtsam::Symbol('x', i) };
    graph.emplace_shared<R3Manifold>(kp, kxi, R3_noise, N, ti, a, b);
    graph.addPrior(kxi, data[i]);
    initial_values.insert(kxi, data[i]);

    ti += dt;

    // std::cout << i << " " << data[i].transpose() << "\n";
  }
  initial_values.insert(kp, gtsam::ParameterMatrix<3>(N));

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(params["iterations"].as<int>());

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  gtsam::ParameterMatrix<3> param_mat{ result.at<gtsam::ParameterMatrix<3>>(kp) };

  // for (double ti = a; ti < b; ti += 0.01)
  // {
  //   const gtsam::FourierBasis::VectorEvaluationFunctor<3> fx(N, ti, a, b);
  //   std::cout << ti << " " << fx(param_mat).transpose() << "\n";
  // }

  std::ofstream ofs(prx::out_path + "/fourier_example.txt");

  ti = a;
  for (int i = 0; i < data.size(); ++i)
  {
    const gtsam::FourierBasis::VectorEvaluationFunctor<3> fx(N, ti, a, b);
    ofs << i << " " << data[i].transpose() << " ";
    ofs << fx(param_mat).transpose() << " ";
    ofs << "\n";
    ti += dt;
  }
  // PRX_DBG_VARS(pk);
}