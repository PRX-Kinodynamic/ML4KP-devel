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
#include <gtsam/basis/Basis.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using Sequence = std::map<double, double>;
using Sample = std::pair<double, double>;
using Weights = Eigen::Matrix<double, 1, -1>; /* 1xN vector */
using CsvReader = prx::utilities::csv_reader_t;

std::vector<gtsam::Pose3> read_data(const std::string filename)
{
  using prx::utilities::convert_to;
  std::vector<gtsam::Pose3> data;
  CsvReader reader(filename, ' ');
  while (reader.has_next_line())
  {
    auto line = reader.next_line();
    if (line.size() == 0)
      break;
    if (line[0] == "#")
      continue;
    const double xi{ convert_to<double>(line[1]) };
    const double yi{ convert_to<double>(line[2]) };
    const double zi{ convert_to<double>(line[3]) };

    const double qwi{ convert_to<double>(line[4]) };
    const double qxi{ convert_to<double>(line[5]) };
    const double qyi{ convert_to<double>(line[6]) };
    const double qzi{ convert_to<double>(line[7]) };

    const Eigen::Vector3d t(xi, yi, zi);
    const Eigen::Quaterniond q(qwi, qxi, qyi, qzi);

    const gtsam::Rot3 rot(q);
    // const gtsam::Pose3 pose(rot, t);

    data.emplace_back(rot, t);
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
  const typename Basis::template ManifoldEvaluationFunctor<Type> _evaluation_function;

  mutable JacobianXX err_H_b;  // Deriv error wrt between

  mutable JacobianXX b_H_x;   // Deriv between wrt x
  mutable JacobianXX b_H_xp;  // Deriv between wrt x_{predicted}

  mutable Eigen::MatrixXd dxp_H_P;  // Deriv \dot{predicted} wrt Params
};

// Factor to "close" the function: f(0) = f(T) which means f(a) == f(b)
template <class Basis, typename Type>
class manifold_close_factor_t : public gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<gtsam::traits<Type>::dimension>>
{
  static constexpr Eigen::Index Dim{ gtsam::traits<Type>::dimension };

  using Base = gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<Dim>>;
  using Derived = manifold_close_factor_t<Basis, Type>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using JacobianXX = Eigen::Matrix<double, Dim, Dim>;
  using JacobianXP = Eigen::Matrix<double, Dim, -1>;

  using ParameterMatrix = gtsam::ParameterMatrix<Dim>;

  using EvalFunction = typename Basis::template ManifoldEvaluationFunctor<Type>;

public:
  manifold_close_factor_t(const gtsam::Key keyPolyParams, const NoiseModel& cost_model, const size_t N, double a,
                          double b)
    : Base(cost_model, keyPolyParams), _eval_func_0(N, a, a, b), _eval_func_T(N, b, a, b)
  {
  }

  virtual Eigen::VectorXd evaluateError(const ParameterMatrix& P, OptDeriv Hp = boost::none) const override
  {
    using TypeTraits = gtsam::traits<Type>;

    const Type x0{ _eval_func_0(P, Hp ? &x0_H_P : nullptr) };
    const Type xT{ _eval_func_T(P, Hp ? &xT_H_P : nullptr) };

    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const Type between{ TypeTraits::Between(x0, xT,                  // no-lint
                                            Hp ? &b_H_x0 : nullptr,  // no-lint
                                            Hp ? &b_H_xT : nullptr) };
    const Eigen::VectorXd error{ TypeTraits::Logmap(between, Hp ? &err_H_b : nullptr) };

    if (Hp)
    {
      *Hp = err_H_b * (b_H_x0 * x0_H_P + b_H_xT * xT_H_P);
    }

    // PRX_DBG_VARS(x0);
    // PRX_DBG_VARS(xT);
    // PRX_DBG_VARS(error);
    return error;
  }

private:
  const EvalFunction _eval_func_0;
  const EvalFunction _eval_func_T;

  mutable JacobianXX err_H_b;  // Deriv error wrt between

  mutable JacobianXX b_H_x0;  // Deriv between wrt x0
  mutable JacobianXX b_H_xT;  // Deriv between wrt xT

  mutable Eigen::MatrixXd x0_H_P;  // Deriv x0 wrt Poly
  mutable Eigen::MatrixXd xT_H_P;  // Deriv xT wrt Poly

  // mutable Eigen::MatrixXd dxp_H_P;  // Deriv \dot{predicted} wrt Params
};

// template <class Basis, typename Type>
// class close_deriv_factor_t : public gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<gtsam::traits<Type>::dimension>>
// {
//   static constexpr Eigen::Index Dim{ gtsam::traits<Type>::dimension };

//   using Base = gtsam::NoiseModelFactorN<gtsam::ParameterMatrix<Dim>>;
//   using Derived = close_deriv_factor_t<Basis, Type>;
//   using OptDeriv = boost::optional<Eigen::MatrixXd&>;
//   using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

//   using JacobianXX = Eigen::Matrix<double, Dim, Dim>;
//   using JacobianXP = Eigen::Matrix<double, Dim, -1>;

//   using ParameterMatrix = gtsam::ParameterMatrix<Dim>;

//   using EvalFunction = typename Basis::template ManifoldEvaluationFunctor<Type>;
//   using VectorDim = Eigen::Matrix<double, Dim, 1>;
//   // using ManifoldCloseFactor = manifold_close_factor_t<Basis, Type>;
//   // using VecEvalFunction = typename Basis::template VectorEvaluationFunctor<Dim>;

// public:
//   close_deriv_factor_t(const gtsam::Key keyPolyParams, const NoiseModel& cost_model, const size_t N, double a, double
//   b,
//                        const double h)
//     : Base(cost_model, keyPolyParams)
//     , _eval_func_0(N, a, a, b)
//     , _eval_func_T(N, b, a, b)
//     , err_H_xdT(Eigen::Matrix<double, Dim, Dim>::Identity())
//     , err_H_xd0(-1 * Eigen::Matrix<double, Dim, Dim>::Identity())
//     , _epsilon(VectorDim::Ones() * h)
//   {
//   }

//   virtual Eigen::VectorXd evaluateError(const ParameterMatrix& P, OptDeriv Hp = boost::none) const override
//   {
//     using TypeTraits = gtsam::traits<Type>;

//     const Type x0{ _eval_func_0(P, &x0_H_P : nullptr) };
//     const Type xT{ _eval_func_T(P, &xT_H_P : nullptr) };

//     // (Dim x Dim) (Dim x 1)
//     const VectorDim x_eps0{ x0_H_P * _epsilon };
//     const VectorDim x_epsT{ xT_H_P * _epsilon };
//     // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
//     // const Type between{ TypeTraits::Between(x0, xT,                  // no-lint
//     //                                         Hp ? &b_H_x0 : nullptr,  // no-lint
//     //                                         Hp ? &b_H_xT : nullptr) };
//     // const Eigen::VectorXd error{ TypeTraits::Logmap(between, Hp ? &err_H_b : nullptr) };
//     const Eigen::VectorXd error{ x_eps0 - x_epsT };

//     if (Hp)
//     {
//       *Hp = err_H_b * (b_H_x0 * x0_H_P + b_H_xT * xT_H_P);
//     }

//     // PRX_DBG_VARS(x0);
//     // PRX_DBG_VARS(xT);
//     // PRX_DBG_VARS(error);
//     return error;
//   }

// private:
//   // const ManifoldCloseFactor _manifold_factor;
//   const VectorDim _epsilon;

//   const EvalFunction _eval_func_0;
//   const EvalFunction _eval_func_T;

//   mutable JacobianXX err_H_xdT;  // Deriv between wrt x0
//   mutable JacobianXX err_H_xd0;  // Deriv between wrt xT

//   mutable Eigen::MatrixXd x0_H_P;  // Deriv x0 wrt Poly
//   mutable Eigen::MatrixXd xT_H_P;  // Deriv xT wrt Poly

//   // mutable Eigen::MatrixXd dxp_H_P;  // Deriv \dot{predicted} wrt Params
// };
//
Eigen::MatrixXd DifferentiationMatrix(const size_t N)
{
  Eigen::MatrixXd D{ Eigen::MatrixXd::Zero(N + 1, N + 1) };
  double k = 1;
  for (size_t i = 1; i < N; i += 2)
  {
    PRX_DBG_VARS(N, i);
    D(i, i + 1) = k;   // sin'(k*x) = k*cos(k*x)
    D(i + 1, i) = -k;  // cos'(k*x) = -k*sin(k*x)
    k += 1;
  }
  return D;
}

template <typename Stream>
void pose3_to_stream(Stream& stream, const gtsam::Pose3& pose)
{
  const Eigen::Vector3d t{ pose.translation() };
  const Eigen::Quaterniond q{ pose.rotation().toQuaternion() };

  stream << t.transpose() << " ";
  stream << q.w() << " ";
  stream << q.x() << " ";
  stream << q.y() << " ";
  stream << q.z() << " ";
}

int main(int argc, char* argv[])
{
  using SE3Manifold = manifold_evaluation_t<gtsam::FourierBasis, gtsam::Pose3>;
  using SE3CloseFactor = manifold_close_factor_t<gtsam::FourierBasis, gtsam::Pose3>;
  using FourierManifoldFunctor = gtsam::FourierBasis::ManifoldEvaluationFunctor<gtsam::Pose3>;
  // using FourierLieAlgebraFunction = gtsam::FourierBasis::VectorEvaluationFunctor<6>;
  using FourierLieAlgebraFunction = gtsam::FourierBasis::VectorDerivativeFunctor<6>;
  // using CloseDerivFactor = close_deriv_factor_t<gtsam::FourierBasis, gtsam::Pose3>;

  prx::param_loader params{};
  params["N"].set(5);
  params["iterations"].set(10);
  params["filename"].set("/Users/Gary/pracsys/mmmpy_lite/test/walking_test_01.txt");
  params.add_opts(argc, argv);

  const std::string filename{ params["filename"].as<>() };
  std::vector<gtsam::Pose3> data{ read_data(filename) };

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  gtsam::noiseModel::Base::shared_ptr SE3_noise{ gtsam::noiseModel::Isotropic::Sigma(6, 1) };
  gtsam::noiseModel::Base::shared_ptr equal_nm{ gtsam::noiseModel::Isotropic::Sigma(6, 1e-5) };

  const gtsam::Key kp{ gtsam::Symbol('P', 0) };

  const double dt{ 0.01 };
  const double a{ 0.0 };
  const double b{ data.size() * dt };
  const std::size_t N{ params["N"].as<std::size_t>() };

  PRX_DBG_VARS(a, b, N);
  double ti{ 0.0 };

  for (int i = 0; i < data.size(); ++i)
  {
    const gtsam::Key kxi{ gtsam::Symbol('x', i) };
    graph.emplace_shared<SE3Manifold>(kp, kxi, SE3_noise, N, ti, a, b);
    graph.addPrior(kxi, data[i]);
    initial_values.insert(kxi, data[i]);

    ti += dt;

    // std::cout << i << " " << data[i].transpose() << "\n";
  }
  // graph.emplace_shared<SE3CloseFactor>(kp, equal_nm, N, a, b);
  // graph.emplace_shared<SE3CloseFactor>(kp, equal_nm, N, a + 3 * dt, b - 3 * dt);
  // graph.emplace_shared<CloseDerivFactor>(kp, equal_nm, N, a, b);

  // manifold_close_factor_t(const gtsam::Key keyPolyParams, const NoiseModel& cost_model, const size_t N, double a,
  //                         double b)
  initial_values.insert(kp, gtsam::ParameterMatrix<6>(N));

  gtsam::LevenbergMarquardtParams lm_params{ prx::fg::default_levenberg_marquardt_parameters() };
  lm_params.setMaxIterations(params["iterations"].as<int>());

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, lm_params);
  gtsam::Values result{ optimizer.optimize() };

  gtsam::ParameterMatrix<6> param_mat{ result.at<gtsam::ParameterMatrix<6>>(kp) };

  // for (double ti = a; ti < b; ti += 0.01)
  // {
  //   const gtsam::FourierBasis::VectorEvaluationFunctor<3> fx(N, ti, a, b);
  //   std::cout << ti << " " << fx(param_mat).transpose() << "\n";
  // }

  std::ofstream ofs(prx::out_path + "/fourier_manifold.txt");
  std::ofstream ofs_poly(prx::out_path + "/polynomial_trajectory.txt");

  ti = a;
  data.push_back(data.back());

  // Eigen::Matrix<double, 6, 1> eps{ Eigen::Matrix<double, 6, 1>::Ones() * 0.1 };

  // Eigen::MatrixXd D7{ DifferentiationMatrix(7) };
  // PRX_DBG_VARS(D7);

  // Eigen::MatrixXd Dn{ DifferentiationMatrix(N) };
  // PRX_DBG_VARS(Dn.rows(), Dn.cols());
  // static DiffMatrix (size_t N) {

  // using VectorM = Eigen::Matrix<double, 6, 1>;
  PRX_DBG_VARS(param_mat.rows(), param_mat.cols())
  Eigen::MatrixXd Dm{ gtsam::FourierBasis::DifferentiationMatrix(param_mat.rows() + 1) };
  PRX_DBG_VARS(Dm)
  Eigen::MatrixXd derivative{ Dm * param_mat.matrix() };
  // gtsam::ParameterMatrix<6> derivativeCoefficients{ derivative };  // = Eigen::MatrixXd(Dn * param_mat.matrix());

  ofs_poly << "# x y z qw qx qy qz xd yd zd Rxd Ryd Rzd \n";
  for (int i = 0; i < data.size(); ++i)
  {
    // PRX_DBG_VARS(ti, a, b);
    Eigen::MatrixXd Hfx;

    const FourierLieAlgebraFunction fla(N, ti, a, b);
    const FourierManifoldFunctor fx(N, ti, a, b);
    const gtsam::Pose3 in_pose{ data[i] };
    const gtsam::Pose3 f_pose{ fx(param_mat, Hfx) };

    gtsam::FourierBasis::VectorEvaluationFunctor<6> dfdx(N, ti, a, b);

    gtsam::Weights wi{ gtsam::FourierBasis::CalculateWeights(N, ti, a, b) };

    // gtsam::FourierBasis::VectorDerivativeFunctor<6> dfdx(N, ti, a, b);

    // const Eigen::Matrix<double, 6, 1> xi{ Hfx * eps };
    // const Eigen::Matrix<double, 6, 1> xi{ fla(param_mat) };
    // const Eigen::Matrix<double, 6, 1> xi{ dfdx(derivativeCoefficients) };
    const Eigen::Matrix<double, 6, 1> xi{ derivative * wi };
    // const gtsam::Pose3 result{ gtsam::Pose3::ChartAtOrigin::Retract(xi) };
    // const bool pose_eqs{ result.equals(f_pose, 1e-5) };
    // PRX_DBG_VARS(pose_eqs);

    // const Eigen::Matrix<double, 6, 1> xi{ Dn * fla(param_mat) };
    // ofs << i << " " << data[i] << " ";
    // ofs << fx(param_mat) << " ";
    ofs_poly << i << " ";
    pose3_to_stream(ofs_poly, f_pose);
    ofs_poly << xi.transpose() << " ";
    // ofs_poly << xi.tail(3).transpose() << " ";
    // ofs_poly << xi.head(3).transpose() << " ";
    ofs_poly << "\n";

    ofs << i << " ";
    pose3_to_stream(ofs, in_pose);
    pose3_to_stream(ofs, f_pose);
    ofs << "\n";
    ti += dt;
  }
  ofs_poly.close();

  const FourierManifoldFunctor fa(N, a, a, b);
  const FourierManifoldFunctor fb(N, b, a, b);
  const gtsam::Pose3 xa{ fa(param_mat) };
  const gtsam::Pose3 xb{ fb(param_mat) };
  PRX_DBG_VARS(xa)
  PRX_DBG_VARS(xb)

  // PRX_DBG_VARS(pk);
}