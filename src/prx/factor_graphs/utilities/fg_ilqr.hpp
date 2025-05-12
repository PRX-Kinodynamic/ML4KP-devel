#pragma once
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <prx/factor_graphs/factors/prx_propagation_factor.hpp>
#include <prx/factor_graphs/utilities/default_parameters.hpp>
#include <prx/factor_graphs/utilities/dbg_utills.hpp>

namespace prx
{
namespace fg
{
// using GainMap = std::map<gtsam::Key, Eigen::Matrix<double, 1, 4>>;
// using CostToGoMap = std::map<gtsam::Key, Eigen::Matrix<double, 4, 4>>;

Eigen::MatrixXd compute_S(gtsam::GaussianFactorGraph& graph, const gtsam::Key key)
{
  // '''Returns the value function matrix at variable `key` given a graph which
  //     goes up and including `key`, but no further (i.e. all time steps after
  //     `key` have already been eliminated).  Does so by aggregating all unary
  //     factors on `key`.  If value function is x^TPx, then this returns P.
  //     "Return Cost" aka "Cost-to-go" aka "Value Function".
  // Arguments:
  //     graph: factor graph in LTI form
  //     key: key in the factor graph for which we want to obtain the return cost
  // Returns:
  //     return_cost: return cost, an nxn array where `n` is dimension of `key`
  // '''
  gtsam::GaussianFactorGraph new_fg{};
  // graph->print("Original FG (S)", SF::formatter);
  for (std::size_t i = 0; i < graph.size(); ++i)
  {
    auto f = graph.at(i);
    if (f->keys().size() == 1 and f->keys()[0] == key)  // # collect unary factors on `key`
    {
      new_fg.push_back(f);
    }
  }
  // new_fg.print("New FG (S)", SF::formatter);
  auto sol_end = new_fg.eliminateSequential();
  const Eigen::MatrixXd S{ sol_end->back()->information() };

  // PRX_DBG_VARS("Ricatti:", S);
  return S;
}

void compute_K_S(gtsam::GaussianFactorGraph& graph, const gtsam::Ordering& ordering, Eigen::MatrixXd& S,
                 Eigen::MatrixXd& K, Eigen::MatrixXd& R)
{
  try
  {
    auto pair_eliminated = graph.eliminatePartialSequential(ordering);
    graph = *(pair_eliminated.second);
    // Ss[keyX0] = compute_S(graph, keyX0);
    R = pair_eliminated.first->back()->R().matrix();
    S = pair_eliminated.first->back()->S().matrix();
    K = R.triangularView<Eigen::Upper>().solve(S);
    // Ks[keyX0] = K;
  }
  catch (gtsam::IndeterminantLinearSystemException e)
  {
    const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
    prx::fg::indeterminant_linear_system_helper(&graph);
    std::cout << msg << std::string(e.what()) << std::endl;
  }
}

// template <typename Graph, typename Xkeys, typename Ukeys, typename Sout, typename Kout>
// void compute_K_S(Graph& graph, const Xkeys& X, const Ukeys& U, Sout& Ss, Kout& Ks)
// {
//   // def get_k_and_p(graph, X, U):
//   // '''Finds optimal control law given by $u=Kx$ and value function $Vx^2$ aka
//   //     cost-to-go which corresponds to solutions to the algebraic, finite
//   //     horizon Ricatti Equation.  K is Extracted from the bayes net and V is
//   //     extracted by incrementally eliminating the factor graph.  If you only
//   //     need K and not V, then use the `get_k` function below.
//   // Arguments:
//   //     graph: factor graph containing factor graph in LQR form
//   //     X: list of state Keys
//   //     U: list of control Keys
//   // Returns:
//   //     K: optimal control matrix, shape (T-1, 1)
//   //     V: value function, shape (T, 1)
//   //         TODO(gerry): support n-dimensional state space
//   // '''
//   // # Find K and V by using bayes net solution
//   auto marginalized_fg = graph;

//   Ss[X.back()] = compute_S(*marginalized_fg, X.back());
//   // for i in range(len(U)-2, -1, -1): # traverse backwards in time
//   for (int i = U.size() - 1; i > -1; --i)  // # traverse backwards in time
//   {
//     // PRX_DBG_VARS(i);
//     const gtsam::Key keyX1{ X[i + 1] };
//     const gtsam::Key keyX0{ X[i] };
//     const gtsam::Key keyU01{ U[i] };

//     const std::string strKeyX1{ SF::formatter(keyX1) };
//     const std::string strKeyX0{ SF::formatter(keyX0) };
//     const std::string strKeyU01{ SF::formatter(keyU01) };

//     // PRX_DBG_VARS(strKeyX1, strKeyX0, strKeyU01);
//     gtsam::Ordering ordering{};
//     ordering.push_back(keyX1);
//     ordering.push_back(keyU01);

//     try
//     {
//       // std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
//       // bayes_net, marginalized_fg = marginalized_fg.eliminatePartialSequential(ordering);
//       auto pair_eliminated = marginalized_fg->eliminatePartialSequential(ordering);
//       marginalized_fg = pair_eliminated.second;
//       // pair_eliminated.first->print("pair_eliminated", SF::formatter);
//       // marginalized_fg->print("marginalized_fg", SF::formatter);
//       Ss[keyX0] = compute_S(*marginalized_fg, keyX0);
//       const Eigen::MatrixXd R{ pair_eliminated.first->back()->R().matrix() };
//       const Eigen::MatrixXd S{ pair_eliminated.first->back()->S().matrix() };
//       const Eigen::MatrixXd K{ R.triangularView<Eigen::Upper>().solve(S) };
//       // PRX_DBG_VARS(strKeyX0);
//       // PRX_DBG_VARS(Ss[keyX0]);
//       // PRX_DBG_VARS(R);
//       // PRX_DBG_VARS(S);
//       // PRX_DBG_VARS(K);
//       Ks[keyX0] = K;
//     }
//     catch (gtsam::IndeterminantLinearSystemException e)
//     {
//       const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.nearbyVariable()) + "\n" };
//       prx::fg::indeterminant_linear_system_helper(marginalized_fg);
//       std::cout << msg << std::string(e.what()) << std::endl;
//     }
//   }
// }

// template <typename Xkeys, typename Ukeys, typename Sout, typename Kout>
// void compute_K_S(gtsam::NonlinearFactorGraph& graph, gtsam::Values& values, const Xkeys& xks, const Ukeys& uks,
//                  Sout& Ss, Kout& Ks)
// {
//   gtsam::GaussianFactorGraph linearized_graph{ graph.linearize(values) };
//   compute_K_S(linearized_graph, xks, uks, Ss, Ks);
// }

}  // namespace fg
}  // namespace prx
