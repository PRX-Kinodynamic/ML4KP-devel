#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"
#include "prx/factor_graphs/factors/constraint_factor.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SOn.h>

BOOST_AUTO_TEST_CASE(constraint_less_than_test)
{
  using Type = double;
  using LessThanFactor = prx::fg::constraint_factor_t<Type, std::less<Type>>;

  const double valid{ 0.5 };
  const double invalid{ -0.5 };
  const double constraint{ 0.0 };

  gtsam::Key k_valid{ 0 };
  gtsam::Key k_invalid{ 1 };

  gtsam::Values values;
  values.insert(k_valid, valid);
  values.insert(k_invalid, invalid);

  gtsam::NonlinearFactorGraph graph_valid;
  gtsam::NonlinearFactorGraph graph_invalid;
  graph_valid.emplace_shared<LessThanFactor>(k_valid, constraint);
  graph_invalid.emplace_shared<LessThanFactor>(k_invalid, constraint);

  // graph.printErrors(values);

  const bool activated_valid{ not graph_valid[0]->active(values) };
  const bool activated_invalid{ graph_invalid[0]->active(values) };

  // Only one should be activated
  BOOST_REQUIRE_MESSAGE(activated_valid, EXPECTED_GOT(1, activated_valid));
  BOOST_REQUIRE_MESSAGE(activated_invalid, EXPECTED_GOT(1, activated_invalid));

  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values result{ gtsam::GaussNewtonOptimizer(graph_invalid, values, params).optimize() };

  const double res_invalid{ result.at<double>(k_invalid) };

  PRX_DBG_VARS(res_invalid);

  // BOOST_REQUIRE_MESSAGE(0.0 <= res_valid, EXPECTED_GOT(0.0, res_valid));
  BOOST_REQUIRE_MESSAGE(0.0 <= res_invalid, EXPECTED_GOT(0.0, res_invalid));
}

BOOST_AUTO_TEST_CASE(constraint_less_than_vector_test)
{
  using Type = Eigen::Vector2d;
  using Cmp = prx::fg::VectorLessThanCmp<Type>;
  using LessThanFactor = prx::fg::constraint_factor_t<Type, Cmp>;

  const Type valid{ 0.5, 0.5 };
  const Type invalid{ -0.5, -0.5 };
  const Type constraint{ 0.0, 0.0 };

  gtsam::Key k_valid{ 0 };
  gtsam::Key k_invalid{ 1 };

  gtsam::Values values;
  values.insert(k_valid, valid);
  values.insert(k_invalid, invalid);

  gtsam::NonlinearFactorGraph graph_valid, graph_invalid;
  graph_valid.emplace_shared<LessThanFactor>(k_valid, constraint);
  graph_invalid.emplace_shared<LessThanFactor>(k_invalid, constraint);

  const bool activated_valid{ not graph_valid[0]->active(values) };
  const bool activated_invalid{ graph_invalid[0]->active(values) };

  // Only one should be activated
  BOOST_REQUIRE_MESSAGE(activated_valid, EXPECTED_GOT("true", activated_valid));
  BOOST_REQUIRE_MESSAGE(activated_invalid, EXPECTED_GOT("true", activated_invalid));

  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values result{ gtsam::GaussNewtonOptimizer(graph_invalid, values, params).optimize() };

  // const Type res_valid{ result.at<Type>(k_valid) };
  const Type res_invalid{ result.at<Type>(k_invalid) };

  // PRX_DBG_VARS(res_valid.transpose());
  PRX_DBG_VARS(res_invalid.transpose());

  // BOOST_REQUIRE_MESSAGE((res_valid.array() > 0.0).all(), EXPECTED_GOT("> 0.0", res_valid));
  BOOST_REQUIRE_MESSAGE((res_invalid.array() > 0.0).all(), EXPECTED_GOT("> 0.0", res_invalid));
  // BOOST_REQUIRE_MESSAGE((res_invalid1.array() > 0.0).all(), EXPECTED_GOT("> 0.0", res_invalid1));
}