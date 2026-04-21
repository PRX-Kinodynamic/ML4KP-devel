#define BOOST_AUTO_TEST_MAIN mat_vec_eigen_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/external/PQP/MatVec.h"
#include "prx/external/PQP/MatVecEigen.hpp"
#include "prx/utilities/defs.hpp"

// Simple comparison that all elements of the matrices are equal (given a tolerance);
inline void compare_pqp_matrices(const PQP_REAL M[3][3], const PQP_EIGEN_MATRIX& Me, const double tolerance = 1e-5)
{
  bool are_similar{ true };
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      are_similar &= std::abs(M[i][j] - Me(i, j)) < tolerance;
    }
  }

  std::stringstream Mss;
  Mss << M[0][0] << " " << M[0][1] << " " << M[0][2] << "\n"   // no-lint
      << M[1][0] << " " << M[1][1] << " " << M[1][2] << "\n"   // no-lint
      << M[2][0] << " " << M[2][1] << " " << M[2][2] << "\n";  // no-lint
  BOOST_REQUIRE_MESSAGE(are_similar, EXPECTED_GOT(Mss.str(), Me));
}

inline void compare_pqp_vectors(const PQP_REAL V[3], const PQP_EIGEN_VECTOR& Ve, const double tolerance = 1e-5)
{
  bool are_similar{ true };
  for (int i = 0; i < 3; ++i)
  {
    are_similar &= std::abs(V[i] - Ve[i]) < tolerance;
  }

  std::stringstream Vss;
  Vss << V[0] << " " << V[1] << " " << V[2] << "\n";  // no-lint
  BOOST_REQUIRE_MESSAGE(are_similar, EXPECTED_GOT(Vss.str(), Ve));
}

BOOST_AUTO_TEST_CASE(Midentity_test)
{
  PQP_REAL M[3][3];
  PQP_EIGEN_MATRIX Me{ Eigen::Matrix3d::Zero() };

  Midentity(M);
  Midentity(Me);

  compare_pqp_matrices(M, Me);
  // BOOST_REQUIRE_MESSAGE(compare_pqp_matrices(M, Me), EXPECTED_GOT(M, Me));
}

BOOST_AUTO_TEST_CASE(Videntity_test)
{
  PQP_REAL V[3];
  PQP_EIGEN_VECTOR Ve;

  Videntity(V);
  Videntity(Ve);

  compare_pqp_vectors(V, Ve);
}

BOOST_AUTO_TEST_CASE(McM_test)
{
  PQP_EIGEN_MATRIX M{ Eigen::Matrix3d::Identity() };
  PQP_EIGEN_MATRIX Mr{ Eigen::Matrix3d::Zero() };

  McM(Mr, M);
  M = M * 2;  // Checking that it was a deep copy

  const Eigen::Matrix3d expected{ Eigen::Matrix3d::Identity() };
  BOOST_REQUIRE_MESSAGE(Mr.isApprox(expected), EXPECTED_GOT(expected, Mr));
}

BOOST_AUTO_TEST_CASE(MTcM_test)
{
  PQP_REAL M[3][3];
  PQP_REAL Mr[3][3];
  Midentity(M);

  PQP_EIGEN_MATRIX Me{ Eigen::Matrix3d::Identity() };
  PQP_EIGEN_MATRIX Mre{ Eigen::Matrix3d::Zero() };

  MTcM(Mr, M);
  MTcM(Mre, Me);

  compare_pqp_matrices(Mr, Mre);
}

BOOST_AUTO_TEST_CASE(VcV_test)
{
  PQP_REAL V[3];
  PQP_REAL Vr[3];
  V[0] = 1;
  V[1] = 1;
  V[2] = 1;
  PQP_EIGEN_VECTOR Ve{ Eigen::Vector3d::Ones() };
  PQP_EIGEN_VECTOR Vre{ Eigen::Vector3d::Zero() };

  VcV(Vr, V);
  VcV(Vre, Ve);

  compare_pqp_vectors(Vr, Vre);
}

BOOST_AUTO_TEST_CASE(McolcV_test)
{
  // TODO
}

BOOST_AUTO_TEST_CASE(McolcMcol_test)
{
  // TODO
}

BOOST_AUTO_TEST_CASE(MxMpV_test)
{
  PQP_EIGEN_MATRIX Mre{ PQP_EIGEN_MATRIX::Zero() };
  PQP_EIGEN_MATRIX M1e{ PQP_EIGEN_MATRIX::Identity() * 2 };
  PQP_EIGEN_MATRIX M2e{ PQP_EIGEN_MATRIX::Identity() * 3 };
  PQP_EIGEN_VECTOR Te{ PQP_EIGEN_VECTOR::Ones() };

  PQP_REAL Mr[3][3];
  PQP_REAL M1[3][3];
  PQP_REAL M2[3][3];
  PQP_REAL T[3];

  M1[0][0] = 2;
  M1[1][1] = 2;
  M1[2][2] = 2;
  M2[0][0] = 3;
  M2[1][1] = 3;
  M2[2][2] = 3;
  T[0] = 1;
  T[1] = 1;
  T[2] = 1;

  MxMpV(Mr, M1, M2, T);
  MxMpV(Mre, M1e, M2e, Te);

  compare_pqp_matrices(Mr, Mre);
}

BOOST_AUTO_TEST_CASE(MxM_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(MxMT_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(MTxM_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(MxV_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(MxVpV_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(sMxVpV_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(MTxV_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(sMTxV_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(sMxV_test)
{
  // TODO
}
BOOST_AUTO_TEST_CASE(VcrossV_test)
{
  // TODO
}
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
// BOOST_AUTO_TEST_CASE(_test)
// {
//   // TODO
// }
