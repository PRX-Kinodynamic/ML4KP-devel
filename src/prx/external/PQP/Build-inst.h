#pragma once
#include "MatVecEigen.hpp"

inline void s2_accum(PQP_REAL S2[3][3], const PQP_REAL p1[3], const PQP_REAL p2[3], const PQP_REAL p3[3])
{
  S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
  S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
  S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);

  S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
  S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
  S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
}

inline void s2_accum(Eigen::Ref<PQP_EIGEN_MATRIX> S2, const PQP_EIGEN_VECTOR& p1, const PQP_EIGEN_VECTOR& p2,
                     const PQP_EIGEN_VECTOR& p3)
{
  S2(0, 0) += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
  S2(1, 1) += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
  S2(2, 2) += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);

  S2(0, 1) += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
  S2(0, 2) += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
  S2(1, 2) += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
}

inline void compute_covariance(PQP_REAL Mr[3][3], PQP_REAL S2[3][3], PQP_REAL S1[3], const PQP_REAL& n)
{
  Mr[0][0] = S2[0][0] - S1[0] * S1[0] / n;
  Mr[1][1] = S2[1][1] - S1[1] * S1[1] / n;
  Mr[2][2] = S2[2][2] - S1[2] * S1[2] / n;
  Mr[0][1] = S2[0][1] - S1[0] * S1[1] / n;
  Mr[1][2] = S2[1][2] - S1[1] * S1[2] / n;
  Mr[0][2] = S2[0][2] - S1[0] * S1[2] / n;
  Mr[1][0] = Mr[0][1];
  Mr[2][0] = Mr[0][2];
  Mr[2][1] = Mr[1][2];
}

inline void compute_covariance(Eigen::Ref<PQP_EIGEN_MATRIX> Mr, Eigen::Ref<PQP_EIGEN_MATRIX> S2,
                               const Eigen::Ref<PQP_EIGEN_VECTOR> S1, const PQP_REAL& n)
{
  Mr(0, 0) = S2(0, 0) - S1[0] * S1[0] / n;
  Mr(1, 1) = S2(1, 1) - S1[1] * S1[1] / n;
  Mr(2, 2) = S2(2, 2) - S1[2] * S1[2] / n;
  Mr(0, 1) = S2(0, 1) - S1[0] * S1[1] / n;
  Mr(1, 2) = S2(1, 2) - S1[1] * S1[2] / n;
  Mr(0, 2) = S2(0, 2) - S1[0] * S1[2] / n;
  Mr(1, 0) = Mr(0, 1);
  Mr(2, 0) = Mr(0, 2);
  Mr(2, 1) = Mr(1, 2);
}

template <typename Rotation, typename Translation>
void get_covariance_triverts(Rotation& M, TriBase<Translation>* tris, int num_tris)
{
  int i;
  Translation S1;
  Rotation S2;

  Videntity(S1);
  Mzero(S2);

  // S1[0] = S1[1] = S1[2] = 0.0;
  // S2[0][0] = S2[1][0] = S2[2][0] = 0.0;
  // S2[0][1] = S2[1][1] = S2[2][1] = 0.0;
  // S2[0][2] = S2[1][2] = S2[2][2] = 0.0;

  // get center of mass
  for (i = 0; i < num_tris; i++)
  {
    const Translation& p1{ tris[i].p1 };
    const Translation& p2{ tris[i].p2 };
    const Translation& p3{ tris[i].p3 };

    S1[0] += p1[0] + p2[0] + p3[0];
    S1[1] += p1[1] + p2[1] + p3[1];
    S1[2] += p1[2] + p2[2] + p3[2];

    s2_accum(S2, p1, p2, p3);

    // S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
    // S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
    // S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);

    // S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
    // S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
    // S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
  }

  PQP_REAL n = (PQP_REAL)(3 * num_tris);

  // now get covariances
  compute_covariance(M, S2, S1, n);

  // M[0][0] = S2[0][0] - S1[0] * S1[0] / n;
  // M[1][1] = S2[1][1] - S1[1] * S1[1] / n;
  // M[2][2] = S2[2][2] - S1[2] * S1[2] / n;
  // M[0][1] = S2[0][1] - S1[0] * S1[1] / n;
  // M[1][2] = S2[1][2] - S1[1] * S1[2] / n;
  // M[0][2] = S2[0][2] - S1[0] * S1[2] / n;
  // M[1][0] = M[0][1];
  // M[2][0] = M[0][2];
  // M[2][1] = M[1][2];
}
