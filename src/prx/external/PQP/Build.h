/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Gottschalk, E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#ifndef PQP_BUILD_H
#define PQP_BUILD_H

#include "PQP.h"
#include "Build-inst.h"

template <typename Translation>
int split_tris(TriBase<Translation>* tris, const int num_tris, Translation& a, const PQP_REAL c)
{
  int i;
  int c1 = 0;
  Translation p;
  PQP_REAL x;
  TriBase<Translation> temp;

  for (i = 0; i < num_tris; i++)
  {
    // loop invariant: up to (but not including) index c1 in group 1,
    // then up to (but not including) index i in group 2
    //
    //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
    //                   c1          i
    // //
    Translation& p1{ tris[i].p1 };
    Translation& p2{ tris[i].p2 };
    Translation& p3{ tris[i].p3 };
    VcV(p, p1);
    VpV(p, p, p2);
    VpV(p, p, p3);
    x = VdotV(p, a);
    x /= 3.0;
    if (x <= c)
    {
      // group 1
      temp = tris[i];
      tris[i] = tris[c1];
      tris[c1] = temp;
      c1++;
    }
    else
    {
      // group 2 -- do nothing
    }
  }

  // split arbitrarily if one group empty

  if ((c1 == 0) || (c1 == num_tris))
    c1 = num_tris / 2;

  return c1;
}

template <typename Translation>
void get_centroid_triverts(Translation& c, TriBase<Translation>* tris, int num_tris)
{
  int i;

  c[0] = c[1] = c[2] = 0.0;

  // get center of mass
  for (i = 0; i < num_tris; i++)
  {
    const Translation& p1{ tris[i].p1 };
    const Translation& p2{ tris[i].p2 };
    const Translation& p3{ tris[i].p3 };

    c[0] += p1[0] + p2[0] + p3[0];
    c[1] += p1[1] + p2[1] + p3[1];
    c[2] += p1[2] + p2[2] + p3[2];
  }

  PQP_REAL n = (PQP_REAL)(3 * num_tris);

  c[0] /= n;
  c[1] /= n;
  c[2] /= n;
}

// template <typename Rotation, typename Translation>
// void get_covariance_triverts(Rotation& M, TriBase<Translation>* tris, int num_tris)
// {
//   int i;
//   Translation S1;
//   PQP_REAL S2[3][3];

//   S1[0] = S1[1] = S1[2] = 0.0;
//   S2[0][0] = S2[1][0] = S2[2][0] = 0.0;
//   S2[0][1] = S2[1][1] = S2[2][1] = 0.0;
//   S2[0][2] = S2[1][2] = S2[2][2] = 0.0;

//   // get center of mass
//   for (i = 0; i < num_tris; i++)
//   {
//     PQP_REAL* p1 = tris[i].p1;
//     PQP_REAL* p2 = tris[i].p2;
//     PQP_REAL* p3 = tris[i].p3;

//     S1[0] += p1[0] + p2[0] + p3[0];
//     S1[1] += p1[1] + p2[1] + p3[1];
//     S1[2] += p1[2] + p2[2] + p3[2];

//     S2[0][0] += (p1[0] * p1[0] + p2[0] * p2[0] + p3[0] * p3[0]);
//     S2[1][1] += (p1[1] * p1[1] + p2[1] * p2[1] + p3[1] * p3[1]);
//     S2[2][2] += (p1[2] * p1[2] + p2[2] * p2[2] + p3[2] * p3[2]);
//     S2[0][1] += (p1[0] * p1[1] + p2[0] * p2[1] + p3[0] * p3[1]);
//     S2[0][2] += (p1[0] * p1[2] + p2[0] * p2[2] + p3[0] * p3[2]);
//     S2[1][2] += (p1[1] * p1[2] + p2[1] * p2[2] + p3[1] * p3[2]);
//   }

//   PQP_REAL n = (PQP_REAL)(3 * num_tris);

//   // now get covariances

//   M[0][0] = S2[0][0] - S1[0] * S1[0] / n;
//   M[1][1] = S2[1][1] - S1[1] * S1[1] / n;
//   M[2][2] = S2[2][2] - S1[2] * S1[2] / n;
//   M[0][1] = S2[0][1] - S1[0] * S1[1] / n;
//   M[1][2] = S2[1][2] - S1[1] * S1[2] / n;
//   M[0][2] = S2[0][2] - S1[0] * S1[2] / n;
//   M[1][0] = M[0][1];
//   M[2][0] = M[0][2];
//   M[2][1] = M[1][2];
// }

template <typename PQPModelType, typename Rotation, typename Translation>
void make_parent_relative(PQPModelType* m, int bn, Rotation& parentR
#if PQP_BV_TYPE & RSS_TYPE
                          ,
                          Translation& parentTr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                          ,
                          Translation& parentTo
#endif
)
{
  Rotation Rpc;
  Translation Tpc;

  if (!m->child(bn)->Leaf())
  {
    // make children parent-relative

    make_parent_relative(m, m->child(bn)->first_child, m->child(bn)->R
#if PQP_BV_TYPE & RSS_TYPE
                         ,
                         m->child(bn)->Tr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                         ,
                         m->child(bn)->To
#endif
    );
    make_parent_relative(m, m->child(bn)->first_child + 1, m->child(bn)->R
#if PQP_BV_TYPE & RSS_TYPE
                         ,
                         m->child(bn)->Tr
#endif
#if PQP_BV_TYPE & OBB_TYPE
                         ,
                         m->child(bn)->To
#endif
    );
  }

  // make self parent relative

  MTxM(Rpc, parentR, m->child(bn)->R);
  McM(m->child(bn)->R, Rpc);
#if PQP_BV_TYPE & RSS_TYPE
  VmV(Tpc, m->child(bn)->Tr, parentTr);
  MTxV(m->child(bn)->Tr, parentR, Tpc);
#endif
#if PQP_BV_TYPE & OBB_TYPE
  VmV(Tpc, m->child(bn)->To, parentTo);
  MTxV(m->child(bn)->To, parentR, Tpc);
#endif
}
template <typename PQPModelType>
int build_recurse(PQPModelType* m, int bn, int first_tri, int num_tris)
{
  using Rotation = typename PQPModelType::RotationType;
  using Translation = typename PQPModelType::TranslationType;

  BVBase<Rotation, Translation>* b = m->child(bn);

  // compute a rotation matrix

  // PQP_REAL C[3][3], E[3][3], R[3][3], s[3], axis[3], mean[3], coord;
  Rotation C, E, R;
  Translation s, axis, mean;
  PQP_REAL coord;

#if RAPID2_FIT
  moment* tri_moment = new moment[num_tris];
  compute_moments(tri_moment, &(m->tris[first_tri]), num_tris);
  accum acc;
  clear_accum(acc);
  for (int i = 0; i < num_tris; i++)
    accum_moment(acc, tri_moment[i]);
  delete[] tri_moment;
  covariance_from_accum(C, acc);
#else
  get_covariance_triverts(C, &m->tris[first_tri], num_tris);
#endif

  Meigen(E, s, C);

  // place axes of E in order of increasing s

  int min, mid, max;
  if (s[0] > s[1])
  {
    max = 0;
    min = 1;
  }
  else
  {
    min = 0;
    max = 1;
  }
  if (s[2] < s[min])
  {
    mid = min;
    min = 2;
  }
  else if (s[2] > s[max])
  {
    mid = max;
    max = 2;
  }
  else
  {
    mid = 2;
  }
  McolcMcol(R, 0, E, max);
  McolcMcol(R, 1, E, mid);
  axis_placement(R, E, max, mid);
  // R[0][2] = E[1][max] * E[2][mid] - E[1][mid] * E[2][max];
  // R[1][2] = E[0][mid] * E[2][max] - E[0][max] * E[2][mid];
  // R[2][2] = E[0][max] * E[1][mid] - E[0][mid] * E[1][max];

  // fit the BV

  b->FitToTris(R, &m->tris[first_tri], num_tris);

  if (num_tris == 1)
  {
    // BV is a leaf BV - first_child will index a triangle

    b->first_child = -(first_tri + 1);
  }
  else if (num_tris > 1)
  {
    // BV not a leaf - first_child will index a BV

    b->first_child = m->num_bvs;
    m->num_bvs += 2;

    // choose splitting axis and splitting coord

    McolcV(axis, R, 0);

#if RAPID2_FIT
    mean_from_accum(mean, acc);
#else
    get_centroid_triverts(mean, &m->tris[first_tri], num_tris);
#endif
    coord = VdotV(axis, mean);

    // now split

    int num_first_half = split_tris(&m->tris[first_tri], num_tris, axis, coord);

    // recursively build the children

    build_recurse(m, m->child(bn)->first_child, first_tri, num_first_half);
    build_recurse(m, m->child(bn)->first_child + 1, first_tri + num_first_half, num_tris - num_first_half);
  }
  return PQP_OK;
}

template <typename PQPModelType>
int build_model(PQPModelType* m, bool parent_relative = true)
{
  using Rotation = typename PQPModelType::RotationType;
  using Translation = typename PQPModelType::TranslationType;
  // set num_bvs to 1, the first index for a child bv

  m->num_bvs = 1;

  // build recursively

  build_recurse(m, 0, 0, m->num_tris);

  if (parent_relative)
  {
    // change BV orientations from world-relative to parent-relative

    Rotation R;
    Translation T;
    // PQP_REAL R[3][3], T[3];
    Midentity(R);
    Videntity(T);

    make_parent_relative(m, 0, R
#if PQP_BV_TYPE & RSS_TYPE
                         ,
                         T
#endif
#if PQP_BV_TYPE & OBB_TYPE
                         ,
                         T
#endif
    );
  }

  return PQP_OK;
}

#endif
