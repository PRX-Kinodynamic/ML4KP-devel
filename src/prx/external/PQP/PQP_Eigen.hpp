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

#ifndef PQP_EIGEN_H
#define PQP_EIGEN_H

#include "PQP.h"
#include "GetTime.h"

//----------------------------------------------------------------------------
//
//  PQP_Collide() - detects collision between two PQP_Models
//
//
//  Declare a PQP_CollideResult struct and pass its pointer to collect
//  collision data.
//
//  [R1, T1] is the placement of model 1 in the world &
//  [R2, T2] is the placement of model 2 in the world.
//  The columns of each 3x3 matrix are the basis vectors for the model
//  in world coordinates, and the matrices are in row-major order:
//  R(row r, col c) = R[r][c].
//
//  If PQP_ALL_CONTACTS is the flag value, after calling PQP_Collide(),
//  the PQP_CollideResult object will contain an array with all
//  colliding triangle pairs. Suppose CR is a pointer to the
//  PQP_CollideResult object.  The number of pairs is gotten from
//  CR->NumPairs(), and the ids of the 15'th pair of colliding
//  triangles is gotten from CR->Id1(14) and CR->Id2(14).
//
//  If PQP_FIRST_CONTACT is the flag value, the PQP_CollideResult array
//  will only get the first colliding triangle pair found.  Thus
//  CR->NumPairs() will be at most 1, and if 1, CR->Id1(0) and
//  CR->Id2(0) give the ids of the colliding triangle pair.
//
//----------------------------------------------------------------------------

// struct PQP_EIGEN_CollideResult : public PQP_CollideResult
// {
//   PQP_EIGEN_MATRIX rotation;
//   PQP_EIGEN_VECTOR translation;
// };
using PQP_CollideResult_Eigen = PQP_CollideResult_Base<PQP_EIGEN_MATRIX, PQP_EIGEN_VECTOR>;
using PQP_Model_Eigen = PQP_Model_Base<PQP_EIGEN_MATRIX, PQP_EIGEN_VECTOR>;
using PQP_DistanceResult_Eigen = PQP_DistanceResult_Base<PQP_EIGEN_MATRIX, PQP_EIGEN_VECTOR>;

template <typename PQPResultPtr, typename PQPModelPtr>
int PQP_Collide(PQPResultPtr result, Eigen::Ref<PQP_EIGEN_MATRIX> R1, Eigen::Ref<PQP_EIGEN_VECTOR> T1, PQPModelPtr o1,
                Eigen::Ref<PQP_EIGEN_MATRIX> R2, Eigen::Ref<PQP_EIGEN_VECTOR> T2, PQPModelPtr o2,
                int flag = PQP_ALL_CONTACTS)
{
  const double t1{ GetTime() };

  // make sure that the models are built

  if (o1->build_state != PQP_BUILD_STATE_PROCESSED)
    return PQP_ERR_UNPROCESSED_MODEL;
  if (o2->build_state != PQP_BUILD_STATE_PROCESSED)
    return PQP_ERR_UNPROCESSED_MODEL;

  // clear the stats

  result->num_bv_tests = 0;
  result->num_tri_tests = 0;

  // don't release the memory, but reset the num_pairs counter

  result->num_pairs = 0;

  // Okay, compute what transform [R,T] that takes us from cs1 to cs2.
  // [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  // First compute the rotation part, then translation part

  MTxM(result->R, R1, R2);
  PQP_EIGEN_VECTOR Ttemp;

  VmV(Ttemp, T2, T1);
  MTxV(result->T, R1, Ttemp);

  // compute the transform from o1->child(0) to o2->child(0)

  PQP_EIGEN_MATRIX Rtemp;
  PQP_EIGEN_MATRIX R;
  PQP_EIGEN_VECTOR T;
  // PQP_REAL Rtemp[3][3], R[3][3], T[3];

  MxM(Rtemp, result->R, o2->child(0)->R);
  MTxM(R, o1->child(0)->R, Rtemp);

#if PQP_BV_TYPE & OBB_TYPE
  MxVpV(Ttemp, result->R, o2->child(0)->To, result->T);
  VmV(Ttemp, Ttemp, o1->child(0)->To);
#else
  MxVpV(Ttemp, result->R, o2->child(0)->Tr, result->T);
  VmV(Ttemp, Ttemp, o1->child(0)->Tr);
#endif

  MTxV(T, o1->child(0)->R, Ttemp);

  // now start with both top level BVs

  CollideRecurse(result, R, T, o1, 0, o2, 0, flag);

  const double t2{ GetTime() };
  result->query_time_secs = t2 - t1;

  return PQP_OK;
}

#if PQP_BV_TYPE & RSS_TYPE  // this is true by default,
                            // and explained in PQP_Compile.h

//----------------------------------------------------------------------------
//
//  PQP_DistanceResult
//
//  This saves and reports results from a distance query.
//
//----------------------------------------------------------------------------
//
//  struct PQP_DistanceResult - declaration contained in PQP_Internal.h
//  {
//    // statistics
//
//    int NumBVTests();
//    int NumTriTests();
//    PQP_REAL QueryTimeSecs();
//
//    // The following distance and points established the minimum distance
//    // for the models, within the relative and absolute error bounds
//    // specified.
//
//    PQP_REAL Distance();
//    const PQP_REAL *P1();  // pointers to three PQP_REALs
//    const PQP_REAL *P2();
//  };

//----------------------------------------------------------------------------
//
//  PQP_Distance() - computes the distance between two PQP_Models
//
//
//  Declare a PQP_DistanceResult struct and pass its pointer to collect
//  distance information.
//
//  "rel_err" is the relative error margin from actual distance.
//  "abs_err" is the absolute error margin from actual distance.  The
//  smaller of the two will be satisfied, so set one large to nullify
//  its effect.
//
//  "qsize" is an optional parameter controlling the size of a priority
//  queue used to direct the search for closest points.  A larger queue
//  can help the algorithm discover the minimum with fewer steps, but
//  will increase the cost of each step. It is not beneficial to increase
//  qsize if the application has frame-to-frame coherence, i.e., the
//  pair of models take small steps between each call, since another
//  speedup trick already accelerates this situation with no overhead.
//
//  However, a queue size of 100 to 200 has been seen to save time in a
//  planning application with "non-coherent" placements of models.
//
//----------------------------------------------------------------------------

// int PQP_Collide(PQPResultPtr result, Eigen::Ref<PQP_EIGEN_MATRIX> R1, Eigen::Ref<PQP_EIGEN_VECTOR> T1, PQPModelPtr
// o1,
//                 Eigen::Ref<PQP_EIGEN_MATRIX> R2, Eigen::Ref<PQP_EIGEN_VECTOR> T2, PQPModelPtr o2,
//                 int flag = PQP_ALL_CONTACTS)

// template <typename PQPDistanceResultPtr, typename PQPModelPtr>
// int PQP_Distance(PQPDistanceResultPtr result, PQP_REAL R1[3][3], PQP_REAL T1[3], PQPModelPtr o1, PQP_REAL R2[3][3],
//                  PQP_REAL T2[3], PQPModelPtr o2, PQP_REAL rel_err, PQP_REAL abs_err, int qsize = 2)
// {}
//----------------------------------------------------------------------------
//
//  PQP_ToleranceResult
//
//  This saves and reports results from a tolerance query.
//
//----------------------------------------------------------------------------
//
//  struct PQP_ToleranceResult - declaration contained in PQP_Internal.h
//  {
//    // statistics
//
//    int NumBVTests();
//    int NumTriTests();
//    PQP_REAL QueryTimeSecs();
//
//    // If the models are closer than ( <= ) tolerance, these points
//    // and distance were what established this.  Otherwise,
//    // distance and point values are not meaningful.
//
//    PQP_REAL Distance();
//    const PQP_REAL *P1();
//    const PQP_REAL *P2();
//
//    // boolean says whether models are closer than tolerance distance
//
//    int CloserThanTolerance();
//  };

//----------------------------------------------------------------------------
//
// PQP_Tolerance() - checks if distance between PQP_Models is <= tolerance
//
//
// Declare a PQP_ToleranceResult and pass its pointer to collect
// tolerance information.
//
// The algorithm returns whether the true distance is <= or >
// "tolerance".  This routine does not simply compute true distance
// and compare to the tolerance - models can often be shown closer or
// farther than the tolerance more trivially.  In most cases this
// query should run faster than a distance query would on the same
// models and configurations.
//
// "qsize" again controls the size of a priority queue used for
// searching.  Not setting qsize is the current recommendation, since
// increasing it has only slowed down our applications.
//
//----------------------------------------------------------------------------

// int PQP_Tolerance(PQP_ToleranceResult* res, PQP_REAL R1[3][3], PQP_REAL T1[3], PQP_Model* o1, PQP_REAL R2[3][3],
//                   PQP_REAL T2[3], PQP_Model* o2, PQP_REAL tolerance, int qsize = 2);

#endif
#endif
