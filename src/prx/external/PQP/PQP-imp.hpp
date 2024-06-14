#pragma once
#include "GetTime.h"
#include "TriDist.h"
#include "BVTQ.h"

inline PQP_REAL max(const PQP_REAL& a, const PQP_REAL& b, const PQP_REAL& c)
{
  PQP_REAL t{ a };
  if (b > t)
    t = b;
  if (c > t)
    t = c;
  return t;
}

inline PQP_REAL min(const PQP_REAL& a, const PQP_REAL& b, const PQP_REAL& c)
{
  PQP_REAL t = a;
  if (b < t)
    t = b;
  if (c < t)
    t = c;
  return t;
}

template <typename Translation>
inline int project6(Translation& ax, Translation& p1, Translation& p2, Translation& p3, Translation& q1,
                    Translation& q2, Translation& q3)
{
  const PQP_REAL P1{ VdotV(ax, p1) };
  const PQP_REAL P2{ VdotV(ax, p2) };
  const PQP_REAL P3{ VdotV(ax, p3) };
  const PQP_REAL Q1{ VdotV(ax, q1) };
  const PQP_REAL Q2{ VdotV(ax, q2) };
  const PQP_REAL Q3{ VdotV(ax, q3) };

  const PQP_REAL mx1{ max(P1, P2, P3) };
  const PQP_REAL mn1{ min(P1, P2, P3) };
  const PQP_REAL mx2{ max(Q1, Q2, Q3) };
  const PQP_REAL mn2{ min(Q1, Q2, Q3) };

  if (mn1 > mx2)
    return 0;
  if (mn2 > mx1)
    return 0;
  return 1;
}

template <typename Translation>
inline int TriContact(Translation& P1, Translation& P2, Translation& P3, Translation& Q1, Translation& Q2,
                      Translation& Q3)
{
  // One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
  // Edges are (e1,e2,e3) and (f1,f2,f3).
  // Normals are n1 and m1
  // Outwards are (g1,g2,g3) and (h1,h2,h3).
  //
  // We assume that the triangle vertices are in the same coordinate system.
  //
  // First thing we do is establish a new c.s. so that p1 is at (0,0,0).

  Translation p1, p2, p3;
  Translation q1, q2, q3;
  Translation e1, e2, e3;
  Translation f1, f2, f3;
  Translation g1, g2, g3;
  Translation h1, h2, h3;
  Translation n1, m1;

  Translation ef11, ef12, ef13;
  Translation ef21, ef22, ef23;
  Translation ef31, ef32, ef33;

  p1[0] = P1[0] - P1[0];
  p1[1] = P1[1] - P1[1];
  p1[2] = P1[2] - P1[2];
  p2[0] = P2[0] - P1[0];
  p2[1] = P2[1] - P1[1];
  p2[2] = P2[2] - P1[2];
  p3[0] = P3[0] - P1[0];
  p3[1] = P3[1] - P1[1];
  p3[2] = P3[2] - P1[2];

  q1[0] = Q1[0] - P1[0];
  q1[1] = Q1[1] - P1[1];
  q1[2] = Q1[2] - P1[2];
  q2[0] = Q2[0] - P1[0];
  q2[1] = Q2[1] - P1[1];
  q2[2] = Q2[2] - P1[2];
  q3[0] = Q3[0] - P1[0];
  q3[1] = Q3[1] - P1[1];
  q3[2] = Q3[2] - P1[2];

  e1[0] = p2[0] - p1[0];
  e1[1] = p2[1] - p1[1];
  e1[2] = p2[2] - p1[2];
  e2[0] = p3[0] - p2[0];
  e2[1] = p3[1] - p2[1];
  e2[2] = p3[2] - p2[2];
  e3[0] = p1[0] - p3[0];
  e3[1] = p1[1] - p3[1];
  e3[2] = p1[2] - p3[2];

  f1[0] = q2[0] - q1[0];
  f1[1] = q2[1] - q1[1];
  f1[2] = q2[2] - q1[2];
  f2[0] = q3[0] - q2[0];
  f2[1] = q3[1] - q2[1];
  f2[2] = q3[2] - q2[2];
  f3[0] = q1[0] - q3[0];
  f3[1] = q1[1] - q3[1];
  f3[2] = q1[2] - q3[2];

  VcrossV(n1, e1, e2);
  VcrossV(m1, f1, f2);

  VcrossV(g1, e1, n1);
  VcrossV(g2, e2, n1);
  VcrossV(g3, e3, n1);
  VcrossV(h1, f1, m1);
  VcrossV(h2, f2, m1);
  VcrossV(h3, f3, m1);

  VcrossV(ef11, e1, f1);
  VcrossV(ef12, e1, f2);
  VcrossV(ef13, e1, f3);
  VcrossV(ef21, e2, f1);
  VcrossV(ef22, e2, f2);
  VcrossV(ef23, e2, f3);
  VcrossV(ef31, e3, f1);
  VcrossV(ef32, e3, f2);
  VcrossV(ef33, e3, f3);

  // now begin the series of tests

  if (!project6(n1, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(m1, p1, p2, p3, q1, q2, q3))
    return 0;

  if (!project6(ef11, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef12, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef13, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef21, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef22, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef23, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef31, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef32, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(ef33, p1, p2, p3, q1, q2, q3))
    return 0;

  if (!project6(g1, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(g2, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(g3, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(h1, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(h2, p1, p2, p3, q1, q2, q3))
    return 0;
  if (!project6(h3, p1, p2, p3, q1, q2, q3))
    return 0;

  return 1;
}

inline PQP_REAL TriDist(Eigen::Ref<PQP_EIGEN_VECTOR> P, Eigen::Ref<PQP_EIGEN_VECTOR> Q, Eigen::Ref<PQP_EIGEN_MATRIX> S,
                        Eigen::Ref<PQP_EIGEN_MATRIX> T)
{
  PQP_REAL Pp[3];
  PQP_REAL Qp[3];
  PQP_REAL Sp[3][3];
  PQP_REAL Tp[3][3];
  copy(Pp, P);
  copy(Qp, Q);
  copy(Sp, S);
  copy(Tp, T);
  const double dist{ TriDist(Pp, Qp, Sp, Tp) };
  copy(P, Pp);
  copy(Q, Qp);
  copy(S, Sp);
  copy(T, Tp);
  return dist;
}

template <typename Rotation, typename Translation>
inline PQP_REAL TriDistance(Rotation& R, Translation& T, TriBase<Translation>* t1, TriBase<Translation>* t2,
                            Translation& p, Translation& q)
{
  // transform tri 2 into same space as tri 1

  Rotation tri1;
  Rotation tri2;

  Translation& p1{ t1->p1 };
  Translation& p2{ t1->p2 };
  Translation& p3{ t1->p3 };

  McolcV(tri1, 0, p1);
  McolcV(tri1, 1, p2);
  McolcV(tri1, 2, p3);
  // VcV(tri1[0], t1->p1);
  // VcV(tri1[1], t1->p2);
  // VcV(tri1[2], t1->p3);
  MxVpV(tri2, 0, R, t2->p1, T);
  MxVpV(tri2, 1, R, t2->p2, T);
  MxVpV(tri2, 2, R, t2->p3, T);

  return TriDist(p, q, tri1, tri2);
}

template <typename PQPCollideResultPtr, typename PQPModelPtr, typename Rotation, typename Translation>
inline void CollideRecurse(PQPCollideResultPtr* res, Rotation& R, Translation& T,  // b2 relative to b1
                           PQPModelPtr o1, int b1, PQPModelPtr o2, int b2, int flag)
{
  // first thing, see if we're overlapping

  res->num_bv_tests++;

  if (!BV_Overlap(R, T, o1->child(b1), o2->child(b2)))
    return;

  // if we are, see if we test triangles next

  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2)
  {
    res->num_tri_tests++;

#if 1
    // transform the points in b2 into space of b1, then compare

    TriBase<Translation>* t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    TriBase<Translation>* t2 = &o2->tris[-o2->child(b2)->first_child - 1];
    Translation q1, q2, q3;
    Translation& p1{ t1->p1 };
    Translation& p2{ t1->p2 };
    Translation& p3{ t1->p3 };
    MxVpV(q1, res->R, t2->p1, res->T);
    MxVpV(q2, res->R, t2->p2, res->T);
    MxVpV(q3, res->R, t2->p3, res->T);
    if (TriContact(p1, p2, p3, q1, q2, q3))
    {
      // add this to result

      res->Add(t1->id, t2->id);
    }
#else
    PQP_REAL p[3], q[3];

    Tri* t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    Tri* t2 = &o2->tris[-o2->child(b2)->first_child - 1];

    if (TriDistance(res->R, res->T, t1, t2, p, q) == 0.0)
    {
      // add this to result

      res->Add(t1->id, t2->id);
    }
#endif

    return;
  }

  // we dont, so decide whose children to visit next

  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();

  Rotation Rc;
  Translation Tc, Ttemp;

  if (l2 || (!l1 && (sz1 > sz2)))
  {
    int c1 = o1->child(b1)->first_child;
    int c2 = c1 + 1;

    MTxM(Rc, o1->child(c1)->R, R);
#if PQP_BV_TYPE & OBB_TYPE
    VmV(Ttemp, T, o1->child(c1)->To);
#else
    VmV(Ttemp, T, o1->child(c1)->Tr);
#endif
    MTxV(Tc, o1->child(c1)->R, Ttemp);
    CollideRecurse(res, Rc, Tc, o1, c1, o2, b2, flag);

    if ((flag == PQP_FIRST_CONTACT) && (res->num_pairs > 0))
      return;

    MTxM(Rc, o1->child(c2)->R, R);
#if PQP_BV_TYPE & OBB_TYPE
    VmV(Ttemp, T, o1->child(c2)->To);
#else
    VmV(Ttemp, T, o1->child(c2)->Tr);
#endif
    MTxV(Tc, o1->child(c2)->R, Ttemp);
    CollideRecurse(res, Rc, Tc, o1, c2, o2, b2, flag);
  }
  else
  {
    int c1 = o2->child(b2)->first_child;
    int c2 = c1 + 1;

    MxM(Rc, R, o2->child(c1)->R);
#if PQP_BV_TYPE & OBB_TYPE
    MxVpV(Tc, R, o2->child(c1)->To, T);
#else
    MxVpV(Tc, R, o2->child(c1)->Tr, T);
#endif
    CollideRecurse(res, Rc, Tc, o1, b1, o2, c1, flag);

    if ((flag == PQP_FIRST_CONTACT) && (res->num_pairs > 0))
      return;

    MxM(Rc, R, o2->child(c2)->R);
#if PQP_BV_TYPE & OBB_TYPE
    MxVpV(Tc, R, o2->child(c2)->To, T);
#else
    MxVpV(Tc, R, o2->child(c2)->Tr, T);
#endif
    CollideRecurse(res, Rc, Tc, o1, b1, o2, c2, flag);
  }
}

template <typename PQPDistanceResultPtr, typename PQPModelPtr, typename Rotation, typename Translation>
inline void DistanceRecurse(PQPDistanceResultPtr res, Rotation& R, Translation& T,  // b2 relative to b1
                            PQPModelPtr o1, int b1, PQPModelPtr o2, int b2)
{
  PQP_REAL sz1 = o1->child(b1)->GetSize();
  PQP_REAL sz2 = o2->child(b2)->GetSize();
  int l1 = o1->child(b1)->Leaf();
  int l2 = o2->child(b2)->Leaf();

  if (l1 && l2)
  {
    // both leaves.  Test the triangles beneath them.

    res->num_tri_tests++;

    Translation p, q;

    TriBase<Translation>* t1 = &o1->tris[-o1->child(b1)->first_child - 1];
    TriBase<Translation>* t2 = &o2->tris[-o2->child(b2)->first_child - 1];

    PQP_REAL d = TriDistance(res->R, res->T, t1, t2, p, q);

    if (d < res->distance)
    {
      res->distance = d;

      VcV(res->p1, p);  // p already in c.s. 1
      VcV(res->p2, q);  // q must be transformed
                        // into c.s. 2 later
      o1->last_tri = t1;
      o2->last_tri = t2;
    }

    return;
  }

  // First, perform distance tests on the children. Then traverse
  // them recursively, but test the closer pair first, the further
  // pair second.

  int a1, a2, c1, c2;  // new bv tests 'a' and 'c'
  Rotation R1, R2;
  Translation T1, T2, Ttemp;

  if (l2 || (!l1 && (sz1 > sz2)))
  {
    // visit the children of b1

    a1 = o1->child(b1)->first_child;
    a2 = b2;
    c1 = o1->child(b1)->first_child + 1;
    c2 = b2;

    MTxM(R1, o1->child(a1)->R, R);
#if PQP_BV_TYPE & RSS_TYPE
    VmV(Ttemp, T, o1->child(a1)->Tr);
#else
    VmV(Ttemp, T, o1->child(a1)->To);
#endif
    MTxV(T1, o1->child(a1)->R, Ttemp);

    MTxM(R2, o1->child(c1)->R, R);
#if PQP_BV_TYPE & RSS_TYPE
    VmV(Ttemp, T, o1->child(c1)->Tr);
#else
    VmV(Ttemp, T, o1->child(c1)->To);
#endif
    MTxV(T2, o1->child(c1)->R, Ttemp);
  }
  else
  {
    // visit the children of b2

    a1 = b1;
    a2 = o2->child(b2)->first_child;
    c1 = b1;
    c2 = o2->child(b2)->first_child + 1;

    MxM(R1, R, o2->child(a2)->R);
#if PQP_BV_TYPE & RSS_TYPE
    MxVpV(T1, R, o2->child(a2)->Tr, T);
#else
    MxVpV(T1, R, o2->child(a2)->To, T);
#endif

    MxM(R2, R, o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
    MxVpV(T2, R, o2->child(c2)->Tr, T);
#else
    MxVpV(T2, R, o2->child(c2)->To, T);
#endif
  }

  res->num_bv_tests += 2;

  PQP_REAL d1 = BV_Distance(R1, T1, o1->child(a1), o2->child(a2));
  PQP_REAL d2 = BV_Distance(R2, T2, o1->child(c1), o2->child(c2));

  if (d2 < d1)
  {
    if ((d2 < (res->distance - res->abs_err)) || (d2 * (1 + res->rel_err) < res->distance))
    {
      DistanceRecurse(res, R2, T2, o1, c1, o2, c2);
    }

    if ((d1 < (res->distance - res->abs_err)) || (d1 * (1 + res->rel_err) < res->distance))
    {
      DistanceRecurse(res, R1, T1, o1, a1, o2, a2);
    }
  }
  else
  {
    if ((d1 < (res->distance - res->abs_err)) || (d1 * (1 + res->rel_err) < res->distance))
    {
      DistanceRecurse(res, R1, T1, o1, a1, o2, a2);
    }

    if ((d2 < (res->distance - res->abs_err)) || (d2 * (1 + res->rel_err) < res->distance))
    {
      DistanceRecurse(res, R2, T2, o1, c1, o2, c2);
    }
  }
}

template <typename PQPDistanceResultPtr, typename PQPModelPtr, typename Rotation, typename Translation>
inline void DistanceQueueRecurse(PQPDistanceResultPtr res, Rotation& R, Translation& T, PQPModelPtr o1, int b1,
                                 PQPModelPtr o2, int b2)
{
  BVTQBase<Rotation, Translation> bvtq(res->qsize);

  BVTBase<Rotation, Translation> min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;
  McM(min_test.R, R);
  VcV(min_test.T, T);

  while (1)
  {
    int l1 = o1->child(min_test.b1)->Leaf();
    int l2 = o2->child(min_test.b2)->Leaf();

    if (l1 && l2)
    {
      // both leaves.  Test the triangles beneath them.

      res->num_tri_tests++;

      Translation p, q;

      TriBase<Translation>* t1 = &o1->tris[-o1->child(min_test.b1)->first_child - 1];
      TriBase<Translation>* t2 = &o2->tris[-o2->child(min_test.b2)->first_child - 1];

      PQP_REAL d = TriDistance(res->R, res->T, t1, t2, p, q);

      if (d < res->distance)
      {
        res->distance = d;

        VcV(res->p1, p);  // p already in c.s. 1
        VcV(res->p2, q);  // q must be transformed
                          // into c.s. 2 later
        o1->last_tri = t1;
        o2->last_tri = t2;
      }
    }
    else if (bvtq.GetNumTests() == bvtq.GetSize() - 1)
    {
      // queue can't get two more tests, recur

      DistanceQueueRecurse(res, min_test.R, min_test.T, o1, min_test.b1, o2, min_test.b2);
    }
    else
    {
      // decide how to descend to children

      PQP_REAL sz1 = o1->child(min_test.b1)->GetSize();
      PQP_REAL sz2 = o2->child(min_test.b2)->GetSize();

      res->num_bv_tests += 2;

      BVTBase<Rotation, Translation> bvt1, bvt2;
      Translation Ttemp;

      if (l2 || (!l1 && (sz1 > sz2)))
      {
        // put new tests on queue consisting of min_test.b2
        // with children of min_test.b1

        int c1 = o1->child(min_test.b1)->first_child;
        int c2 = c1 + 1;

        // init bv test 1

        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        MTxM(bvt1.R, o1->child(c1)->R, min_test.R);
#if PQP_BV_TYPE & RSS_TYPE
        VmV(Ttemp, min_test.T, o1->child(c1)->Tr);
#else
        VmV(Ttemp, min_test.T, o1->child(c1)->To);
#endif
        MTxV(bvt1.T, o1->child(c1)->R, Ttemp);
        bvt1.d = BV_Distance(bvt1.R, bvt1.T, o1->child(bvt1.b1), o2->child(bvt1.b2));

        // init bv test 2

        bvt2.b1 = c2;
        bvt2.b2 = min_test.b2;
        MTxM(bvt2.R, o1->child(c2)->R, min_test.R);
#if PQP_BV_TYPE & RSS_TYPE
        VmV(Ttemp, min_test.T, o1->child(c2)->Tr);
#else
        VmV(Ttemp, min_test.T, o1->child(c2)->To);
#endif
        MTxV(bvt2.T, o1->child(c2)->R, Ttemp);
        bvt2.d = BV_Distance(bvt2.R, bvt2.T, o1->child(bvt2.b1), o2->child(bvt2.b2));
      }
      else
      {
        // put new tests on queue consisting of min_test.b1
        // with children of min_test.b2

        int c1 = o2->child(min_test.b2)->first_child;
        int c2 = c1 + 1;

        // init bv test 1

        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        MxM(bvt1.R, min_test.R, o2->child(c1)->R);
#if PQP_BV_TYPE & RSS_TYPE
        MxVpV(bvt1.T, min_test.R, o2->child(c1)->Tr, min_test.T);
#else
        MxVpV(bvt1.T, min_test.R, o2->child(c1)->To, min_test.T);
#endif
        bvt1.d = BV_Distance(bvt1.R, bvt1.T, o1->child(bvt1.b1), o2->child(bvt1.b2));

        // init bv test 2

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        MxM(bvt2.R, min_test.R, o2->child(c2)->R);
#if PQP_BV_TYPE & RSS_TYPE
        MxVpV(bvt2.T, min_test.R, o2->child(c2)->Tr, min_test.T);
#else
        MxVpV(bvt2.T, min_test.R, o2->child(c2)->To, min_test.T);
#endif
        bvt2.d = BV_Distance(bvt2.R, bvt2.T, o1->child(bvt2.b1), o2->child(bvt2.b2));
      }

      bvtq.AddTest(bvt1);
      bvtq.AddTest(bvt2);
    }

    if (bvtq.Empty())
    {
      break;
    }
    else
    {
      min_test = bvtq.ExtractMinTest();

      if ((min_test.d + res->abs_err >= res->distance) && ((min_test.d * (1 + res->rel_err)) >= res->distance))
      {
        break;
      }
    }
  }
}