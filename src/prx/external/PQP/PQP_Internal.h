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

#include "Tri.h"
#include "BV.h"
#include "PQP_defs.h"
#include "Build.h"

template <typename Rotation, typename Translation>
class PQP_Model_Base
{
public:
  using RotationType = Rotation;
  using TranslationType = Translation;

  int build_state;

  TriBase<Translation>* tris;
  int num_tris;
  int num_tris_alloced;

  BVBase<Rotation, Translation>* b;
  int num_bvs;
  int num_bvs_alloced;

  TriBase<Translation>* last_tri;  // closest tri on this model in last distance test

  BVBase<Rotation, Translation>* child(int n)
  {
    return &b[n];
  }

  PQP_Model_Base()
    : b(0)
    , num_bvs_alloced(0)
    , num_bvs(0)
    , tris(0)
    , num_tris(0)
    , num_tris_alloced(0)
    , last_tri(0)
    , build_state(PQP_BUILD_STATE_EMPTY)
  {
  }

  ~PQP_Model_Base()
  {
    if (b != NULL)
      delete[] b;
    if (tris != NULL)
      delete[] tris;
  }

  // preallocate for num_tris triangles;
  // the parameter is optional, since
  // arrays are reallocated as needed
  int BeginModel(int num_tris = 8)
  {
    int& n{ num_tris };
    // reset to initial state if necessary

    if (build_state != PQP_BUILD_STATE_EMPTY)
    {
      delete[] b;
      delete[] tris;

      num_tris = num_bvs = num_tris_alloced = num_bvs_alloced = 0;
    }

    // prepare model for addition of triangles

    if (n <= 0)
      n = 8;
    num_tris_alloced = n;
    tris = new TriBase<Translation>[n];
    if (!tris)
    {
      fprintf(stderr,
              "PQP Error!  Out of memory for tri array on "
              "BeginModel() call!\n");
      return PQP_ERR_MODEL_OUT_OF_MEMORY;
    }

    // give a warning if called out of sequence

    if (build_state != PQP_BUILD_STATE_EMPTY)
    {
      fprintf(stderr,
              "PQP Warning! Called BeginModel() on a PQP_Model that \n"
              "was not empty. This model was cleared and previous\n"
              "triangle additions were lost.\n");
      build_state = PQP_BUILD_STATE_BEGUN;
      return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
    }

    build_state = PQP_BUILD_STATE_BEGUN;
    return PQP_OK;
  }

  int AddTri(const Translation& p1, const Translation& p2, const Translation& p3, int id)
  {
    if (build_state == PQP_BUILD_STATE_EMPTY)
    {
      BeginModel();
    }
    else if (build_state == PQP_BUILD_STATE_PROCESSED)
    {
      fprintf(stderr,
              "PQP Warning! Called AddTri() on PQP_Model \n"
              "object that was already ended. AddTri() was\n"
              "ignored.  Must do a BeginModel() to clear the\n"
              "model for addition of new triangles\n");
      return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
    }

    // allocate for new triangles

    if (num_tris >= num_tris_alloced)
    {
      TriBase<Translation>* temp;
      temp = new TriBase<Translation>[num_tris_alloced * 2];
      if (!temp)
      {
        fprintf(stderr,
                "PQP Error!  Out of memory for tri array on"
                " AddTri() call!\n");
        return PQP_ERR_MODEL_OUT_OF_MEMORY;
      }
      memcpy(temp, tris, sizeof(TriBase<Translation>) * num_tris);
      delete[] tris;
      tris = temp;
      num_tris_alloced = num_tris_alloced * 2;
    }

    // initialize the new triangle

    tris[num_tris].p1[0] = p1[0];
    tris[num_tris].p1[1] = p1[1];
    tris[num_tris].p1[2] = p1[2];

    tris[num_tris].p2[0] = p2[0];
    tris[num_tris].p2[1] = p2[1];
    tris[num_tris].p2[2] = p2[2];

    tris[num_tris].p3[0] = p3[0];
    tris[num_tris].p3[1] = p3[1];
    tris[num_tris].p3[2] = p3[2];

    tris[num_tris].id = id;

    num_tris += 1;

    return PQP_OK;
  }

  int EndModel(bool parent_relative = true)
  {
    if (build_state == PQP_BUILD_STATE_PROCESSED)
    {
      fprintf(stderr,
              "PQP Warning! Called EndModel() on PQP_Model \n"
              "object that was already ended. EndModel() was\n"
              "ignored.  Must do a BeginModel() to clear the\n"
              "model for addition of new triangles\n");
      return PQP_ERR_BUILD_OUT_OF_SEQUENCE;
    }

    // report error is no tris

    if (num_tris == 0)
    {
      fprintf(stderr,
              "PQP Error! EndModel() called on model with"
              " no triangles\n");
      return PQP_ERR_BUILD_EMPTY_MODEL;
    }

    // shrink fit tris array

    if (num_tris_alloced > num_tris)
    {
      TriBase<Translation>* new_tris = new TriBase<Translation>[num_tris];
      if (!new_tris)
      {
        fprintf(stderr,
                "PQP Error!  Out of memory for tri array "
                "in EndModel() call!\n");
        return PQP_ERR_MODEL_OUT_OF_MEMORY;
      }
      memcpy(new_tris, tris, sizeof(Tri) * num_tris);
      delete[] tris;
      tris = new_tris;
      num_tris_alloced = num_tris;
    }

    // create an array of BVs for the model

    b = new BVBase<Rotation, Translation>[2 * num_tris - 1];
    if (!b)
    {
      fprintf(stderr,
              "PQP Error! out of memory for BV array "
              "in EndModel()\n");
      return PQP_ERR_MODEL_OUT_OF_MEMORY;
    }
    num_bvs_alloced = 2 * num_tris - 1;
    num_bvs = 0;

    // we should build the model now.

    build_model(this);
    build_state = PQP_BUILD_STATE_PROCESSED;

    last_tri = tris;

    return PQP_OK;
  }

  // returns model mem usage.
  // prints message to stderr if msg == TRUE
  int MemUsage(int msg)
  {
    int mem_bv_list = sizeof(BVBase<Rotation, Translation>) * num_bvs;
    int mem_tri_list = sizeof(TriBase<Translation>) * num_tris;

    int total_mem = mem_bv_list + mem_tri_list + sizeof(PQP_Model_Base<Rotation, Translation>);

    if (msg)
    {
      fprintf(stderr, "Total for model %x: %d bytes\n", this, total_mem);
      fprintf(stderr, "BVs: %d alloced, take %d bytes each\n", num_bvs, sizeof(BV));
      fprintf(stderr, "Tris: %d alloced, take %d bytes each\n", num_tris, sizeof(Tri));
    }

    return total_mem;
  }

  int ScaleModel(float scale_factor)
  {
    if (build_state != PQP_BUILD_STATE_PROCESSED)
    {
      fprintf(stderr, "PQP Error!ScaleModel() called on an incomplete model\n");
      return PQP_ERR_UNPROCESSED_MODEL;
    }

    if (!b)
    {
      fprintf(stderr,
              "PQP Error! out of memory for BV array "
              "in ScaleModel()\n");
      return PQP_ERR_MODEL_OUT_OF_MEMORY;
    }
    num_bvs_alloced = 2 * num_tris - 1;
    num_bvs = 0;

    // Iterate over each triangle and scale it

    for (int i = 0; i < num_tris; ++i)
    {
      PQP_REAL x_centroid = (tris[i].p1[0] + tris[i].p2[0] + tris[i].p3[0]) / 3.0;
      PQP_REAL y_centroid = (tris[i].p1[1] + tris[i].p2[1] + tris[i].p3[1]) / 3.0;
      PQP_REAL z_centroid = (tris[i].p1[2] + tris[i].p2[2] + tris[i].p3[2]) / 3.0;

      // std::cout << "P1[0]: " << tris[i].p1[0] << ", P1[1]: " <<tris[i].p1[1] << ", P1[2]: " << tris[i].p1[2] <<
      // std::endl; std::cout << "P2[0]: " << tris[i].p2[0] << ", P2[1]: " <<tris[i].p2[1] << ", P2[2]: " <<
      // tris[i].p2[2]
      // << std::endl; std::cout << "P3[0]: " << tris[i].p3[0] << ", P3[1]: " <<tris[i].p3[1] << ", P3[2]: " <<
      // tris[i].p3[2] << std::endl;

      tris[i].p1[0] -= x_centroid;
      tris[i].p1[0] *= scale_factor;
      tris[i].p1[0] += x_centroid;

      tris[i].p1[1] -= y_centroid;
      tris[i].p1[1] *= scale_factor;
      tris[i].p1[1] += y_centroid;

      tris[i].p1[2] -= z_centroid;
      tris[i].p1[2] *= scale_factor;
      tris[i].p1[2] += z_centroid;

      tris[i].p2[0] -= x_centroid;
      tris[i].p2[0] *= scale_factor;
      tris[i].p2[0] += x_centroid;

      tris[i].p2[1] -= y_centroid;
      tris[i].p2[1] *= scale_factor;
      tris[i].p2[1] += y_centroid;

      tris[i].p2[2] -= z_centroid;
      tris[i].p2[2] *= scale_factor;
      tris[i].p2[2] += z_centroid;

      tris[i].p3[0] -= x_centroid;
      tris[i].p3[0] *= scale_factor;
      tris[i].p3[0] += x_centroid;

      tris[i].p3[1] -= y_centroid;
      tris[i].p3[1] *= scale_factor;
      tris[i].p3[1] += y_centroid;

      tris[i].p3[2] -= z_centroid;
      tris[i].p3[2] *= scale_factor;
      tris[i].p3[2] += z_centroid;
      // std::cout << "----------POST SCALE" << std::endl;
      // std::cout << "P1[0]: " << tris[i].p1[0] << ", P1[1]: " <<tris[i].p1[1] << ", P1[2]: " << tris[i].p1[2] <<
      // std::endl; std::cout << "P2[0]: " << tris[i].p2[0] << ", P2[1]: " <<tris[i].p2[1] << ", P2[2]: " <<
      // tris[i].p2[2]
      // << std::endl; std::cout << "P3[0]: " << tris[i].p3[0] << ", P3[1]: " <<tris[i].p3[1] << ", P3[2]: " <<
      // tris[i].p3[2] << std::endl;
    }

    // we now rebuild the model

    build_model(this, true);
    build_state = PQP_BUILD_STATE_PROCESSED;

    last_tri = tris;

    return PQP_OK;
  }
};

struct CollisionPair
{
  int id1;
  int id2;
};

template <typename Rotation, typename Translation>
struct PQP_CollideResult_Base
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2
  Rotation R;
  Translation T;
  // PQP_REAL R[3][3];
  // PQP_REAL T[3];

  int num_pairs_alloced;
  int num_pairs;
  CollisionPair* pairs;

  void SizeTo(int n)
  {
    CollisionPair* temp;

    if (n < num_pairs)
    {
      fprintf(stderr,
              "PQP Error: Internal error in "
              "'PQP_CollideResult::SizeTo(int n)'\n");
      fprintf(stderr, "       n = %d, but num_pairs = %d\n", n, num_pairs);
      return;
    }

    temp = new CollisionPair[n];
    memcpy(temp, pairs, num_pairs * sizeof(CollisionPair));
    delete[] pairs;
    pairs = temp;
    num_pairs_alloced = n;
    return;
  }

  void Add(int a, int b)
  {
    if (num_pairs >= num_pairs_alloced)
    {
      // allocate more

      SizeTo(num_pairs_alloced * 2 + 8);
    }

    // now proceed as usual

    pairs[num_pairs].id1 = a;
    pairs[num_pairs].id2 = b;
    num_pairs++;
  }

  PQP_CollideResult_Base()
  {
    pairs = 0;
    num_pairs = num_pairs_alloced = 0;
    num_bv_tests = 0;
    num_tri_tests = 0;
  }

  ~PQP_CollideResult_Base()
  {
    delete[] pairs;
  };

  // statistics

  int NumBVTests()
  {
    return num_bv_tests;
  }
  int NumTriTests()
  {
    return num_tri_tests;
  }
  double QueryTimeSecs()
  {
    return query_time_secs;
  }

  // free the list of contact pairs; ordinarily this list is reused
  // for each query, and only deleted in the destructor.

  void FreePairsList()
  {
    num_pairs = num_pairs_alloced = 0;
    delete[] pairs;
    pairs = 0;
  }

  // query results

  int Colliding()
  {
    return (num_pairs > 0);
  }
  int NumPairs()
  {
    return num_pairs;
  }
  int Id1(int k)
  {
    return pairs[k].id1;
  }
  int Id2(int k)
  {
    return pairs[k].id2;
  }
};
using PQP_Model = PQP_Model_Base<PQP_REAL[3][3], PQP_REAL[3]>;
using PQP_CollideResult = PQP_CollideResult_Base<PQP_REAL[3][3], PQP_REAL[3]>;

#if PQP_BV_TYPE & RSS_TYPE  // distance/tolerance are only available with RSS

template <typename Rotation, typename Translation>
struct PQP_DistanceResult_Base
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  Rotation R;
  Translation T;

  PQP_REAL rel_err;
  PQP_REAL abs_err;

  PQP_REAL distance;
  Translation p1;
  Translation p2;
  int qsize;

  // statistics

  int NumBVTests()
  {
    return num_bv_tests;
  }
  int NumTriTests()
  {
    return num_tri_tests;
  }
  double QueryTimeSecs()
  {
    return query_time_secs;
  }

  // The following distance and points established the minimum distance
  // for the models, within the relative and absolute error bounds
  // specified.
  // Points are defined: PQP_REAL p1[3], p2[3];

  PQP_REAL Distance()
  {
    return distance;
  }
  const Translation& P1()
  {
    return p1;
  }
  const Translation& P2()
  {
    return p2;
  }
};

using PQP_DistanceResult = PQP_DistanceResult_Base<PQP_REAL[3][3], PQP_REAL[3]>;

struct PQP_ToleranceResult
{
  // stats

  int num_bv_tests;
  int num_tri_tests;
  double query_time_secs;

  // xform from model 1 to model 2

  PQP_REAL R[3][3];
  PQP_REAL T[3];

  int closer_than_tolerance;
  PQP_REAL tolerance;

  PQP_REAL distance;
  PQP_REAL p1[3];
  PQP_REAL p2[3];
  int qsize;

  // statistics

  int NumBVTests()
  {
    return num_bv_tests;
  }
  int NumTriTests()
  {
    return num_tri_tests;
  }
  double QueryTimeSecs()
  {
    return query_time_secs;
  }

  // If the models are closer than ( <= ) tolerance, these points
  // and distance were what established this.  Otherwise,
  // distance and point values are not meaningful.

  PQP_REAL Distance()
  {
    return distance;
  }
  const PQP_REAL* P1()
  {
    return p1;
  }
  const PQP_REAL* P2()
  {
    return p2;
  }

  // boolean says whether models are closer than tolerance distance

  int CloserThanTolerance()
  {
    return closer_than_tolerance;
  }
};

#endif