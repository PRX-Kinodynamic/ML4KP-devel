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

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#ifndef PQP_BVTQ_H
#define PQP_BVTQ_H

#include <stdio.h>
#include <stdlib.h>
#include "PQP_Compile.h"

inline int LChild(int p)
{
  return (2 * p + 1);
}

inline int Parent(int c)
{
  return ((c - 1) / 2);
}

template <typename Rotation, typename Translation>
struct BVTBase
{
  PQP_REAL d;     // distance between the bvs
  int b1, b2;     // bv numbers - b1 is from model 1, b2 from model 2
  Rotation R;     // the relative rotation from b1 to b2
  Translation T;  // the relative translation from b1 to b2
  int pindex;     // the index of the pointer that points to this -
                  // needed when filling the hole left by an ExtractMin
};
using BVT = BVTBase<PQP_REAL[3][3], PQP_REAL[3]>;
// BVTBase<Rotation, Translation>;

template <typename Rotation, typename Translation>
class BVTQBase
{
  int size;                               // max number of bv tests
  int numtests;                           // number of bv tests in queue
  BVTBase<Rotation, Translation>* bvt;    // an array of bv tests - seems faster than 'new' for each
  BVTBase<Rotation, Translation>** bvtp;  // the queue: an array of pointers to elts of bvt

public:
  BVTQBase(int sz)
  {
    size = sz;
    bvt = new BVTBase<Rotation, Translation>[size];
    bvtp = new BVTBase<Rotation, Translation>*[size];
    numtests = 0;
  }
  ~BVTQBase()
  {
    delete[] bvt;
    delete[] bvtp;
  }
  int Empty()
  {
    return (numtests == 0);
  }
  int GetNumTests()
  {
    return numtests;
  }
  int GetSize()
  {
    return size;
  }
  PQP_REAL MinTest()
  {
    return bvtp[0]->d;
  }
  BVTBase<Rotation, Translation> ExtractMinTest();
  void AddTest(BVTBase<Rotation, Translation>&);
};

using BVTQ = BVTQBase<PQP_REAL[3][3], PQP_REAL[3]>;

template <typename Rotation, typename Translation>
inline void BVTQBase<Rotation, Translation>::AddTest(BVTBase<Rotation, Translation>& t)
{
  bvtp[numtests] = &bvt[numtests];

  *bvtp[numtests] = t;
  bvtp[numtests]->pindex = numtests;

  BVTBase<Rotation, Translation>* temp;
  int c = numtests;
  int p;

  while ((c != 0) && (bvtp[(p = Parent(c))]->d >= bvtp[c]->d))
  {
    // swap p and c pointers

    temp = bvtp[p];
    bvtp[p] = bvtp[c];
    bvtp[c] = temp;

    // the bv tests pointed to by p and c need new indices

    bvtp[p]->pindex = p;
    bvtp[c]->pindex = c;

    c = p;
  }
  numtests++;
}

template <typename Rotation, typename Translation>
inline BVTBase<Rotation, Translation> BVTQBase<Rotation, Translation>::ExtractMinTest()
{
  // store min test to be extracted

  BVTBase<Rotation, Translation> min_test = *bvtp[0];

  // copy last bvt to the empty space;
  // reset the pointer to this moved bvt

  *bvtp[0] = bvt[numtests - 1];
  bvtp[bvt[numtests - 1].pindex] = bvtp[0];

  // copy the last pointer to the first

  bvtp[0] = bvtp[numtests - 1];

  numtests--;

  BVTBase<Rotation, Translation>* temp;
  int p = 0;
  int c1, c2, c;

  while (1)
  {
    c1 = LChild(p);
    c2 = c1 + 1;

    if (c1 < numtests)
    {
      if (c2 < numtests)
      {
        // p has both children, promote the minimum

        if (bvtp[c1]->d < bvtp[c2]->d)
          c = c1;
        else
          c = c2;

        if (bvtp[c]->d < bvtp[p]->d)
        {
          temp = bvtp[p];
          bvtp[p] = bvtp[c];
          bvtp[c] = temp;

          bvtp[p]->pindex = p;
          bvtp[c]->pindex = c;

          p = c;
        }
        else
        {
          break;
        }
      }
      else
      {
        // p has only left child

        if (bvtp[c1]->d < bvtp[p]->d)
        {
          temp = bvtp[p];
          bvtp[p] = bvtp[c1];
          bvtp[c1] = temp;

          bvtp[p]->pindex = p;
          bvtp[c1]->pindex = c1;

          p = c1;
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      // p has no children

      break;
    }
  }

  return min_test;
}

#endif
