#ifndef PQP_DEFS_H
#define PQP_DEFS_H

enum BUILD_STATE
{
  PQP_BUILD_STATE_EMPTY,     // empty state, immediately after constructor
  PQP_BUILD_STATE_BEGUN,     // after BeginModel(), state for adding triangles
  PQP_BUILD_STATE_PROCESSED  // after tree has been built, ready to use
};

const int PQP_OK = 0;
// Used by all API routines upon successful completion except
// constructors and destructors

const int PQP_ERR_MODEL_OUT_OF_MEMORY = -1;
// Returned when an API function cannot obtain enough memory to
// store or process a PQP_Model object.

const int PQP_ERR_OUT_OF_MEMORY = -2;
// Returned when a PQP query cannot allocate enough storage to
// compute or hold query information.  In this case, the returned
// data should not be trusted.

const int PQP_ERR_UNPROCESSED_MODEL = -3;
// Returned when an unprocessed model is passed to a function which
// expects only processed models, such as PQP_Collide() or
// PQP_Distance().

const int PQP_ERR_BUILD_OUT_OF_SEQUENCE = -4;
// Returned when:
//       1. AddTri() is called before BeginModel().
//       2. BeginModel() is called immediately after AddTri().
// This error code is something like a warning: the invoked
// operation takes place anyway, and PQP does what makes "most
// sense", but the returned error code may tip off the client that
// something out of the ordinary is happenning.

const int PQP_ERR_BUILD_EMPTY_MODEL = -5;
// Returned when EndModel() is called on a model to which no
// triangles have been added.  This is similar in spirit to the
// OUT_OF_SEQUENCE return code, except that the requested operation
// has FAILED -- the model remains "unprocessed", and the client may
// NOT use it in queries.

const int PQP_ALL_CONTACTS = 1;   // find all pairwise intersecting triangles
const int PQP_FIRST_CONTACT = 2;  // report first intersecting tri pair found

#endif