// Linear Algebra Routines
// Jonathan Currie 09-13

#include "qp_header.h"

//Dot Product
realT jdot(const int n, const realT* vecA, const realT* vecB);
//Matrix * Vector (non-transposed)
void jmv(const int rows, const int cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y);
//Matrix * Vector (transposed)
void jtmv(const int rows, const int cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y);
//Form Linear System
void formLinSys(const realT *A, const realT *lamt, const int ndec, const int mc, realT *RQP);
//Solve Linear System using Cholesky Decomposition
int cholSolve(realT *R, realT *x, const int n);
//Solver Triangular Linear System
void jtris(const realT *R, realT *x, const int n);
//Cholesky Decomposition
int jchol(realT *A, const int n);
//Forward Substitution
void jfsub(const realT *A,  realT *x, const int n);
//Backward Substitution
void jbsub(const realT *A,  realT *x, const int n);
//Maximum Absolute Value in a Vector
realT vecmax(const realT *vec, int len);
realT vecmmax(const realT *vec, realT *t, int len);

/* USEFUL MACROS */
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))