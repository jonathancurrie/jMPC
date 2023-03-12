// Naive Linear Algebra Routines
// Jonathan Currie 09-13

#include "jmath.h"
#include <math.h>
#ifdef SINGLE_PREC
    #define sqrt(x) sqrtf(x)
    #define fabs(x) fabsf(x)
#endif

//Dot Product
realT jdot(const int n, const realT* vecA, const realT* vecB)
{
    int i=n;
    realT x = 0.0;
    while(i--)
        x += *vecA++ * *vecB++;   
    return x;
}

//Matrix * Vector (non-transposed)
void jmv(const int rows, const int cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y)
{
    int i;
    realT *Aptr;    
    for(i = 0; i < rows; i++) {
        realT x = 0.0;
        int k = cols;
        Aptr = (realT*)matA;
        while(k--) {
            x += a * *Aptr * *vecA++;
            Aptr += rows;
        }
        y[i] = x + b*y[i];
		matA++;
		vecA-=cols;
    }
}

//Matrix (Transposed) * Vector [a*matA^T * vecA + b*y]
void jtmv(const int rows, const int cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y)
{
    int i;
	for(i = 0; i < rows; i++){
		realT x = 0.0;
		int k = cols;
		while(k--)
			x += a * *matA++ * *vecA++;

		y[i] = x + b*y[i];
		vecA-=cols;
	} 
}

//Cholesky Decomposition 
int jchol(realT *A, const int n)
{
    int j,k,status=1;
    realT s = 0.0, *A_jn = A, *A_kn = A;

    for (j=0; j<n; j++) {
        for (k=0; k<j; k++) {
            realT t = 0.0;
            {
                /* Inner product */
                realT *x1 = A_kn;
                realT *x2 = A_jn;
                int  kk = k;
                while(kk--) {
                    t += *x1++ * *x2++;
                }
            }
            t       = (A_jn[k] - t) / A_kn[k];
            A_jn[k] = t;
            s      += t*t;
            A_kn   += n;
        }
        s  = A_jn[j] - s;
        if(s <= 0.0) {
            //return -1; //normal Cholesky
            //A_jn[j] = 1000000.0; //replace with larger val
            status = -1; //not positive definite, pivot skipped
        }
        else
            A_jn[j] = sqrt(s);
        A_jn   += n;
        s = 0.0;
        A_kn = A;
    }
    return status;
}

//Forward Substitution (assumes A requires transpose i.e. tri solve only)
void jfsub(const realT *A,  realT *x, const int n)
{
    int i, j;
    realT s;
    
    for(i = 0; i < n; i++) {
        s = 0.0;
        for(j = 0; j < i; j++)
            s += *A++ * x[j];
        
        x[i] = (x[i] - s)/ *A ;
        A += n-i;
    }
}

//Backward Substitution
void jbsub(const realT *A,  realT *x, const int n)
{
    int i, j;
    realT s;
    realT *Aptr;
    A += (n*n)-1; //start at end
    Aptr = (realT*)A;
    
    for(i = n-1; i >= 0; i--) {
        s = 0.0;
        for(j = n-1; j > i; j--){
            s += *A * x[j];
            A -= n;
        }
        
        x[i] = (x[i] - s)/ *A ;
        A = --Aptr;
    }
}
  
//Cholesky Linear System Solver
int cholSolve(realT *R, realT *x, const int n)
{
    int status = jchol(R, n);
    jfsub(R,x,n); //this function already assumes R'
    jbsub(R,x,n);
    return status;
}

//Triangular System Solver
void jtris(const realT *R, realT *x, const int n)
{
    jfsub(R,x,n); //this function already assumes R'
    jbsub(R,x,n);
}

//Form Linear System (A'*diag(lamt)*A), upper tri only
void formLinSys(const realT *A, const realT *lamt, const int ndec, const int mc, realT *RQP)
{
    int i, j;
    realT *lt = (realT*)lamt;
    realT *Ai = (realT*)A;
    for(i = 0; i < ndec; i++) { //row
        realT *Aj = Ai;
        realT *Rj = &RQP[i+i*ndec];
        for(j = i; j < ndec; j++) { //col, only cover upper tri elements						
            realT sum = *Rj;
            int k = mc;
            while(k--)
				sum += *Ai++**Aj++**lt++;
            
            *Rj = sum;
            Rj += ndec;
            Ai -= mc;
            lt -= mc;
        }
        Ai += mc;
    }
}

//Maximum Absolute Value in a Vector
realT vecmax(const realT *vec, int len)
{
    int i;
    realT m = fabs(vec[0]), tmp;
    for(i=1;i<len;i++) {
        tmp = fabs(vec[i]);
        if(tmp > m)
            m = tmp;
    }
    return m;
}

//Maximum Absolute Value in a Vector - Vector
realT vecmmax(const realT *vec, realT *t, int len)
{
    int i;
    realT m = fabs(vec[0]-t[0]), tmp;
    for(i=1;i<len;i++) {
        tmp = fabs(vec[i]-t[i]);
        if(tmp > m)
            m = tmp;
    }
    return m;
}