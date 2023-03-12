/* JMATH - Simple Linear Algebra Routines for Embedded MPC 
 * Copyright (C) Jonathan Currie 2012 (I2C2) 
 */

//EMBED START
#include <math.h>

//Dot Product
realT jdot(intT n, const realT* vecA, const realT* vecB)
{
    intT i=n;
    realT x = 0.0;
    while(i--)
        x += *vecA++ * *vecB++;   
    return x;
}

//Matrix * Vector (non-transposed)
void jmv(const intT rows, const intT cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y)
{
    intT i;
    realT *Aptr;    
    for(i = 0; i < rows; i++) {
        realT x = 0.0;
        intT k = cols;
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
void jtmv(const intT rows, const intT cols, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y)
{
    intT i;
	for(i = 0; i < rows; i++){
		realT x = 0.0;
		intT k = cols;
		while(k--)
			x += a * *matA++ * *vecA++;

		y[i] = x + b*y[i];
		vecA-=cols;
	} 
}   

//Matrix (Transposed) * Vector [a*matA^T * vecA + b*y] (selected rows only)
void jtmvs(const intT rows, const intT cols, const intT *idx, const realT a, const realT *matA, const realT *vecA, const realT b, realT *y)
{
    intT i;
	for(i = 0; i < rows; i++){
		realT x = 0.0;   
        if(i==*idx++) { //in selected rows
            intT k = cols;
            while(k--)
                x += a * *matA++ * *vecA++;
            y[i] = x + b*y[i];
            vecA-=cols;
        }
        else //skip other rows
            matA+=cols;
	} 
} 

//Cholesky Decomposition
intT jchol(realT *A, const intT n)
{
    intT i, status = 1;
    realT *A_jn = A;
    for (i=0; i<n; i++) {        
        realT *A_kn = A, s = 0.0;
        intT j;
        for (j=0; j<i; j++) {
            realT t = 0.0;            
            realT *x1 = A_kn, *x2 = A_jn;
            intT  k = j;
            while(k--) 
                t += *x1++ * *x2++;

            t = (A_jn[j] - t) / A_kn[j];
            A_jn[j] = t;
            A_kn[i] = t; //copy
            s += t*t;
            A_kn += n;
        }
        s = A_jn[i] - s;
        if(s <= 0.0) {
            //return -1; //normal Cholesky
            //A_jn[i] = 1000000.0; //replace with larger val
            status = -1; //not positive definite, pivot skipped
        }
        else
            A_jn[i] = sqrt(s);
        A_jn += n;
    }
    return status;
}

//Forward Substitution (assumes A requires transpose i.e. tri solve only)
void jfsub(const realT *A,  realT *x, const intT n)
{
    intT i;
    for(i = 0; i < n; i++) {
        realT s = 0.0, *xptr = x;
        intT k = i;
        while(k--)
            s += *A++ * *xptr++;
        
        x[i] = (x[i] - s)/ *A ;
        A += n-i;
    }
}

//Backward Substitution
void jbsub(const realT *A,  realT *x, const intT n)
{
    intT i, j;
    realT *Aptr;
    A += (n*n)-1; //start at end
    Aptr = (realT*)A;    
    for(i = n-1; i >= 0; i--) {
        realT s = 0.0;
        for(j = n-1; j > i; j--){
            s += *A * x[j];
            A -= n;
        }        
        x[i] = (x[i] - s)/ *A ;
        A = --Aptr;
    }
}

//Triangular Solver
void jtris(const realT *R, realT *x, const intT n)
{
    jfsub(R,x,n); //this function already assumes R'
    jbsub(R,x,n);
}
  
//Cholesky Linear System Solver
intT cholSolve(realT *R, realT *x, const intT n)
{
    intT status = jchol(R, n);
    jfsub(R,x,n); //this function already assumes R'
    jbsub(R,x,n);
    return status;
}


//Form Linear System (A'*diag(lamt)*A), upper tri only
/*void formLinSys(const realT *A, const realT *lamt, const intT ndec, const intT mc, realT *RQP)
{
    intT i, j, k;
    for(i = 0; i < ndec; i++) { //row       
        for(j = i; j < ndec; j++) { //col, only cover upper tri elements
            for(k = 0; k < mc; k++)
            	RQP[i+j*ndec] += A[k + i*mc]*A[k + j*mc]*lamt[k];
        }
    }
}*/

//Form Linear System (A'*diag(lamt)*A), upper tri only
void formLinSys(const realT *A, const realT *lamt, const intT ndec, const intT mc, realT *RQP)
{
    uintT i, j;
    realT *lt = (realT*)lamt;
    realT *Ai = (realT*)A;
    for(i = 0; i < ndec; i++) { //row
        realT *Aj = Ai;
        realT *Rj = &RQP[i+i*ndec];
        for(j = i; j < ndec; j++) { //col, only cover upper tri elements						
            realT sum = *Rj;
            intT k = mc;
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
realT vecmax(const realT *vec, intT len)
{
    intT i;
    realT m = fabs(vec[0]), tmp;
    for(i=1;i<len;i++) {
        tmp = fabs(vec[i]);
        if(tmp > m)
            m = tmp;
    }
    return m;
}

//Maximum Absolute Value in a Vector - Vector
realT vecmmax(const realT *vec, realT *t, intT len)
{
    intT i;
    realT m = fabs(vec[0]-t[0]), tmp;
    for(i=1;i<len;i++) {
        tmp = fabs(vec[i]-t[i]);
        if(tmp > m)
            m = tmp;
    }
    return m;
}